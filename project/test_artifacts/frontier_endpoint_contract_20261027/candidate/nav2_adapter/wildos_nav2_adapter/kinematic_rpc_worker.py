"""Run the synchronous BeamNG RPC away from the ROS executor thread."""
from copy import deepcopy
import threading
import time


class KinematicRpcWorker:
    """Serialize BeamNG kinematic calls and expose a nonblocking health snapshot."""

    def __init__(self, exchange, stop_rpc, interval_sec=0.05, healthy_age_sec=0.5):
        if interval_sec <= 0 or healthy_age_sec <= 0:
            raise ValueError("RPC intervals must be positive")
        self._exchange = exchange
        self._stop_rpc = stop_rpc
        self._interval_sec = float(interval_sec)
        self._healthy_age_sec = float(healthy_age_sec)
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._stopping = False
        self._active = True
        self._retry_open = False
        self._command = (0.0, 0.0)
        self._data = None
        self._last_success_at = None
        self._last_success_wall_time = None
        self._last_error = None
        self._thread = None

    def start(self):
        with self._lock:
            if self._thread is not None:
                raise RuntimeError("RPC worker already started")
            self._thread = threading.Thread(
                target=self._run, name="beamng-kinematic-rpc", daemon=True)
            self._thread.start()

    def set_command(self, linear, angular):
        with self._lock:
            if self._stopping:
                return
            self._command = (float(linear), float(angular))
        self._wake.set()

    def set_active(self, active):
        """Enable polling only while the caller has fresh source state.

        A failed BeamNG RPC opens a circuit breaker. It is retried only after
        the source state stream recovers or the caller requests an explicit
        retry, preventing a dead TCom endpoint from accumulating sockets.
        """
        active = bool(active)
        with self._lock:
            if self._stopping:
                return
            was_active = self._active
            self._active = active
            if active and not was_active:
                self._retry_open = False
                self._last_error = None
                self._data = None
                self._last_success_at = None
                self._last_success_wall_time = None
            elif not active:
                self._data = None
                self._last_success_at = None
                self._last_success_wall_time = None
        self._wake.set()

    def retry(self):
        """Retry one failed session after an explicit operator action."""
        with self._lock:
            if self._stopping:
                return
            self._retry_open = False
            self._last_error = None
            self._data = None
            self._last_success_at = None
            self._last_success_wall_time = None
        self._wake.set()

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else float(now)
        with self._lock:
            age = (None if self._last_success_at is None else
                   max(0.0, now - self._last_success_at))
            healthy = (self._active and age is not None and age <= self._healthy_age_sec
                       and self._last_error is None)
            return dict(data=deepcopy(self._data), telemetry_age_sec=age,
                        telemetry_ok=healthy, actuator_ok=healthy,
                        error=self._last_error,
                        rpc_wall_time=self._last_success_wall_time,
                        active=self._active, retry_open=self._retry_open)

    def close(self, timeout_sec=4.5):
        with self._lock:
            if self._thread is None:
                return True
            self._stopping = True
            thread = self._thread
        self._wake.set()
        thread.join(max(0.0, float(timeout_sec)))
        return not thread.is_alive()

    def _run(self):
        retry_delay = self._interval_sec
        while True:
            with self._lock:
                stopping = self._stopping
                command = self._command
                active = self._active
                retry_open = self._retry_open
            if stopping:
                try:
                    self._stop_rpc()
                except Exception as exc:
                    with self._lock:
                        self._last_error = f"{type(exc).__name__}: {exc}"[:240]
                return

            if not active or retry_open:
                self._wake.wait()
                self._wake.clear()
                continue

            try:
                data = self._exchange(*command)
                if not isinstance(data, dict):
                    raise TypeError("BeamNG kinematic status must be a JSON object")
                completed_at = time.monotonic()
                with self._lock:
                    if self._active and not self._retry_open:
                        self._data = data
                        self._last_success_at = completed_at
                        self._last_success_wall_time = time.time()
                        self._last_error = None
                retry_delay = self._interval_sec
            except Exception as exc:
                with self._lock:
                    if self._active:
                        self._last_error = f"{type(exc).__name__}: {exc}"[:240]
                        self._retry_open = True

            self._wake.wait(retry_delay)
            self._wake.clear()
