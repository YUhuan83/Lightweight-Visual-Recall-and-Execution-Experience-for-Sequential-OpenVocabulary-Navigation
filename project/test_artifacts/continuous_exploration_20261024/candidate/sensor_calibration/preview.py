"""Main-thread GUI, one sequential sensor worker; no simulator calls in the GUI."""
import os
import signal
import threading
import time
import uuid
import numpy as np
from build import write_json


def preflight():
    message = '--preview requires GUI OpenCV and an interactive desktop (Linux: DISPLAY/Wayland); headless is unsupported'
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError(message) from exc
    gui = next((s.split(':', 1)[1].strip() for s in cv2.getBuildInformation().splitlines()
                if s.strip().startswith('GUI:')), 'NONE')
    if gui in ('NONE', 'NO', '') or (os.name != 'nt' and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))):
        raise RuntimeError(message)
    try:
        cv2.namedWindow('Sensor preview GUI check', cv2.WINDOW_NORMAL)
        cv2.waitKey(1)
        cv2.destroyWindow('Sensor preview GUI check')
    except cv2.error as exc:
        raise RuntimeError(message) from exc
    return cv2


def imu_measurements(reading):
    """Keep finite source measurements only, never world pose or derived values."""
    result = []
    for record in reading if isinstance(reading, list) else []:
        if not isinstance(record, dict):
            continue
        measured = {}
        for key in ('time', 'mass', 'accRaw', 'accSmooth', 'angVel', 'angVelSmooth'):
            if key not in record:
                continue
            value = record[key]
            try:
                values = [value[axis] for axis in ('x', 'y', 'z')] if isinstance(value, dict) else value
                array = np.asarray(values, dtype=float)
                shape = () if key in ('time', 'mass') else (3,)
                if array.shape == shape and np.isfinite(array).all():
                    measured[key] = value
            except (KeyError, TypeError, ValueError):
                continue
        if 'accRaw' in measured and 'angVel' in measured:
            result.append(measured)
    return result


def sample_ready(name, reading):
    """Missing initial/cached buffers are not renderable sensor frames."""
    if name == 'imu':
        return bool(imu_measurements(reading))
    if not isinstance(reading, dict):
        return False
    if name != 'lidar':
        image = reading.get('colour')
        return image is not None and callable(getattr(image, 'convert', None))
    cloud = reading.get('pointCloud')
    if cloud is None:
        return False
    points = np.asarray(cloud)
    return points.ndim == 2 and points.shape[1] == 3 and len(points) > 0


def lidar_xy(cloud, origin, frame):
    cloud = np.asarray(cloud)
    origin = np.asarray(origin, dtype=float)
    if cloud.ndim != 2 or cloud.shape[1] != 3 or origin.shape != (3,) or not np.isfinite(origin).all():
        raise ValueError('Malformed cloud or queried origin')
    finite = np.isfinite(cloud).all(axis=1)
    points = cloud[finite].astype(float)
    if frame == 'raw_world':
        points -= origin
    elif frame != 'translated_local_world_axes':
        raise ValueError('Unknown frame hypothesis')
    ranges = np.linalg.norm(points, axis=1)
    valid = (ranges > .05) & (ranges <= 60.1)
    return points[valid, :2], dict(total=len(cloud), finite=int(finite.sum()), displayed=int(valid.sum()),
                                  rejected=int(len(cloud)-valid.sum()), frame_hypothesis=frame,
                                  queried_origin=origin.tolist(), range_m=[.05, 60.1])


def format_frame(cv2, name, sample, calibrated, frame):
    reading, meta, fps = sample
    counts = {}
    lines = [name + (' | FIXTURE CHECKS PASSED' if calibrated else ' | NOT CALIBRATED'),
             'Host acquisition FPS: ' + ('warming up' if fps is None else f'{fps:.2f}') +
             f' | age {max(0, time.monotonic()-meta["host_monotonic_end"]):.1f}s | NOT simulation time',
             'q/Esc/close: exit | S: save latest raw samples (not synchronized)']
    if name == 'imu':
        measurements = imu_measurements(reading)
        image = np.zeros((300, 960, 3), np.uint8)
        texts = [f'Finite accel/gyro samples: {len(measurements)} / {len(reading)} in latest poll (not cumulative)']
        if measurements:
            last = measurements[-1]
            texts.append('Source time (s): ' + (str(last['time']) if 'time' in last else 'unavailable'))
            for key, unit in (('accRaw', 'm/s^2'), ('angVel', 'rad/s')):
                value = last[key]
                values = [value[axis] for axis in ('x', 'y', 'z')] if isinstance(value, dict) else value
                texts.append(key + ' (' + unit + '): ' + ', '.join(f'{v:.6g}' for v in values))
        else:
            texts.append('Waiting for finite actual accel/gyro measurements; no replacement values')
        texts.append('Sensor local axes | gravity enabled | latest valid record in returned order')
        for i, text in enumerate(texts):
            cv2.putText(image, text, (10,35+40*i), cv2.FONT_HERSHEY_SIMPLEX, .5, (220,220,220), 1)
    elif name != 'lidar':
        image = cv2.cvtColor(np.asarray(reading['colour'].convert('RGB')), cv2.COLOR_RGB2BGR)
    else:
        frames = [frame] if frame in ('raw_world', 'translated_local_world_axes') else ['raw_world', 'translated_local_world_axes']
        panels = []
        for hypothesis in frames:
            xy, info = lidar_xy(reading['pointCloud'], meta['pose_before']['position'], hypothesis)
            counts[hypothesis] = info
            panel = np.zeros((640, 640, 3), np.uint8)
            pixels = np.rint(xy * [4.5, -4.5] + [320, 320]).astype(int)
            panel[pixels[:, 1], pixels[:, 0]] = (80, 230, 120)
            cv2.arrowedLine(panel, (320,320), (610,320), (170,170,170), 1)
            cv2.arrowedLine(panel, (320,320), (320,55), (170,170,170), 1)
            for text, pos in [(hypothesis, (8,20)), ('world +X', (535,345)), ('world +Y', (330,65)),
                              ('queried sensor origin', (325,370)),
                              (f'points {info["displayed"]}/{info["total"]}; 60.1m range', (8,605)),
                              (str(info['queried_origin']), (8,630))]:
                cv2.putText(panel, text, pos, cv2.FONT_HERSHEY_SIMPLEX, .45, (220,220,220), 1)
            panels.append(panel)
        image = np.concatenate(panels, axis=1)
        lines.append('XY world axes; frame ' + frame + ' | hypotheses, not verified extrinsics')
    banner = np.zeros((110, image.shape[1], 3), np.uint8)
    for i, line in enumerate(lines):
        cv2.putText(banner, line, (10,22+25*i), cv2.FONT_HERSHEY_SIMPLEX, .48,
                    (80,220,100) if calibrated else (70,180,255), 1)
    return np.concatenate((banner, image), axis=0), counts


def save_samples(output, latest, frame, progress):
    if not latest:
        progress('No actual samples available to save yet')
        return
    destination = output/'preview'/uuid.uuid4().hex
    destination.mkdir(parents=True)
    for name, (reading, meta, fps) in latest.items():
        counts = {}
        if name == 'imu':
            measurements = imu_measurements(reading)
            write_json(destination/'imu.json', measurements)
            write_json(destination/'imu_metadata.json', dict(metadata=meta, host_acquisition_fps=fps,
                       received_samples=len(reading), saved_samples=len(measurements), synchronized=False,
                       scope='Source measurement fields only; world pose and nonfinite/malformed fields excluded'))
            continue
        if name == 'lidar':
            np.save(destination/'lidar.npy', reading['pointCloud'], allow_pickle=False)
            for hypothesis in ('raw_world', 'translated_local_world_axes'):
                _, counts[hypothesis] = lidar_xy(reading['pointCloud'], meta['pose_before']['position'], hypothesis)
        else:
            reading['colour'].save(destination/(name+'.png'))
        write_json(destination/(name+'.json'), dict(metadata=meta, host_acquisition_fps=fps,
                   frame_observed=frame if name == 'lidar' else None, display_counts=counts,
                   raw_unfiltered=True, synchronized=False))
    progress('Snapshot: ' + str(destination))


def run_preview(cv2, sensors, adhoc, timeout, seconds, output, report, state, progress):
    stop = threading.Event()
    lock = threading.Lock()
    latest, errors = {}, []
    windows = []
    handlers = {}
    frame = report['checks'].get('lidar', {}).get('frame_observed', 'unresolved')

    def worker():
        previous = {}
        last_valid = {name: time.monotonic() for name in sensors}
        waiting = set()
        try:
            while not stop.is_set():
                cycle_start = time.monotonic()
                for name, sensor in sensors.items():
                    if stop.is_set():
                        return
                    reading, meta = adhoc(sensor, timeout, silent=True, stop=stop)
                    if not sample_ready(name, reading):
                        if name not in waiting:
                            progress(f'{name}: waiting for a valid frame; last valid frame retained if available')
                            waiting.add(name)
                        if time.monotonic() - last_valid[name] >= timeout:
                            raise TimeoutError(f'{name}: no valid frame for {timeout:g}s')
                        continue
                    last_valid[name] = time.monotonic()
                    if name in waiting:
                        progress(f'{name}: valid frames resumed')
                        waiting.remove(name)
                    now = meta['host_monotonic_end']
                    fps = 1/max(now-previous[name], 1e-9) if name in previous else None
                    previous[name] = now
                    with lock:
                        latest[name] = (reading, meta, fps)
                stop.wait(max(.01, .2 - (time.monotonic()-cycle_start)))
        except InterruptedError:
            if not stop.is_set():
                with lock:
                    errors.append('Unexpected interrupted sensor request')
        except Exception as exc:
            with lock:
                errors.append(type(exc).__name__ + ': ' + str(exc))

    def on_signal(signum, unused):
        state['exit_reason'] = 'signal ' + str(signum)
        stop.set()

    thread = threading.Thread(target=worker, name='sensor-preview', daemon=True)
    started = False
    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            handlers[sig] = signal.signal(sig, on_signal)
        for name in sensors:
            title = 'Sensor calibration: ' + name
            cv2.namedWindow(title, cv2.WINDOW_NORMAL)
            windows.append(title)
            waiting = np.zeros((160, 960, 3), np.uint8)
            cv2.putText(waiting, name + ': waiting for actual sample | NOT CALIBRATED | q/Esc: exit',
                        (10,80), cv2.FONT_HERSHEY_SIMPLEX, .6, (70,180,255), 1)
            cv2.imshow(title, waiting)
        thread.start()
        started = True
        start = time.monotonic()
        displayed = {}
        while not stop.is_set():
            with lock:
                snapshots, failures = dict(latest), list(errors)
            if failures:
                state['error'] = failures[0]
                raise RuntimeError('Preview acquisition failed: ' + failures[0])
            # Check before imshow: imshow can recreate a window the user closed.
            try:
                closed = any(cv2.getWindowProperty(w, cv2.WND_PROP_VISIBLE) < 1 for w in windows)
            except cv2.error:
                closed = True
            if closed:
                state['exit_reason'] = 'window closed'
                break
            for name, sample in snapshots.items():
                stamp = sample[1]['host_monotonic_end']
                if displayed.get(name) == stamp:
                    # Refresh the age banner occasionally even when data stalls.
                    if time.monotonic() - displayed.get((name, 'redraw'), 0) < 1:
                        continue
                image, _ = format_frame(cv2, name, sample, report['calibrated'], frame)
                cv2.imshow('Sensor calibration: ' + name, image)
                displayed[name] = stamp
                displayed[(name, 'redraw')] = time.monotonic()
            key = cv2.waitKey(15) & 255
            if key in (27, ord('q'), ord('Q')):
                state['exit_reason'] = 'key'
                break
            if key in (ord('s'), ord('S')):
                save_samples(output, snapshots, frame, progress)
            if seconds and time.monotonic()-start >= seconds:
                state['exit_reason'] = 'duration'
                break
    finally:
        stop.set()
        for title in windows:
            try:
                cv2.destroyWindow(title)
            except cv2.error:
                pass
        if started:
            progress(f'Windows closed; waiting up to {timeout+1:g}s for sensor worker before cleanup')
            thread.join(timeout+1)
        state['worker_alive'] = thread.is_alive()
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
