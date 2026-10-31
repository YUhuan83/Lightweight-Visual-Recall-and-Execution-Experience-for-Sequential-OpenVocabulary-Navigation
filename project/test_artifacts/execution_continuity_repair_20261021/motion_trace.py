"""Opt-in field observer; captures actual admission inputs, never edits messages."""
from pathlib import Path
import atexit
import hashlib
import json
import os
import time


def install():
    from rclpy.serialization import serialize_message
    from wildos_nav2_adapter.graph_path_bridge import GraphPathBridge
    root = Path(os.environ['WILDOS_MOTION_TRACE_DIR'])
    root.mkdir(parents=True, exist_ok=True)
    out = (root/'events.jsonl').open('x', buffering=1)
    atexit.register(out.close)

    def persist(kind, began, blobs, details):
        try:
            files = {}
            for role, data in blobs.items():
                digest = hashlib.sha256(data).hexdigest()
                path = root/(digest+'.cdr')
                if not path.exists():
                    path.write_bytes(data)
                files[role] = dict(file=path.name, bytes=len(data))
            out.write(json.dumps(dict(kind=kind, wall_ns=time.time_ns(),
                total_ns=time.monotonic_ns()-began, messages=files, **details))+'\n')
        except Exception as exc:
            out.write(json.dumps(dict(diagnostic_error=repr(exc)))+'\n')

    original_send = GraphPathBridge.send_motion_segment

    def send(self, replacement=None, after_maneuver=False):
        began = time.monotonic_ns()
        blobs = {}
        details = dict(after_maneuver=after_maneuver, replacement=replacement is not None,
            epoch=self.epoch, serial=self.follow_request_serial,
            exploration_id=(self.exploration_metadata(self.route) or {}).get('exploration_id'))
        try:
            queue = replacement if replacement is not None else self.motion_queue
            messages = dict(grid=self.grid, local_grid=self.local_grid, odom=self.odom,
                            route=self.route)
            messages.update({f'queue_{i}': p for i, p in enumerate(queue)})
            blobs = {k: serialize_message(v) for k,v in messages.items() if v is not None}
            details['capture_ns'] = time.monotonic_ns()-began
        except Exception as exc:
            details['capture_error'] = repr(exc)
        try:
            result = original_send(self, replacement, after_maneuver)
            details.update(result=result, status=self.last_status,
                           queue_after=len(self.motion_queue), serial_after=self.follow_request_serial)
            return result
        finally:
            persist('motion_admission', began, blobs, details)

    GraphPathBridge.send_motion_segment = send
    original_checked = GraphPathBridge.execute_checked_path

    def checked(self, path, target):
        began = time.monotonic_ns()
        blobs = {}
        details = dict(epoch=self.epoch, target=list(target))
        try:
            blobs['full_path'] = serialize_message(path)
        except Exception as exc:
            details['capture_error'] = repr(exc)
        try:
            return original_checked(self, path, target)
        finally:
            persist('checked_full_path', began, blobs, details)

    GraphPathBridge.execute_checked_path = checked
    original_reuse = GraphPathBridge.execute_frontier_choice_path

    def reuse(self, waiting, metadata):
        began = time.monotonic_ns()
        result = original_reuse(self, waiting, metadata)
        if waiting is not None:
            persist('committed_choice', began, {}, dict(reused=result,
                uuid=waiting['message'].chosen_uuid, metadata=metadata))
        return result

    GraphPathBridge.execute_frontier_choice_path = reuse
