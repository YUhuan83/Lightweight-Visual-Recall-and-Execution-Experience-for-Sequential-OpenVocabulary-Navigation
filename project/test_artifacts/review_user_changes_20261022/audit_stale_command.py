"""Read-only diagnosis of accepted stamped command freshness in a closed trial bag."""
import collections
import json
import sqlite3
import sys
from pathlib import Path

from geometry_msgs.msg import TwistStamped
from rclpy.serialization import deserialize_message
from std_msgs.msg import String


goal = Path(sys.argv[1])
assert (goal / "stop.json").exists(), "trial goal must be closed"
database = next((goal / "interfaces").glob("*.db3"))
db = sqlite3.connect(f"file:{database}?mode=ro", uri=True)


def topic_id(name):
    row = db.execute("SELECT id FROM topics WHERE name=?", (name,)).fetchone()
    return row[0] if row else None


def rows(name, msg_type):
    tid = topic_id(name)
    if tid is None:
        return []
    output = []
    for bag_ns, data in db.execute(
            "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (tid,)):
        message = deserialize_message(data, msg_type)
        output.append((bag_ns * 1e-9, message))
    return output


controls = []
for bag_time, msg in rows("/wildos/control_status", String):
    try:
        data = json.loads(msg.data)
    except (TypeError, ValueError):
        continue
    data["_time"] = data.get("stamp_ns", int(bag_time * 1e9)) * 1e-9
    controls.append(data)

stale = [c for c in controls if c.get("reason") == "stale_command"]
rejections = collections.Counter(str(c.get("controller_rejection")) for c in stale)
ages = [c.get("controller_source_age_ros_sec") for c in stale
        if isinstance(c.get("controller_source_age_ros_sec"), (int, float))]
gaps = []
for a, b in zip(controls, controls[1:]):
    if a.get("reason") == "stale_command":
        gaps.append((max(0., b["_time"] - a["_time"]), a))

commands = rows("/wildos/cmd_vel_stamped", TwistStamped)
cmd_rows = []
for bag_time, message in commands:
    stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
    cmd_rows.append((bag_time, stamp, message.header.frame_id,
                     float(message.twist.linear.x), float(message.twist.angular.z)))
cmd_gaps = []
for a, b in zip(cmd_rows, cmd_rows[1:]):
    cmd_gaps.append((b[1] - a[1], a, b))

long_stale = sorted(stale, key=lambda c: c.get("controller_source_age_ros_sec") or 0,
                    reverse=True)[:12]
long_command_gaps = sorted((g for g in cmd_gaps if g[0] > .5), reverse=True)[:15]
print(json.dumps({
    "control_samples": len(controls),
    "stale_command_samples": len(stale),
    "stale_rejection_counts": dict(rejections),
    "stale_source_age_sec": {
        "min": min(ages) if ages else None,
        "median": sorted(ages)[len(ages)//2] if ages else None,
        "max": max(ages) if ages else None,
    },
    "stale_samples": [{k: c.get(k) for k in (
        "_time", "controller_rejection", "controller_source_age_ros_sec",
        "command_receipt_age_steady_sec", "controller_deadline_remaining_sec",
        "reason", "target_speed", "enabled", "native_health_ok")}
        for c in long_stale],
    "stamped_command_count": len(cmd_rows),
    "command_frame_counts": dict(collections.Counter(c[2] for c in cmd_rows)),
    "nonmonotonic_source_stamps": sum(b[1] <= a[1] for a, b in zip(cmd_rows, cmd_rows[1:])),
    "command_gaps_over_half_second": [
        {"dt": round(dt, 3), "start": round(a[1], 3), "end": round(b[1], 3),
         "frame": a[2], "v_before": a[3], "v_after": b[3]}
        for dt, a, b in long_command_gaps],
}, indent=2))
