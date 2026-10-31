import json
import sqlite3
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


bag = Path(
    "/home/ly/town_memory_validation_20261003/"
    "green_house_station_arrival_20261005_trial8/rosbag/rosbag_0.db3"
)
db = sqlite3.connect(f"file:{bag}?mode=ro", uri=True)
topic_types = {
    name: get_message(type_name)
    for name, type_name in db.execute("select name, type from topics")
}
topic_ids = {
    name: topic_id
    for name, topic_id in db.execute("select name, id from topics")
}

for topic in (
    "/spot1/object_mask",
    "/spot1/object_localization",
    "/spot1/object_confirmation",
    "/spot1/remembered_object_candidates",
    "/spot1/imgnav_waypoint",
):
    print(f"TOPIC {topic}")
    rows = db.execute(
        "select timestamp, data from messages where topic_id=? order by timestamp",
        (topic_ids[topic],),
    )
    for stamp_ns, raw in rows:
        message = deserialize_message(raw, topic_types[topic])
        stamp_s = round(stamp_ns / 1e9, 3)
        if topic == "/spot1/object_mask":
            print(
                stamp_s,
                message.query,
                "mask_pixels=",
                int(sum(message.object_mask.data)),
                "mask_values=",
                len(message.object_mask.data),
            )
        elif topic == "/spot1/imgnav_waypoint":
            print(
                stamp_s,
                [
                    round(message.pose.position.x, 3),
                    round(message.pose.position.y, 3),
                ],
            )
        else:
            print(stamp_s, message.data[:1200])
