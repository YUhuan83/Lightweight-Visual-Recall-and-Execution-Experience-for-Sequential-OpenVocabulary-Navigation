"""Isolated reproduction using the production GoalStandoff callback.

Runs with the existing ROS interface doubles. It compares identical unknown
local maps with (a) map/current odometry aligned and (b) a 0.25 m pose skew.
"""
import json
import sys
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / "nav2_adapter"), str(ROOT / "nav2_adapter/test"),
                str(ROOT / "official_adapter")]

from offline_ros_fakes import install
install()

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String
from beamng_wildos_adapter.goal_standoff import GoalStandoff


def run_case(skew_m):
    now = [100_000_000_000]
    node = GoalStandoff()
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        nanoseconds=now[0], to_msg=lambda: SimpleNamespace(
            sec=now[0] // 1_000_000_000, nanosec=now[0] % 1_000_000_000)))
    node.on_query(String(data="house"))

    old = Odometry()
    old.header.frame_id = "spot1/odom"
    old.header.stamp.sec, old.header.stamp.nanosec = divmod(now[0], 1_000_000_000)
    old.pose.pose.orientation.w = 1.0
    node.on_odom(old)

    grid = OccupancyGrid()
    grid.header.frame_id = "spot1/base_link"
    grid.header.stamp = old.header.stamp
    grid.info.width = grid.info.height = 80
    grid.info.resolution = 0.5
    grid.info.origin.position.x = grid.info.origin.position.y = -20.0
    grid.info.origin.orientation.w = 1.0
    grid.data = [-1] * (80 * 80)
    node.on_occupancy(grid)

    if skew_m:
        now[0] += 150_000_000
        current = Odometry()
        current.header.frame_id = "spot1/odom"
        current.header.stamp.sec, current.header.stamp.nanosec = divmod(
            now[0], 1_000_000_000)
        current.pose.pose.position.x = float(skew_m)
        current.pose.pose.orientation.w = 1.0
        node.on_odom(current)

    target = PoseStamped()
    target.header.frame_id = "spot1/odom"
    target.pose.position.x = 20.0
    target.pose.position.z = 2.0
    reason = node.publish_standoff(target)
    current_xy = [node.odom.pose.pose.position.x, node.odom.pose.pose.position.y]
    map_xy = [node.occupancy_odom.pose.pose.position.x,
              node.occupancy_odom.pose.pose.position.y]
    goal = list(node.last_goal) if node.last_goal is not None else None
    result = {
        "pose_skew_m": float(skew_m),
        "approach_goal_ready": reason is None and goal is not None,
        "failure_reason": reason,
        "current_odom_xy": current_xy,
        "occupancy_paired_odom_xy": map_xy,
        "published_goal_xy": goal,
        "goal_distance_from_current_m": (None if goal is None else
            ((goal[0] - current_xy[0]) ** 2 + (goal[1] - current_xy[1]) ** 2) ** 0.5),
        "object_distance_from_current_m": 20.0 - current_xy[0],
        "publisher_calls": node.publisher.publish.call_count,
    }
    node.destroy_node()
    return result


offsets = (0.0, 0.19, 0.20, 0.21, 0.25, 0.29)
results = [run_case(offset) for offset in offsets]
out = Path(__file__).with_name("goal_standoff_pose_skew_repro.json")
out.write_text(json.dumps({"results": results}, indent=2), encoding="utf-8")
print(json.dumps({"results": results}, indent=2))
for row in results:
    if row["pose_skew_m"] < 0.2:
        assert row["failure_reason"] == "no_reachable_standoff", row
        assert row["publisher_calls"] == 0, row
    else:
        assert row["approach_goal_ready"] is True, row
        assert row["publisher_calls"] == 1, row
        assert row["goal_distance_from_current_m"] == row["pose_skew_m"], row
        assert row["object_distance_from_current_m"] > 10.0, row
