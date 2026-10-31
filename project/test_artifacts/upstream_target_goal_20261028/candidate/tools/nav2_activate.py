"""Ensure Nav2 lifecycle nodes reach ACTIVE, tolerating an already-active node.

Works around nav2_lifecycle_manager giving up on change_state timeouts on the
loaded server. Reads node state first so "already active" is not a failure.
"""
from __future__ import annotations

import time

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node

NODES = ["/planner_server", "/controller_server"]
STATE_NAMES = {
    State.PRIMARY_STATE_UNKNOWN: "unknown",
    State.PRIMARY_STATE_UNCONFIGURED: "unconfigured",
    State.PRIMARY_STATE_INACTIVE: "inactive",
    State.PRIMARY_STATE_ACTIVE: "active",
    State.PRIMARY_STATE_FINALIZED: "finalized",
}


def get_state(node, name, timeout=20.0):
    client = node.create_client(GetState, name + "/get_state")
    if not client.wait_for_service(timeout_sec=timeout):
        print(f"[activate] no get_state for {name}")
        return None
    future = client.call_async(GetState.Request())
    deadline = time.time() + timeout
    while not future.done() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not future.done():
        return None
    return future.result().current_state


def change_state(node, name, transition, timeout=40.0):
    client = node.create_client(ChangeState, name + "/change_state")
    if not client.wait_for_service(timeout_sec=20.0):
        print(f"[activate] no change_state for {name}")
        return False
    request = ChangeState.Request()
    request.transition.id = transition
    future = client.call_async(request)
    deadline = time.time() + timeout
    while not future.done() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not future.done():
        print(f"[activate] timeout {name} {transition}")
        return False
    result = future.result()
    return bool(result is not None and result.success)


def ensure_active(node, name):
    for _ in range(3):
        state = get_state(node, name)
        label = STATE_NAMES.get(state.id, f"id={state.id}") if state else "None"
        print(f"[activate] {name} state={label}")
        if state is not None and state.id == State.PRIMARY_STATE_ACTIVE:
            return True
        if state is not None and state.id == State.PRIMARY_STATE_UNCONFIGURED:
            change_state(node, name, Transition.TRANSITION_CONFIGURE)
            continue
        if change_state(node, name, Transition.TRANSITION_ACTIVATE):
            continue
        time.sleep(1.0)
    state = get_state(node, name)
    return bool(state is not None and state.id == State.PRIMARY_STATE_ACTIVE)


def main() -> int:
    rclpy.init()
    node = Node("wildos_nav2_activate")
    ok = all(ensure_active(node, name) for name in NODES)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    print("ACTIVATE_OK" if ok else "ACTIVATE_FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
