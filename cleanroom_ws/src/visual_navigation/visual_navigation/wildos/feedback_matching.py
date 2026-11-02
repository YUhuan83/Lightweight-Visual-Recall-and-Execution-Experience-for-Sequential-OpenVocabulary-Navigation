"""Match traversal feedback to the observation preceding that traversal."""


def select_feedback_sample(
    samples, start_stamp, from_node, max_age, future_tolerance=0.2
):
    eligible = [
        (index, sample)
        for index, sample in enumerate(samples)
        if sample.get("node_uuid") == from_node
        and -future_tolerance <= start_stamp - sample["ros_stamp"] <= max_age
    ]
    if not eligible:
        return None
    return max(eligible, key=lambda item: item[1]["ros_stamp"])[0]
