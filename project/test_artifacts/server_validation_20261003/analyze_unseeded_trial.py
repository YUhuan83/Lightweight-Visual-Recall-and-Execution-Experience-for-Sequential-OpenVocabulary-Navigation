"""Read only query, localization and memory evidence from the recorded trial."""
from collections import Counter
from pathlib import Path
import json
import math
import sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import String

root = Path('/home/ly/town_memory_validation_20261003')
out = root / sys.argv[1]
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=str(out/'rosbag'), storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('', ''))
topics = ['/spot1/object_search/query', '/spot1/object_localization',
          '/spot1/object_confirmation', '/spot1/remembered_object_candidates',
          '/wildos/memory_selection']
reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
queries, localizations, confirmations, candidates, selections = [], [], [], [], []
while reader.has_next():
    topic, payload, timestamp = reader.read_next()
    text = deserialize_message(payload, String).data
    if topic == topics[0]:
        queries.append({'query': text, 'received_ns': timestamp})
        continue
    value = json.loads(text)
    value['received_ns'] = timestamp
    {topics[1]: localizations, topics[2]: confirmations,
     topics[3]: candidates, topics[4]: selections}[topic].append(value)

summary = {}
for query in ['house', 'gas station']:
    rows = [v for v in localizations if v.get('query') == query]
    confidences = [v.get('quality', {}).get('confidence', 0) for v in rows]
    max_cluster = 0
    for row in rows:
        current = row['observation_stamp_ns']
        window = [v for v in rows if 0 <= current-v['observation_stamp_ns'] <= 30_000_000_000
                  and v.get('quality', {}).get('confidence', 0) >= .5][-10:]
        values = np.asarray([v['position'] for v in window])
        for seed in values:
            inliers = np.linalg.norm(values[:, :2] - seed[:2], axis=1) <= 3.0
            centre = np.median(values[inliers], axis=0)
            inliers = np.linalg.norm(values[:, :2] - centre[:2], axis=1) <= 1.5
            if np.count_nonzero(inliers):
                centre = np.median(values[inliers], axis=0)
                inliers &= np.linalg.norm(values[:, :2] - centre[:2], axis=1) <= 1.5
                max_cluster = max(max_cluster, int(np.count_nonzero(inliers)))
    summary[query] = {'localization_count': len(rows),
                      'confident_localization_count': sum(v >= .5 for v in confidences),
                      'confidence_range': [min(confidences), max(confidences)] if rows else None,
                      'max_support_within_1_5m_and_30s': max_cluster,
                      'confirmation_count': sum(v.get('query') == query for v in confirmations)}

report = {'queries': queries, 'localization_summary': summary,
          'first_station_candidates': next((v for v in candidates if v.get('query') == 'gas station'), None),
          'station_confirmations': [v for v in confirmations if v.get('query') == 'gas station'],
          'memory_selection_statuses': dict(Counter(v.get('status') for v in selections)),
          'localizations': localizations}
(out/'query_memory_diagnosis.json').write_text(json.dumps(report, indent=2))
print(json.dumps({k:v for k,v in report.items() if k != 'localizations'}, indent=2))
