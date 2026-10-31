"""Classify exact Native DDS time boundaries; never reads SQLite."""
import argparse
import json
from pathlib import Path
import statistics

def stats(a):
    a=sorted(a)
    return dict(n=len(a),median=statistics.median(a),p95=a[min(len(a)-1,int(.95*len(a)))],max=a[-1]) if a else None

def summarize(folder,preview=False):
    if not preview:assert (folder/'result.json').is_file()
    rows=[]
    for line in (folder/'native_timing.jsonl').read_text().splitlines():
        try:r=json.loads(line)
        except json.JSONDecodeError:continue
        if r.get('issued_wall_ns') is not None:rows.append(r)
    summary=dict(records=len(rows),preview=preview,segments_ms={})
    segments={
        'issue_to_dds_publish':lambda r:r['metadata']['source_timestamp']-r['issued_wall_ns'],
        'dds_publish_to_receive':lambda r:r['metadata']['received_timestamp']-r['metadata']['source_timestamp'],
        'dds_receive_to_take':lambda r:r['take_wall_ns']-r['metadata']['received_timestamp'],
        'take_to_callback':lambda r:r['callback_mono_ns']-r['take_mono_ns'],
        'callback_duration':lambda r:r['end_mono_ns']-r['callback_mono_ns'],
        'issued_to_callback':lambda r:r['callback_wall_ns']-r['issued_wall_ns']}
    if rows and rows[0].get('take_begin_mono_ns'):
        segments.update(ready_to_take_begin=lambda r:r['take_begin_mono_ns']-r['ready_mono_ns'],
                        take_call=lambda r:r['take_mono_ns']-r['take_begin_mono_ns'],
                        enter_handle=lambda r:r['handle_entered_mono_ns']-r['take_begin_mono_ns'])
    for name,fn in segments.items():summary['segments_ms'][name]=stats([fn(r)/1e6 for r in rows])
    summary['over_200ms']=[r for r in rows if (r['callback_wall_ns']-r['issued_wall_ns'])>200_000_000]
    if not preview:(folder/'timing_review.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps({k:v if k!='over_200ms' else len(v) for k,v in summary.items()},indent=2))
    return summary

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder',type=Path);p.add_argument('--preview',action='store_true');a=p.parse_args()
    summarize(a.folder,a.preview)
