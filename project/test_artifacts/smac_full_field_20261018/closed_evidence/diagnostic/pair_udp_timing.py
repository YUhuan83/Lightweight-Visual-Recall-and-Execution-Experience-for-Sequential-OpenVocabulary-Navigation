"""Pair process UDP/kernel timestamps with actual DDS/subscription boundaries."""
import argparse
import json
from pathlib import Path
import statistics
def stats(a):
    a=sorted(a)
    return dict(n=len(a),median=statistics.median(a),p95=a[int(.95*len(a))],max=a[-1]) if a else None
def pair(folder,preview=False):
    if not preview:assert (folder/'session_result.json').is_file()
    rx=[json.loads(l) for l in (folder/'native_udp.jsonl').read_text().splitlines()]
    rows=[json.loads(l) for l in (folder/'native_timing.jsonl').read_text().splitlines()]
    lookup={}
    for r in rx:lookup.setdefault((r['sequence'],r['issued_wall_ns']),r)
    paired=[]
    for row in rows:
        if 'issued_wall_ns' not in row:continue
        raw=lookup.get((row['sequence'],row['issued_wall_ns']))
        if not raw:continue
        paired.append(dict(sequence=row['sequence'],issued_wall_ns=row['issued_wall_ns'],
            kernel_wall_ns=raw['kernel_wall_ns'],recv_wall_ns=raw['recv_wall_ns'],
            dds_source_ns=row['metadata']['source_timestamp'],dds_received_ns=row['metadata']['received_timestamp'],
            callback_ns=row['callback_wall_ns'],take_begin_ns=row.get('take_begin_wall_ns'),ready_ns=row.get('ready_wall_ns'),
            take_ns=row['take_wall_ns'],tid=raw['tid'],fd=raw['fd']))
    valid=[r for r in paired if r['kernel_wall_ns']>0]
    report=dict(paired=len(paired),udp_packets=len(rx),valid_kernel=len(valid),preview=preview,segments_ms={})
    for name,fn in {
        'dds_publish_to_kernel':lambda r:r['kernel_wall_ns']-r['dds_source_ns'],
        'kernel_to_recv':lambda r:r['recv_wall_ns']-r['kernel_wall_ns'],
        'recv_to_dds_received':lambda r:r['dds_received_ns']-r['recv_wall_ns'],
        'dds_received_to_ready':lambda r:r['ready_ns']-r['dds_received_ns'],
        'recv_to_callback':lambda r:r['callback_ns']-r['recv_wall_ns']}.items():
        report['segments_ms'][name]=stats([fn(r)/1e6 for r in valid if r.get('ready_ns')])
    report['late']=[r for r in paired if r['callback_ns']-r['issued_wall_ns']>200_000_000]
    if not preview:(folder/'udp_timing_review.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v if k!='late' else len(v) for k,v in report.items()},indent=2))
    return report
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder',type=Path);p.add_argument('--preview',action='store_true');a=p.parse_args();pair(a.folder,a.preview)
