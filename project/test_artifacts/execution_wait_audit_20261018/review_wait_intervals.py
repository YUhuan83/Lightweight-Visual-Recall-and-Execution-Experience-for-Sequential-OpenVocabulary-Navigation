"""Separate explicitly logged wait-to-send from time after planner dispatch."""
from pathlib import Path
import bisect,json,re,statistics
OUT=Path(__file__).parent
CLOSED=OUT.parent/'controller_source_field_20261018/closed_evidence'
SESSION=next((CLOSED/'field_results').glob('session_*'))
assert (SESSION/'session_result.json').is_file() and (SESSION/'goal_001/result.json').is_file()
events=[json.loads(x) for x in (SESSION/'goal_001/events.jsonl').read_text(encoding='utf8').splitlines() if x]
statuses=[e for e in events if e.get('kind')=='plan_status']
requests=[];results=[];accepted=[]
for line in (SESSION/'stack.log').read_text(encoding='utf8').splitlines():
    if '[wildos_graph_path_bridge]' not in line:continue
    stamp=re.search(r'\[(\d+\.\d+)\]',line)
    if not stamp:continue
    t=float(stamp[1])
    sent=re.search(r'ComputePath request sent: epoch=(\d+) candidate=(\d+) request=(\d+)',line)
    result=re.search(r'ComputePath result: epoch=(\d+) candidate=(\d+) status=(\d+) poses=(\d+)',line)
    accept=re.search(r'ComputePath goal accepted: epoch=(\d+) candidate=(\d+)',line)
    if sent:requests.append(dict(t=t,epoch=int(sent[1]),candidate=int(sent[2]),serial=int(sent[3])))
    if result:results.append(dict(t=t,epoch=int(result[1]),candidate=int(result[2]),status=int(result[3]),poses=int(result[4])))
    if accept:accepted.append(dict(t=t,epoch=int(accept[1]),candidate=int(accept[2])))
waits=[]
for i,event in enumerate(statuses):
    if event['status']!='waiting_for_costmap_update':continue
    start=event['stamp_ns']/1e9
    end=statuses[i+1]['stamp_ns']/1e9 if i+1<len(statuses) else start
    sent=next((r for r in requests if start<=r['t']<end),None)
    waits.append(dict(start=start,end=end,public_label_sec=end-start,
        candidate_index=event.get('candidate_index'),
        first_request=sent,
        wait_to_first_send_upper_sec=min(end-start,sent['t']-start) if sent else None,
        after_first_send_public_label_sec=end-sent['t'] if sent else None,
        next_status=statuses[i+1]['status'] if i+1<len(statuses) else None))
lo=next(e['stamp_ns']/1e9 for e in statuses)
hi=max(e['stamp_ns']/1e9 for e in statuses)
paired=[];used=set();unmatched_results=[]
for response in results:
    if not lo<=response['t']<=hi:continue
    # The legacy result line lacks serial. Never reuse a result for multiple
    # requests; the latest preceding same-epoch/index dispatch is the only
    # possible current callback. Preserve unpaired transactions explicitly.
    eligible=[r for r in requests if lo<=r['t']<=response['t'] and r['serial'] not in used
              and r['epoch']==response['epoch'] and r['candidate']==response['candidate']]
    if not eligible:
        unmatched_results.append(response);continue
    request=max(eligible,key=lambda r:r['t']);used.add(request['serial'])
    paired.append(dict(request=request,result=response,roundtrip_sec=response['t']-request['t']))
def stats(values):
    v=sorted(values)
    return dict(count=len(v),median=statistics.median(v),p95=v[min(len(v)-1,int(.95*len(v)))],max=v[-1]) if v else None
out=dict(scope='Public waiting label split by explicit request-sent logs; before-send is an upper bound, '
    'includes callback scheduling/preparation; no proof of private costmap lock CPU time.',
    waiting_windows=len(waits),public_label_total_sec=sum(x['public_label_sec'] for x in waits),
    matched_waits=sum(x['first_request'] is not None for x in waits),
    matched_before_send_upper_sec=sum(x['wait_to_first_send_upper_sec'] or 0 for x in waits),
    matched_after_send_label_sec=sum(x['after_first_send_public_label_sec'] or 0 for x in waits),
    wait_to_send=stats([x['wait_to_first_send_upper_sec'] for x in waits if x['first_request']]),
    request_roundtrip=stats([x['roundtrip_sec'] for x in paired]),
    planned_requests=len([r for r in requests if lo<=r['t']<=hi]),matched_results=len(paired),
    unmatched_requests=[r for r in requests if lo<=r['t']<=hi and r['serial'] not in used],
    unmatched_results=unmatched_results,waits=waits,paired=paired)
(OUT/'wait_intervals.json').write_text(json.dumps(out,indent=2)+'\n',encoding='utf8')
print(json.dumps({k:v for k,v in out.items() if k not in ('waits','paired')},indent=2))
