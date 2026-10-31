"""Separate recorded packet latency, receipt lifetime and SDK call timing; no guard changes."""
exec(compile(open('motion_audit/audit.py').read().split('intervals=[]; run=None')[0],'motion_audit/audit.py','exec'))
def quantile(values):
    a=sorted(x for x in values if isinstance(x,(int,float)) and math.isfinite(x))
    return dict(n=len(a),min=a[0],p50=a[int((len(a)-1)*.5)],p95=a[int((len(a)-1)*.95)],p99=a[int((len(a)-1)*.99)],max=a[-1]) if a else None
commands=[]
for t,b in c.execute('select timestamp,data from messages where topic_id=? order by timestamp',(topics['/wildos/native_control_command'],)):
    v=json.loads(deserialize_message(b,String).data);commands.append((t/1e9-start,v))
firsts={}
for t,v in native:
    rec=v.get('source_receipt')
    if rec and rec.get('sequence') not in firsts:firsts[rec['sequence']]=(t,v)
command_timing=[]
for t,v in commands:
    n=firsts.get(v['sequence']);issued=v['issued_wall_ns']/1e9;expires=v['expires_wall_ns']/1e9
    command_timing.append(dict(video_sec=t,sequence=v['sequence'],generation=v['generation'],control=v['control'],
        issued_to_recorder_ms=(start+t-issued)*1000,lifetime_ms=(expires-issued)*1000,
        first_native_seen_ms=((start+n[0]-issued)*1000 if n else None),
        native_callback_delay_ms=((start+n[0]-n[1]['command_receipt_age']-issued)*1000 if n and n[1].get('command_receipt_age') is not None else None)))
for i,x in enumerate(command_timing):
    if i:x['publisher_issue_gap_ms']=(commands[i][1]['issued_wall_ns']-commands[i-1][1]['issued_wall_ns'])/1e6
r=dict(scope=__doc__,native_sample_count=len(native),
 rpc_ms=quantile([v.get('control_rpc_ms') for t,v in native]),poll_ms=quantile([v.get('poll_roundtrip_ms') for t,v in native]),
 native_period_ms=quantile([v.get('period',0)*1000 for t,v in native]),
 native_actuator_bad=sum(v.get('actuator_ok') is not True for t,v in native),
 native_lease_tripped=sum(bool((v.get('actual') or {}).get('wildosControlLeaseTripped')) for t,v in native),
 command_count=len(commands),timing={k:quantile([v.get(k) for v in command_timing]) for k in ('issued_to_recorder_ms','lifetime_ms','first_native_seen_ms','native_callback_delay_ms','publisher_issue_gap_ms')},
 commands=command_timing,clock_alignment_sample=[dict(record_wall=t+start,payload_wall=v.get('wall_time'),difference_ms=1000*((t+start)-v.get('wall_time'))) for t,v in native[:3]])
(root/'motion_audit/source_timing.json').write_text(json.dumps(r,indent=2)+'\n')
print(json.dumps({k:v for k,v in r.items() if k!='commands'},indent=2))
print('AROUND_FIRST_FAULT')
for x in command_timing:
 if 92.9<x['video_sec']<94.4:print(x)
