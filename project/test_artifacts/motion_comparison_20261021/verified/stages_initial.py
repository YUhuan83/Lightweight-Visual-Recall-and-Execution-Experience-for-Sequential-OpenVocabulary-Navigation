"""Compare existing Native kernel, DDS and executor trace; no new instrumentation or simulation."""
from compare import OUT,q
import json,collections,pathlib
reports=[]
for x in ('215010','224538','225946'):
 r=json.load(open(OUT/('trial_20261020_'+x+'.json')));s=pathlib.Path(r['session']);begin=r['query_wall_start']*1e9;end=begin+420e9
 udp=collections.defaultdict(list)
 for line in (s/'native_udp.jsonl').open():
  v=json.loads(line);issued=v.get('issued_wall_ns',0)
  if begin<=issued<=end:udp[(issued,v['sequence'])].append(v)
 stages=collections.defaultdict(list);matches=[];timers=collections.defaultdict(list)
 for line in (s/'native_timing.jsonl').open():
  v=json.loads(line)
  if v.get('kind')=='timer':
   timers[v['callback']].append((v['end_mono_ns']-v['begin_mono_ns'])/1e6);continue
  issued=v.get('issued_wall_ns',0)
  if not begin<=issued<=end:continue
  row=dict(t=(issued-begin)/1e9,sequence=v['sequence'])
  met=v.get('metadata') or {};wire=udp.get((issued,v['sequence']),[])
  vals={'issue_to_callback_ms':(v['callback_wall_ns']-issued)/1e6,
        'callback_duration_ms':(v['end_mono_ns']-v['callback_mono_ns'])/1e6,
        'dds_received_to_take_ms':(v['take_wall_ns']-met['received_timestamp'])/1e6,
        'rmw_take_ms':(v['take_mono_ns']-v['take_begin_mono_ns'])/1e6,
        'executor_ready_to_handler_ms':(v['handle_entered_mono_ns']-v['ready_mono_ns'])/1e6,
        'handler_to_take_begin_ms':(v['take_begin_mono_ns']-v['handle_entered_mono_ns'])/1e6,
        'dds_received_to_ready_ms':(v['ready_wall_ns']-met['received_timestamp'])/1e6}
  if wire:
   first=min(wire,key=lambda a:a['kernel_wall_ns']);vals.update(
    issue_to_first_kernel_ms=(first['kernel_wall_ns']-issued)/1e6,
    kernel_to_recv_ms=(first['recv_wall_ns']-first['kernel_wall_ns'])/1e6,
    first_recv_to_dds_metadata_ms=(met['received_timestamp']-first['recv_wall_ns'])/1e6)
   row['datagram_count']=len(wire)
  for k,value in vals.items():stages[k].append(value)
  row.update(vals);matches.append(row)
 report=dict(scope=__doc__,trial=x,source_bag_sha256=r['source_bag']['sha256'],
   udp_datagrams=sum(len(v) for v in udp.values()),udp_unique_sequence_count=len(udp),udp_datagrams_per_sequence=q([len(v) for v in udp.values()]),
   traced_callbacks=len(matches),layer_ms={k:q(v) for k,v in stages.items()},timer_duration_ms={k:q(v) for k,v in timers.items()},
   slowest_callbacks=sorted(matches,key=lambda a:a['issue_to_callback_ms'],reverse=True)[:15],
   first_90sec_ms={k:q([v[k] for v in matches if v['t']<90 and k in v]) for k in stages},
   minute_layers_ms=[dict(range_sec=[t,t+60],layers={k:q([v[k] for v in matches if t<=v['t']<t+60 and k in v]) for k in stages}) for t in range(0,420,60)],
   limits='Datagram rows may include retransmits. Metadata received time is middleware metadata, not proof of where a reliability repair occurred. Timer quantiles cover whole owned trace including setup/cleanup.')
 (OUT/('native_layers_'+x+'.json')).write_text(json.dumps(report,indent=2)+'\n');reports.append(report)
 print(json.dumps({k:v for k,v in report.items() if k not in ('minute_layers_ms','slowest_callbacks','first_90sec_ms')},indent=2))
