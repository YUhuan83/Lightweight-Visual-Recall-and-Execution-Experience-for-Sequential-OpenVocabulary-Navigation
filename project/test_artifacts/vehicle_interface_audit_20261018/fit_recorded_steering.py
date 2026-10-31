"""Estimate steering-to-curvature from measured rear-axle motion, not parameters."""
import json,math,statistics,sys
from pathlib import Path
import numpy as np
from types import SimpleNamespace as S

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'nav2_adapter'))
sys.path.insert(0,str(ROOT/'official_adapter'))
from beamng_wildos_adapter.geometry import body_rotation
from wildos_nav2_adapter.vehicle_geometry import SBR_BASE_OFFSET,SBR_WHEELBASE

def vec(a):return S(x=a[0],y=a[1],z=a[2])
def inspect(label,session):
    assert (session/'session_result.json').is_file()
    rows=[]
    for line in (session/'execution_observer/messages.jsonl').open():
        row=json.loads(line)
        if row['topic']!='/wildos/native_control_status':continue
        v=json.loads(row['message']['data'])
        if not all(isinstance(v.get(k),list) for k in ('position','direction','up','velocity')):continue
        rotation=body_rotation(vec(v['direction']),vec(v['up']))
        rear=np.asarray(v['position'])+rotation@np.asarray(SBR_BASE_OFFSET)
        heading=math.atan2(v['direction'][1],v['direction'][0])
        signed_speed=float(np.dot(v['velocity'],rotation[:,0]))
        actual=v.get('actual') or {};sent=v['sent'];steer=actual.get('steering_input')
        good=(type(steer) in (int,float) and v.get('telemetry_ok') is True and v.get('actuator_ok') is True
              and (v.get('source_receipt') or {}).get('applied') is True
              and v['telemetry_age_sec']<.2 and sent['brake']==0 and actual.get('brake',1)<.01
              and abs(signed_speed)>.4 and abs(steer)>.1
              and abs(steer-sent['steering'])<.02)
        rows.append(dict(t=v['wall_time'],xy=rear[:2],heading=heading,v=signed_speed,steer=steer,good=good))
    windows=[];next_allowed=-math.inf
    for i,r in enumerate(rows):
        if r['t']<next_allowed or not r['good']:continue
        j=i
        while j+1<len(rows) and rows[j+1]['t']-r['t']<=2.5:j+=1
        block=rows[i:j+1]
        if len(block)<25 or block[-1]['t']-r['t']<1.8 or not all(v['good'] and v['v']*r['v']>0 for v in block):continue
        if max(b['t']-a['t'] for a,b in zip(block,block[1:]))>.15:continue
        steer=np.array([v['steer'] for v in block])
        if np.ptp(steer)>.025:continue
        sign=1 if r['v']>0 else -1
        ds=np.array([0.]+[sign*float(np.linalg.norm(b['xy']-a['xy'])) for a,b in zip(block,block[1:])]).cumsum()
        if abs(ds[-1])<1.2:continue
        headings=np.unwrap([v['heading'] for v in block])
        curvature,intercept=np.polyfit(ds,headings,1)
        error=np.max(np.abs(headings-(curvature*ds+intercept)))
        if abs(curvature)<.025 or error>.015:continue
        normalized=float(np.median(steer))
        effective_angle=math.atan(SBR_WHEELBASE*curvature)/(-normalized)
        windows.append(dict(start_wall=r['t'],duration=block[-1]['t']-r['t'],signed_distance_m=float(ds[-1]),
            normalized_steering=normalized,curvature_per_m=float(curvature),radius_m=1/abs(curvature),
            fitted_max_steer_rad=effective_angle,configured_max_steer_rad=.54,
            expected_curvature=-math.tan(normalized*.54)/SBR_WHEELBASE,
            max_heading_fit_error_rad=float(error)))
        next_allowed=block[-1]['t']
    return dict(label=label,session=str(session),samples=len(rows),windows=windows,
        median_fitted_max_steer_rad=statistics.median(w['fitted_max_steer_rad'] for w in windows) if windows else None,
        limits='Recorded steady-motion fit; no direct wheel angle/force measurement. Input lag, slip and compliance remain possible.')

sessions=[('ca8',ROOT/'test_artifacts/controller_source_field_20261018/closed_evidence/field_results/session_20261018_090814'),
          ('d9d',ROOT/'test_artifacts/planning_phase_physical_20261018/closed_evidence/field_results/session_20261018_151920')]
result=[inspect(*pair) for pair in sessions]
out=Path(__file__).with_name('recorded_steering_fit.json');out.write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
