"""Public occupancy and bounded point evidence in the measured rear-axle frame."""
from pathlib import Path
import json,math,sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
OUT=Path(__file__).parent;ROOT=OUT.parents[1]
sys.path.insert(0,str(ROOT/'nav2_adapter'))
from wildos_nav2_adapter.pointcloud import split_costmap_points,filter_robot_footprint,filter_height,local_ground_plane
evidence=json.loads((OUT/'map_evidence.json').read_text(encoding='utf8'))
samples={s['label']:s for s in evidence['samples']}
stalls=json.loads((OUT/'stalls.json').read_text(encoding='utf8'))
reports=[]
fig,axes=plt.subplots(2,2,figsize=(12,12),layout='constrained')
for i in range(2):
    s=samples[f'case{i}_0s_before'];data=np.load(OUT/(s['label']+'.npz'))
    pose=s['odom']['message']['pose']['pose'];p=pose['position'];q=pose['orientation']
    yaw=math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))
    rotation=np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
    adapted=data['adapted']+np.array([2.172,-.33,0.])
    # Adapted/markings are nearest individual samples, not exact same source
    # stamp. Re-filtered adapted points isolate pure geometry policy only.
    kept=filter_robot_footprint(adapted,3.4,1.05,1.05)
    candidates=filter_height(kept,.1,1.5)
    plane=local_ground_plane(kept,8.)
    filtered,clearing=split_costmap_points(adapted,min_height=.1,max_height=1.5,
        filter_footprint=True,front_extent=3.4,rear_extent=1.05,half_width=1.05,
        voxel=.2,max_points=20000,ground_plane_radius=8.,ground_clearance=.25)
    def region(a):return a[(a[:,0]>2.5)&(a[:,0]<6)&(np.abs(a[:,1])<1.3)]
    report=dict(label=s['label'],source_stamps=s['clouds'],
        fitted_ground_plane=plane.tolist() if plane is not None else None,
        front_regions={k:region(v).tolist() for k,v in dict(adapted=adapted,candidates=candidates,
            refiltered=filtered,actual_markings=data['markings']).items()},body_costs={})
    for j,key in enumerate(('local','global')):
        ax=axes[i,j];grid=data[key];info=s['maps'][key]['info'];resolution=info['resolution']
        origin=info['origin']['position'];ys,xs=np.indices(grid.shape)
        world=np.stack([origin['x']+(xs.ravel()+.5)*resolution,
                        origin['y']+(ys.ravel()+.5)*resolution],axis=1)
        body=(world-np.array([p['x'],p['y']]))@rotation
        select=(body[:,0]>-2)&(body[:,0]<8)&(abs(body[:,1])<4)
        values=grid.ravel()
        ax.scatter(body[select,0],body[select,1],c=values[select],cmap='gray_r',vmin=0,vmax=100,
                   s=22,marker='s',alpha=.65)
        raw=adapted[(adapted[:,2]>.1)&(adapted[:,2]<1.5)]
        ax.scatter(raw[:,0],raw[:,1],s=2,c='tab:blue',label='adapted points z .1..1.5')
        mark=data['markings'];ax.scatter(mark[:,0],mark[:,1],s=10,c='red',label='actual markings')
        footprint=np.array([[-1.05,-1.05],[3.4,-1.05],[3.4,1.05],[-1.05,1.05],[-1.05,-1.05]])
        ax.plot(footprint[:,0],footprint[:,1],c='orange',lw=2,label='SBR footprint')
        path=stalls[i].get('last_controller_path')
        if path:
            xy=np.array([[a['pose']['position']['x'],a['pose']['position']['y']] for a in path[1]['poses']])
            if path[1]['header']['frame_id']=='spot1/odom':
                xy=(xy-np.array([p['x'],p['y']]))@rotation
            else:
                assert path[1]['header']['frame_id']=='spot1/nav_base_link'
                # At a nearly stationary stall the newest body-frame plan
                # aligns approximately; source/odom age remains explicit.
            ax.plot(xy[:,0],xy[:,1],color='limegreen',lw=2,label='last observed controller path')
        inside=(body[:,0]>=-1.05)&(body[:,0]<=3.4)&(abs(body[:,1])<=1.05)
        front=(body[:,0]>3.4)&(body[:,0]<5.)&(abs(body[:,1])<=1.05)
        report['body_costs'][key]=dict(max_inside=int(values[inside].max()),
            lethal_inside=int((values[inside]>=99).sum()),
            max_front=int(values[front].max()),lethal_front=int((values[front]>=99).sum()))
        ax.set(xlim=(-2,8),ylim=(-4,4),xlabel='rear axle forward x (m)',ylabel='left y (m)',
               title=f'case {i}: {key} public occupancy at stall')
        ax.set_aspect('equal');ax.grid(alpha=.15)
    reports.append(report)
axes[0,0].legend(fontsize=8)
fig.savefig(OUT/'stall_maps.png',dpi=160)
(OUT/'stall_map_review.json').write_text(json.dumps(reports,indent=2)+'\n',encoding='utf8')
print(json.dumps([{k:v for k,v in x.items() if k!='front_regions'} for x in reports],indent=2))
