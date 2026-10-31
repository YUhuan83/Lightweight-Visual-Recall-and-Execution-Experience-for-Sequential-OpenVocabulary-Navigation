"""Plot measured motion and the replayed collision footprint from saved data."""
from pathlib import Path
import json
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap,BoundaryNorm
from matplotlib.patches import Circle,Polygon

folder=Path(__file__).resolve().parent/'failure_audit/house_geometry_rviz_attempt2'
audit=json.loads((folder/'audit.json').read_text())
case=json.loads((folder.parents[1]/'house_geometry_rviz_attempt2.json').read_text())
replay=json.loads((folder/'geometry_replay.json').read_text())
fig,axes=plt.subplots(1,2,figsize=(13,6),layout='constrained')
ax=axes[0];native=np.array([n['position'][:2] for n in audit['native']]);born=np.array(case['start_position'])
ax.plot(native[:,0],native[:,1],color='#ef820d',lw=2,label='Measured motion: 25.52 m travelled')
ax.scatter(*born,color='black',marker='*',s=130,label='Town gas station spawn',zorder=6)
ax.add_patch(Circle(born,20,fill=False,color='#497aab',ls='--',lw=1.5,label='Required 20 m departure'))
goals=np.array([e['graph_goal'] for e in case['events'] if e.get('status')=='planned' and e['phase']=='house_search'])
ax.scatter(goals[:,0],goals[:,1],marker='x',color='#487db3',s=45,label='GraphNav route endpoints')
ax.set_title('Travel in a small area: max departure 5.37 m')
ax.set_xlabel('World X (m)');ax.set_ylabel('World Y (m)');ax.set_aspect('equal');ax.grid(alpha=.2)
ax.legend(fontsize=8,loc='upper left')

ax=axes[1];grid=np.load(folder/'48.25_local_raw.npz');a=grid['data'].reshape(*grid['shape'])
categories=np.zeros_like(a);categories[(a>0)&(a<253)]=1;categories[a==253]=2;categories[a==254]=3;categories[a==255]=4
origin=grid['origin'];res=float(grid['resolution'])
cmap=ListedColormap(['#fafafa','#d7e2ef','#9cb4cf','#b73732','#cccccc'])
ax.imshow(categories,origin='lower',extent=[origin[0],origin[0]+a.shape[1]*res,origin[1],origin[1]+a.shape[0]*res],
          cmap=cmap,norm=BoundaryNorm(np.arange(-.5,5),cmap.N),interpolation='nearest')
fail=next(f for f in audit['failures'] if f['status'].get('reason')=='follow_aborted')
path=fail['last_published_section']['points'];ax.plot(*np.array(path).T,color='#bb9200',lw=1.5,label='Planned centreline')
arc=fail['last_arc']['points'];ax.plot(*np.array([fail['rear'],*arc]).T,color='#8552a1',lw=2,label='RPP collision projection')
projected=replay['collision_replay'][1]['arc_end']['local_raw']
for pos,h,color,label in [(fail['rear'],fail['heading'],'#29a5a5','Current body'),
                          (projected['position'],projected['heading_inferred'],'#8552a1','Predicted padded body')]:
    body=np.array([[3.41,1.06],[3.41,-1.06],[-1.06,-1.06],[-1.06,1.06]])
    c,s=math.cos(h),math.sin(h);world=body@np.array([[c,s],[-s,c]])+pos
    ax.add_patch(Polygon(world,fill=False,edgecolor=color,lw=2,label=label))
block=projected['blockers'][0]['world'];ax.scatter(*block,marker='x',s=100,color='#e11919',label='Lethal cell at (728.30, -12.10)',zorder=9)
ax.set_xlim(726.5,736);ax.set_ylim(-15,-7.5);ax.set_aspect('equal');ax.set_xlabel('World X (m)');ax.set_ylabel('World Y (m)')
ax.set_title('48.25 s: front body edge intersects a lethal cell')
ax.legend(loc='upper right',fontsize=8);fig.savefig(folder.parent/'failure_analysis.png',dpi=160)
print(folder.parent/'failure_analysis.png')
