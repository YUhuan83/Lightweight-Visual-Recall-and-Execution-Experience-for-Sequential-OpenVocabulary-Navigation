"""Offline ground-model counterexamples and recorded adapted-cloud controls; no repair."""
from pathlib import Path
import hashlib,json,sys
import numpy as np
root=Path(__file__).resolve().parents[2];out=Path(__file__).resolve().parent
sys.path[:0]=[str(root/'nav2_adapter'),str(root/'official_adapter')]
import wildos_nav2_adapter.pointcloud as pc
from wildos_nav2_adapter.math3d import matrix_from_quaternion
from wildos_nav2_adapter.vehicle_geometry import SBR_FRONT_EXTENT,SBR_REAR_EXTENT,SBR_HALF_WIDTH,SBR_OBSTACLE_MAX_HEIGHT
from wildos_nav2_adapter.sensor_geometry import SBR_LIDAR_OFFSET
from beamng_wildos_adapter.mapping import occupancy_from_points
from beamng_wildos_adapter.terrain import terrain_layers

def paired_minimum_plane(points,radius=8.):
    """Diagnostic variant: keep each minimum's actual XY, not its cell center."""
    points=np.asarray(points,float)
    near=points[(np.hypot(points[:,0],points[:,1])<=radius)&(points[:,2]>=-2.)&(points[:,2]<=1.2)]
    if len(near)<100:return None
    cells=np.floor(near[:,:2]/.5).astype(np.int32)
    unique,inverse=np.unique(cells,axis=0,return_inverse=True)
    if len(unique)<30:return None
    minima=np.full(len(unique),np.inf);np.minimum.at(minima,inverse,near[:,2])
    is_min=near[:,2]==minima[inverse]
    indices=np.full(len(unique),len(near),dtype=np.int64)
    np.minimum.at(indices,inverse[is_min],np.flatnonzero(is_min))
    coordinates=near[indices,:2]
    matrix=np.c_[coordinates,np.ones(len(coordinates))];keep=np.ones(len(coordinates),bool)
    for _ in range(4):
        if keep.sum()<30:return None
        plane,_,rank,_=np.linalg.lstsq(matrix[keep],minima[keep],rcond=None)
        if rank<3:return None
        error=minima-matrix@plane
        keep=(error>=np.quantile(error,.05)-.05)&(error<=np.quantile(error,.65)+.05)
    if keep.sum()<30 or np.hypot(*plane[:2])>.6 or np.percentile(np.abs(error[keep]),90)>.15:return None
    return plane

def ground_relative_hypothesis(points,plane):
    points=np.asarray(points,np.float32)
    if plane is None:return pc.filter_height(points,.1,SBR_OBSTACLE_MAX_HEIGHT)
    nearby=np.hypot(points[:,0],points[:,1])<=8.
    road=points[:,:2]@plane[:2]+plane[2]
    return points[(points[:,2]<=SBR_OBSTACLE_MAX_HEIGHT)&
                  ((nearby&(points[:,2]-road>=.1))|(~nearby&(points[:,2]>=.1)))]

xy=np.array([(x,y) for x in np.arange(.03,7.5,.1) for y in np.arange(-2.97,3.,.1)])
cases={
    'steep_planar_road':np.c_[xy,.24*xy[:,0]+.24*xy[:,1]-.45],
    'smooth_convex_road':np.c_[xy,.015*xy[:,0]**2-.5],
    'smooth_crest_road':np.c_[xy,-.015*(xy[:,0]-4.)**2-.4],
}
controls={}
for name,points in cases.items():
    current=pc.local_ground_plane(points,8.);paired=paired_minimum_plane(points)
    controls[name]=dict(points=len(points),current_plane=None if current is None else current.tolist(),
        paired_plane=None if paired is None else paired.tolist(),
        raw_ground_relative_current_fit_markings=len(ground_relative_hypothesis(points,current)),
        raw_ground_relative_paired_fit_markings=len(ground_relative_hypothesis(points,paired)),
        truth='Continuous surface with no vertical step; classification counts are prototype outputs.')
expected=np.array([.24,.24,-.45]);actual=pc.local_ground_plane(cases['steep_planar_road'],8.)
paired=paired_minimum_plane(cases['steep_planar_road'])
assert np.max(np.abs(paired-expected))<1e-8
assert abs(actual[2]-expected[2])>.03
controls['steep_planar_road']['known_plane']=expected.tolist()
controls['steep_planar_road']['current_intercept_error_m']=float(actual[2]-expected[2])

# Distinct continuous surfaces produce exactly the same coarse measurements.
coarse=np.array([(x,y) for x in (-.5,0.,.5) for y in (-.5,0.,.5)])
step=np.where(coarse[:,0]>=0.,.17,0.)
ramp=np.clip(.34*(coarse[:,0]+.5),0.,.17)
assert np.array_equal(step,ramp)
measured=np.c_[coarse+[5.75,.25],step-.5]
layers=terrain_layers(np.repeat(measured,2,axis=0))
ambiguity=dict(coarse_cell_spacing_m=.5,step_true_jump_m=.17,ramp_true_jump_m=0.,ramp_max_grade=.34,
    exact_same_observed_heights=True,terrain_measured_cells=int(layers['elevation_valid'].sum()),
    conclusion='A formula over these samples cannot identify the true vertical step without additional observations/assumptions.')

# A roof-mounted free ray traverses XY while passing above a low obstacle.
origin=np.asarray(SBR_LIDAR_OFFSET);endpoint=np.array([10.,0.,-.5]);obstacle_x=4.
fraction=(obstacle_x-origin[0])/(endpoint[0]-origin[0])
ray_height=origin[2]+fraction*(endpoint[2]-origin[2]);obstacle_top=-.5+.17
grid=occupancy_from_points(endpoint[None,:],sensor_origin_xy=origin[:2])
cell=(int(np.floor(obstacle_x/.5+40)),40)
assert grid[cell[1],cell[0]]==0 and ray_height>obstacle_top
ray=dict(sensor_origin=origin.tolist(),endpoint=endpoint.tolist(),test_obstacle_x=obstacle_x,
    unobserved_obstacle_top_z=obstacle_top,ray_z_at_obstacle_x=float(ray_height),
    xy_cell_cost=int(grid[cell[1],cell[0]]),
    actual_collision_object_claimed=False,
    conclusion='XY ray clearing cannot by itself prove a low swept body volume is empty.')

evidence_dir=root/'test_artifacts/execution_wait_audit_20261018'
metadata=json.loads((evidence_dir/'map_evidence.json').read_text(encoding='utf8'))
opts=dict(min_height=.1,max_height=SBR_OBSTACLE_MAX_HEIGHT,filter_footprint=True,
    front_extent=SBR_FRONT_EXTENT,rear_extent=SBR_REAR_EXTENT,half_width=SBR_HALF_WIDTH,
    voxel=.2,max_points=20000,ground_plane_radius=8.,ground_clearance=.25)
rows=[]
def point_set(points):return {tuple(p) for p in np.round(points,4)}
for sample in metadata['samples']:
    path=evidence_dir/(sample['label']+'.npz');arrays=np.load(path,allow_pickle=False)
    tf=next(t['message'] for t in sample['transforms'] if t['message']['child_frame_id']=='spot1/ouster/front/os_lidar')
    assert tf['header']['frame_id']=='spot1/base_link'
    translation=tf['transform']['translation'];rotation=tf['transform']['rotation']
    matrix=np.asarray(matrix_from_quaternion([rotation[k] for k in ('x','y','z','w')]))
    points=arrays['adapted']@matrix.T+np.array([translation[k] for k in ('x','y','z')])
    points=points[np.linalg.norm(points,axis=1)<=40.]
    marked,cleared=pc.split_costmap_points(points,**opts)
    external=pc.filter_robot_footprint(points,SBR_FRONT_EXTENT,SBR_REAR_EXTENT,SBR_HALF_WIDTH)
    current=pc.local_ground_plane(external,8.);paired=paired_minimum_plane(external)
    matched=sample['clouds']['adapted']['stamp_ns']==sample['clouds']['markings']['stamp_ns']
    comparison=dict(same_source_stamp=matched,actual_count=len(arrays['markings']),replayed_count=len(marked),
        equal_rounded_point_sets=point_set(marked)==point_set(arrays['markings']))
    if matched:assert comparison['equal_rounded_point_sets'],sample['label']
    proposals={}
    for label,plane in (('current_plane',current),('paired_plane',paired)):
        points_proposed=ground_relative_hypothesis(external,plane)
        preview=((points_proposed[:,0]>=SBR_FRONT_EXTENT)&(points_proposed[:,0]<=SBR_FRONT_EXTENT+2.)&
                 (np.abs(points_proposed[:,1])<=SBR_HALF_WIDTH))
        proposals[label]=dict(plane=None if plane is None else plane.tolist(),
            markings_before_voxel=len(points_proposed),near_front_markings_before_voxel=int(preview.sum()))
    rows.append(dict(label=sample['label'],sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        adapted_frame=sample['clouds']['adapted']['frame'],baseline_replay=comparison,diagnostic_proposals=proposals))
report=dict(scope=__doc__,production_modified=False,source_pointcloud_sha256=hashlib.sha256(Path(pc.__file__).read_bytes()).hexdigest(),
    controls=controls,coarse_height_ambiguity=ambiguity,clearing_ray_counterexample=ray,recorded_samples=rows,
    local_recorded_source_candidate=metadata['candidate'],true_acquisition_timing_validated=False,
    newest_contact_recording_replayed=False,complete_navigation_passed=False,
    proposed_policy_approved_for_deployment=False)
(out/'ground_model_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf8')
print(json.dumps(dict(controls=controls,ambiguity=ambiguity,ray=ray,
    samples=[dict(label=r['label'],baseline=r['baseline_replay'],proposals=r['diagnostic_proposals']) for r in rows]),indent=2))
