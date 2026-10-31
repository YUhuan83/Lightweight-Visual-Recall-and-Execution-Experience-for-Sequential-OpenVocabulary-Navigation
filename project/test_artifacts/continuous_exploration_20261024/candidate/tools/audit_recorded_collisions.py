"""Read-only provenance audit of recorded Nav2 collision failures (ROS host)."""
import argparse
import json
import math
from pathlib import Path
import sqlite3
from types import SimpleNamespace

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from wildos_nav2_adapter.graph_route import footprint_blockers, cell_cost
from wildos_nav2_adapter.pointcloud import decode_xyz, local_ground_plane, split_costmap_points
from wildos_nav2_adapter.math3d import matrix_from_quaternion
from wildos_nav2_adapter.vehicle_geometry import SBR_BASE_OFFSET


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def yaw(p):
    q = p.orientation
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag', type=Path)
    ap.add_argument('output', type=Path)
    args = ap.parse_args()
    db = sqlite3.connect('file:'+str(args.bag)+'?mode=ro', uri=True)
    topics = {name: (i, get_message(t)) for i, name, t in db.execute('select id,name,type from topics')}

    def window(name, t, before=2., after=.5):
        i, cls = topics[name]
        rows = db.execute('select timestamp,data from messages where timestamp between ? and ? '
                          'and topic_id=? order by timestamp',
                          (int((t-before)*1e9), int((t+after)*1e9), i))
        return [(ts*1e-9, deserialize_message(raw, cls)) for ts, raw in rows]

    def last(name, t):
        rows = window(name, t, before=3., after=0.)
        return rows[-1] if rows else (None, None)

    out = []
    for t in (1791011231.137014757, 1791011251.578310147,
              1791011272.334510322, 1791011291.430038396):
        ot, odom = last('/spot1/odom', t)
        pose = odom.pose.pose
        xy = (pose.position.x, pose.position.y)
        angle = yaw(pose)
        report = dict(abort_time=t, odom_bag_time=ot, position=list(xy), heading=angle,
                      speed=odom.twist.twist.linear.x)
        grids = {}
        for key in ('local', 'global'):
            mt, grid = last('/'+key+'_costmap/costmap', t)
            grids[key] = grid
            report[key] = dict(bag_time=mt, stamp=stamp(grid),
                resolution=grid.info.resolution, origin=[grid.info.origin.position.x,grid.info.origin.position.y],
                centre_cost=cell_cost(grid,*xy), footprint_blockers=footprint_blockers(grid,xy,angle,limit=20))
        ct, cloud = last('/wildos/points', t)
        points = decode_xyz(cloud)
        # Match the recorded cloud pose by measurement stamp, not receipt time.
        poses = window('/spot1/odom', ct, before=2., after=.5)
        _, co = min(poses, key=lambda row: abs(stamp(row[1])-stamp(cloud)))
        cp = co.pose.pose
        q = cp.orientation
        rotation = np.asarray(matrix_from_quaternion([q.x,q.y,q.z,q.w]))
        world = points@rotation.T + [cp.position.x,cp.position.y,cp.position.z]
        dx,dy = world[:,0]-xy[0],world[:,1]-xy[1]
        body = np.c_[math.cos(angle)*dx+math.sin(angle)*dy,
                      -math.sin(angle)*dx+math.cos(angle)*dy, world[:,2]]
        report['cloud'] = dict(bag_time=ct,stamp=stamp(cloud),pose_stamp_error=stamp(co)-stamp(cloud),
            frame=cloud.header.frame_id,count=len(points))
        nearest = []
        for a,b in report['local']['footprint_blockers']:
            distances=np.hypot(body[:,0]-a,body[:,1]-b)
            inds=np.argsort(distances)[:6]
            nearest.append(dict(cell_body=[a,b], nearest_points_body_world_z=body[inds].tolist(),
                                distances=distances[inds].tolist()))
        report['cell_point_support']=nearest
        at, arc = last('/lookahead_collision_arc',t)
        report['arc']=dict(bag_time=at,stamp=stamp(arc) if arc else None,
            poses=[[p.pose.position.x,p.pose.position.y,yaw(p.pose)] for p in arc.poses] if arc else [])
        pt, plan = last('/received_global_plan', t)
        report['controller_plan']=dict(bag_time=pt,frame=plan.header.frame_id if plan else None,
            points=[[p.pose.position.x,p.pose.position.y,yaw(p.pose)] for p in plan.poses] if plan else [])
        lt, carrot = last('/lookahead_point',t)
        curvature=2*carrot.point.y/(carrot.point.x**2+carrot.point.y**2)
        direction=-1 if carrot.point.x<0 else 1
        report['carrot']=dict(time=lt,x=carrot.point.x,y=carrot.point.y,curvature=curvature,direction=direction)
        if arc:
            # RPP's visualization leaves orientations unset. Recover yaw from
            # the signed integration step; treating zero quaternion as yaw=0
            # would check the wrong body orientation.
            _, ao=min(window('/spot1/odom',at,before=2.,after=.5),
                      key=lambda row: abs(stamp(row[1])-stamp(arc)))
            previous=(ao.pose.pose.position.x,ao.pose.pose.position.y)
            entries=[]
            for p in arc.poses:
                pos=(p.pose.position.x,p.pose.position.y)
                move_heading=math.atan2(pos[1]-previous[1],pos[0]-previous[0])
                heading=move_heading+(math.pi if direction<0 else 0.)+direction*grids['local'].info.resolution*curvature
                entry=dict(pose=[*pos,heading],
                    local=footprint_blockers(grids['local'],pos,heading,limit=20),
                    global_grid=footprint_blockers(grids['global'],pos,heading,limit=20))
                support=[]
                for a,b in entry['local']:
                    wx=pos[0]+math.cos(heading)*a-math.sin(heading)*b
                    wy=pos[1]+math.sin(heading)*a+math.cos(heading)*b
                    distances=np.hypot(world[:,0]-wx,world[:,1]-wy)
                    inds=np.argsort(distances)[:4]
                    support.append(dict(cell_world=[wx,wy],nearest_points_world=world[inds].tolist(),
                        nearest_points_base=points[inds].tolist(),distances=distances[inds].tolist()))
                entry['point_support']=support
                entries.append(entry)
                previous=pos
            report['arc_blockers']=entries
        rawt, raw = min(window('/spot1/ouster/front/points_filtered',ct,before=2.,after=.5),
                        key=lambda row: abs(stamp(row[1])-stamp(cloud)))
        rawpoints=decode_xyz(raw)-np.asarray(SBR_BASE_OFFSET)
        rawpoints=rawpoints[np.linalg.norm(rawpoints,axis=1)<=40.]
        plane=local_ground_plane(rawpoints,8.)
        report['ground_plane']=plane.tolist() if plane is not None else None
        report['raw_cloud']=dict(bag_time=rawt,stamp=stamp(raw),count=len(rawpoints))
        opts=dict(min_height=.1,filter_footprint=True,front_extent=3.4,rear_extent=1.05,
                  half_width=1.05,voxel=.2,max_points=20000,ground_plane_radius=8.,ground_clearance=.25)
        old,_=split_costmap_points(rawpoints,max_height=3.,**opts)
        new,_=split_costmap_points(rawpoints,max_height=1.5,**opts)
        report['height_replay']=dict(old_markings=len(old),new_markings=len(new),cells=[])
        # Inspect every lethal cell within 4 m of the axle, not just body edges.
        g=grids['local'];idx=np.flatnonzero(np.asarray(g.data)>=100)
        wx=g.info.origin.position.x+(idx%g.info.width+.5)*g.info.resolution
        wy=g.info.origin.position.y+(idx//g.info.width+.5)*g.info.resolution
        for k in np.flatnonzero(np.hypot(wx-xy[0],wy-xy[1])<4.):
            item=dict(world=[float(wx[k]),float(wy[k])])
            for label,marks in [('old',old),('new',new)]:
                w=marks@rotation.T+[cp.position.x,cp.position.y,cp.position.z]
                same=((np.floor((w[:,0]-g.info.origin.position.x)/g.info.resolution)==idx[k]%g.info.width)
                    & (np.floor((w[:,1]-g.info.origin.position.y)/g.info.resolution)==idx[k]//g.info.width))
                item[label+'_heights_base']=marks[same,2].tolist()
            report['height_replay']['cells'].append(item)
        # Save compact evidence around the car, enough to replay the height split.
        near=np.hypot(rawpoints[:,0],rawpoints[:,1])<10.
        args.output.parent.mkdir(parents=True,exist_ok=True)
        np.savez_compressed(args.output.with_name('cloud_'+str(len(out))+'.npz'), points=rawpoints[near],
            rotation=rotation,position=[cp.position.x,cp.position.y,cp.position.z],
            local_grid=np.asarray(g.data).reshape(g.info.height,g.info.width),
            origin=[g.info.origin.position.x,g.info.origin.position.y],resolution=g.info.resolution)
        out.append(report)
    args.output.write_text(json.dumps(out,indent=2))
    for r in out:
        print(json.dumps({k:r[k] for k in ('abort_time','position','speed','local','global','cloud','ground_plane')},indent=2))


if __name__ == '__main__':
    main()
