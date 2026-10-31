"""Bounded rear-axle pursuit preview for admitting an actual-pose handoff.

This is a geometric rollout, not dynamics or a substitute for the controller's
live collision checks. It follows nearest-pose pruning and circle lookahead;
the full section must capture its actual checker disk before changing gear.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class TrackingPreview:
    reached: bool
    reason: str | None
    closest_m: float
    trajectory: tuple


def preview_tracking(points, headings, position, heading, radius, *, lookahead=3.,
                     position_tolerance=.2, heading_tolerance=.35, step=.025,
                     search_distance=10., local_extent=8., max_steps=4000):
    values=(*position,heading,radius,lookahead,position_tolerance,heading_tolerance,step,search_distance,local_extent)
    if (len(points)<2 or len(points)!=len(headings) or len(position)!=2
            or not all(math.isfinite(x) for x in values)
            or min(radius,lookahead,position_tolerance,heading_tolerance,step,search_distance,local_extent)<=0
            or max_steps<1 or max_steps>10000
            or any(len(p)!=2 or not all(math.isfinite(x) for x in p) for p in points)
            or not all(math.isfinite(h) for h in headings)):
        raise ValueError('Invalid tracking preview input')
    direction=None
    for i,(a,b) in enumerate(zip(points,points[1:])):
        dx,dy=b[0]-a[0],b[1]-a[1]
        if math.hypot(dx,dy)<1e-9: continue
        longitudinal=dx*math.cos(headings[i])+dy*math.sin(headings[i])
        if abs(longitudinal)<1e-9: raise ValueError('Sideways tracking edge')
        edge_direction=1 if longitudinal>0 else -1
        if direction is not None and direction!=edge_direction:
            raise ValueError('Tracking preview requires one motion direction')
        direction=edge_direction
    if direction is None: raise ValueError('Tracking section has no motion')
    x,y=position; angle=heading; first=0; closest=math.inf; trajectory=[]
    end=points[-1]; end_heading=headings[-1]
    for _ in range(max_steps+1):
        gap=math.hypot(end[0]-x,end[1]-y); closest=min(closest,gap)
        trajectory.append((x,y,angle))
        yaw_error=abs(math.remainder(angle-end_heading,2*math.pi))
        if gap<=position_tolerance and yaw_error<=heading_tolerance:
            return TrackingPreview(True,None,closest,tuple(trajectory))
        last=first; walked=0.
        while last+1<len(points) and walked<search_distance:
            walked+=math.dist(points[last],points[last+1]);last+=1
        first=min(range(first,last+1),key=lambda i:math.hypot(points[i][0]-x,points[i][1]-y))
        clipped=[]
        for p in points[first:]:
            if math.hypot(p[0]-x,p[1]-y)>local_extent: break
            clipped.append(p)
        if not clipped:
            return TrackingPreview(False,'tracking_preview_outside_local_window',closest,tuple(trajectory))
        target=clipped[-1]
        for i,p in enumerate(clipped):
            if math.hypot(p[0]-x,p[1]-y)<lookahead: continue
            target=p
            if i:
                a=clipped[i-1];dx,dy=p[0]-a[0],p[1]-a[1]
                length2=dx*dx+dy*dy
                if length2>1e-12:
                    ax,ay=a[0]-x,a[1]-y
                    linear=2*(ax*dx+ay*dy);constant=ax*ax+ay*ay-lookahead*lookahead
                    discriminant=linear*linear-4*length2*constant
                    if discriminant>=0:
                        t=(-linear+math.sqrt(discriminant))/(2*length2)
                        if 0<=t<=1: target=(a[0]+t*dx,a[1]+t*dy)
            break
        dx,dy=target[0]-x,target[1]-y
        bx=dx*math.cos(angle)+dy*math.sin(angle)
        by=-dx*math.sin(angle)+dy*math.cos(angle)
        if direction*(1 if bx>=0 else -1)<0:
            return TrackingPreview(False,'tracking_preview_would_reverse',closest,tuple(trajectory))
        squared=bx*bx+by*by
        requested=2*by/squared if squared>.001 else 0.
        curvature=max(-1/radius,min(1/radius,requested))
        signed=direction*step; turn=signed*curvature
        lx=math.sin(turn)/curvature if abs(curvature)>1e-10 else signed
        ly=(1-math.cos(turn))/curvature if abs(curvature)>1e-10 else 0.
        x,y=x+lx*math.cos(angle)-ly*math.sin(angle),y+lx*math.sin(angle)+ly*math.cos(angle)
        angle+=turn
    return TrackingPreview(False,'tracking_preview_budget_exhausted',closest,tuple(trajectory))
