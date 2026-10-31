"""ROS-independent, read-only scene projection and bounded object labels."""
from dataclasses import dataclass
import math


def graph_primitives(graph):
    """Project actual graph snapshots into XY; never invent missing nodes."""
    nodes=[]; frontiers=[]; frontier_points=[]; edges=[]
    positions={}
    for index,node in enumerate(graph.nodes):
        p=node.pose.position
        if not all(math.isfinite(v) for v in (p.x,p.y,p.z)):continue
        point=(float(p.x),float(p.y));positions[index]=point;nodes.append(point)
        if any(v.is_frontier for v in node.trav_properties):frontiers.append(point)
        for properties in node.trav_properties:
            if properties.is_frontier:
                for q in properties.frontier_points:
                    if all(math.isfinite(v) for v in (q.x,q.y,q.z)):
                        frontier_points.append((float(q.x),float(q.y)))
    for edge in graph.edges:
        if edge.from_idx in positions and edge.to_idx in positions:
            edges.extend((positions[edge.from_idx],positions[edge.to_idx]))
    return dict(nodes=nodes,frontiers=frontiers,frontier_points=frontier_points,edges=edges)


@dataclass
class ObjectLabel:
    serial: int
    label: str
    frame: str
    position: tuple
    stamp_ns: int
    confirmed: bool
    object_id: str


class ObjectLabelIndex:
    """Display cache only: no commands, scoring or navigation confirmation."""
    def __init__(self,capacity=128):
        self.capacity=max(1,int(capacity));self.items=[];self.serial=0
        self.frame=None;self.query='';self.query_started_ns=0

    def set_frame(self,frame):
        if frame!=self.frame:
            self.items=[];self.frame=frame

    def set_query(self,query,now_ns):
        query=' '.join(str(query).split())
        if query!=self.query:
            self.query=query;self.query_started_ns=now_ns

    def observe(self,data,now_ns,*,confirmed=False):
        if not isinstance(data,dict) or not self.frame or not self.query:return False
        label=' '.join(str(data.get('query','')).split())
        if label!=self.query or data.get('frame')!=self.frame:return False
        if data.get('localization_valid') is not True:return False
        if confirmed and data.get('position_confirmed') is not True:return False
        point=data.get('object_position') if confirmed else data.get('position')
        stamp=data.get('observation_stamp_ns')
        if (not isinstance(stamp,int) or isinstance(stamp,bool) or stamp<=0
                or stamp<self.query_started_ns or not 0<=now_ns-stamp<=5_000_000_000):return False
        try:
            if not isinstance(point,(list,tuple)) or len(point)!=3:return False
            point=tuple(float(v) for v in point)
            if not all(math.isfinite(v) for v in point):return False
        except (TypeError,ValueError,OverflowError):return False
        identity=str(data.get('object_id') or data.get('instance_id') or '')
        candidates=[v for v in self.items if v.label==label and v.frame==self.frame
                    and ((identity and v.object_id==identity) or
                         (not identity and not v.object_id and math.dist(v.position,point)<=3.))]
        match=min(candidates,key=lambda v:math.dist(v.position,point)) if candidates else None
        if match:
            if stamp<match.stamp_ns or (match.confirmed and not confirmed):return False
            if stamp==match.stamp_ns and (not confirmed or match.confirmed):return False
            match.position=point;match.stamp_ns=stamp;match.confirmed|=confirmed
        else:
            self.serial+=1
            self.items.append(ObjectLabel(self.serial,label,self.frame,point,stamp,confirmed,identity))
            if len(self.items)>self.capacity:
                self.items.remove(min(self.items,key=lambda v:(v.confirmed,v.stamp_ns)))
        return True

    def visible(self,now_ns):
        # Tentative localization expires. Confirmed historical labels are
        # explicitly retained within this frame/session, never used as goals.
        self.items=[v for v in self.items if v.confirmed or 0<=now_ns-v.stamp_ns<=5_000_000_000]
        return tuple(self.items)
