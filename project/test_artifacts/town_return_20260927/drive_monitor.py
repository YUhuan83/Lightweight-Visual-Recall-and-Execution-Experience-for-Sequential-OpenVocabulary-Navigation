import json, math, time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from object_search_msgs.msg import ObjectMaskWithTf
from std_msgs.msg import String
from beamng_msgs.msg import StateSensor
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

class TownReturnWatch(Node):
    def __init__(self, log_path):
        super().__init__('town_return_watch')
        self.log_path = log_path
        self.log_file = open(log_path, 'a', buffering=1)
        self.started = time.monotonic()
        self.stage = 'search'
        self.stage_started = self.started
        self.start_xy = None
        self.xy = None
        self.speed = float('inf')
        self.mask_streak = 0
        self.target_seen = False
        self.target_seen_at = None
        self.first_positive_at = None
        self.goal_after_positive = False
        self.return_goal = (715.5294, -16.4555)
        self.return_goal_source = 'last remembered gas-station approach'
        self.arrival_streak = 0
        self.last_query_sent = 0.0
        self.stop_future = None
        self.finish_at = None
        self.mask_positive = 0
        self.mask_fraction = 0.0
        self.query_pub = self.create_publisher(String, '/spot1/object_search/query', 10)
        self.create_subscription(Odometry, '/spot1/odom', self.on_odom, 10)
        self.create_subscription(StateSensor, '/vehicles/ego/sensors/state', self.on_state, 10)
        self.create_subscription(ObjectMaskWithTf, '/spot1/object_mask', self.on_mask, 10)
        self.create_subscription(PoseStamped, '/spot1/raw_object_waypoint', self.on_raw_waypoint, 10)
        self.create_subscription(PoseStamped, '/spot1/imgnav_waypoint', self.on_img_waypoint, 10)
        self.create_subscription(Path, '/spot1/graphnav_planner/path', self.on_path, 10)
        self.deactivate = self.create_client(ChangeState, '/controller_server/change_state')
        self.create_timer(1.0, self.tick)
        self.emit('monitor_ready', start_xy=self.start_xy, search_timeout_s=120, max_search_radius_m=20,
                  return_goal_xy=self.return_goal)

    def emit(self, event, **fields):
        row = {'t_monotonic': round(time.monotonic()-self.started, 3), 'stage': self.stage, 'event': event}
        row.update(fields)
        self.log_file.write(json.dumps(row, ensure_ascii=False) + '\n')
        print(json.dumps(row, ensure_ascii=False), flush=True)

    def on_odom(self, m):
        p=m.pose.pose.position
        self.xy=(float(p.x),float(p.y))
        if self.start_xy is None:
            self.start_xy=self.xy
            self.emit('spawn_odom', x=round(p.x,3),y=round(p.y,3),z=round(p.z,3))

    def on_state(self,m):
        v=m.velocity
        self.speed=math.sqrt(float(v.x)**2+float(v.y)**2+float(v.z)**2)

    def on_mask(self,m):
        data=np.asarray(m.object_mask.data,dtype=np.uint8)
        count=int(np.count_nonzero(data))
        frac=count/max(1,data.size)
        self.mask_positive=count
        self.mask_fraction=frac
        now=time.monotonic()
        if count>=500 and frac>=0.003:
            self.mask_streak+=1
            if self.first_positive_at is None: self.first_positive_at=now
            if not self.target_seen and self.mask_streak>=3:
                self.target_seen=True
                self.target_seen_at=now
                self.emit('trash_can_mask_stable',pixels=count,fraction=round(frac,5))
        else:
            self.mask_streak=0
        if count>=500 and frac>=0.003:
            self.emit('mask_positive',pixels=count,fraction=round(frac,5),streak=self.mask_streak)

    def on_raw_waypoint(self,m):
        p=m.pose.position
        now=time.monotonic()
        self.emit('raw_object_waypoint',x=round(p.x,3),y=round(p.y,3),z=round(p.z,3))
        if self.stage=='search' and self.first_positive_at and now>=self.first_positive_at:
            self.goal_after_positive=True

    def on_img_waypoint(self,m):
        p=m.pose.position
        now=time.monotonic()
        self.emit('approach_waypoint',x=round(p.x,3),y=round(p.y,3),z=round(p.z,3))
        if self.stage=='search' and self.first_positive_at and now>=self.first_positive_at:
            self.goal_after_positive=True

    def on_path(self,m):
        if not m.poses: return
        p=m.poses[-1].pose.position
        ex,ey=self.return_goal
        if self.stage=='return' and math.hypot(p.x-ex,p.y-ey)<=5.0:
            self.return_goal=(float(p.x),float(p.y))
            self.return_goal_source='post-switch graphnav path endpoint'
            self.emit('return_path_endpoint',x=round(p.x,3),y=round(p.y,3))

    def request_stop(self,reason):
        if self.stop_future is not None: return
        if not self.deactivate.service_is_ready():
            self.emit('controller_deactivate_unavailable',reason=reason)
            return
        req=ChangeState.Request()
        req.transition.id=Transition.TRANSITION_DEACTIVATE
        self.stop_future=self.deactivate.call_async(req)
        self.stop_future.add_done_callback(lambda f:self.emit('controller_deactivated',reason=reason,
            success=bool(f.result() and f.result().success)))

    def switch_to_return(self,reason):
        if self.stage!='search': return
        self.stage='return'
        self.stage_started=time.monotonic()
        self.arrival_streak=0
        self.emit('return_trigger',reason=reason,target_seen=self.target_seen,
                  max_displacement_m=round(math.hypot(self.xy[0]-self.start_xy[0],self.xy[1]-self.start_xy[1]),2)
                  if self.xy and self.start_xy else None)
        self.last_query_sent=0.0
        self.publish_return_query()

    def publish_return_query(self):
        self.query_pub.publish(String(data='gas station'))
        self.last_query_sent=time.monotonic()
        self.emit('return_query_published',query='gas station')

    def tick(self):
        now=time.monotonic()
        if self.stage=='search' and self.xy and self.start_xy:
            radius=math.hypot(self.xy[0]-self.start_xy[0],self.xy[1]-self.start_xy[1])
            elapsed=now-self.stage_started
            if self.target_seen and self.goal_after_positive:
                self.switch_to_return('trash_can_mask_and_approach_waypoint')
            elif self.target_seen and self.target_seen_at and now-self.target_seen_at>=15:
                self.switch_to_return('trash_can_mask_seen_no_triangulated_approach_within_15s')
            elif radius>=20.0:
                self.switch_to_return('20m_search_geofence')
            elif elapsed>=120.0:
                self.switch_to_return('120s_search_timeout')
            elif elapsed>=70.0 and radius<1.0:
                self.switch_to_return('no_departure_progress_70s')
            if self.stage=='search':
                self.emit('odom_sample',x=round(self.xy[0],3),y=round(self.xy[1],3),
                          speed=round(self.speed,3) if math.isfinite(self.speed) else None,
                          displacement=round(radius,3),mask_pixels=self.mask_positive,
                          mask_fraction=round(self.mask_fraction,5))
        elif self.stage=='return' and self.xy:
            if now-self.last_query_sent>=2.0 and now-self.stage_started<8.0:
                self.publish_return_query()
            d=math.hypot(self.xy[0]-self.return_goal[0],self.xy[1]-self.return_goal[1])
            if d<=2.5 and self.speed<=0.15:
                self.arrival_streak+=1
            else:
                self.arrival_streak=0
            self.emit('return_sample',x=round(self.xy[0],3),y=round(self.xy[1],3),
                      speed=round(self.speed,3) if math.isfinite(self.speed) else None,
                      goal_x=round(self.return_goal[0],3),goal_y=round(self.return_goal[1],3),
                      goal_source=self.return_goal_source,distance_to_goal=round(d,3),
                      arrival_streak=self.arrival_streak)
            if self.arrival_streak>=5:
                self.stage='complete'
                self.emit('return_verified',distance_to_goal=round(d,3),speed=round(self.speed,3),
                          target_seen=self.target_seen)
                self.request_stop('return_verified')
                self.finish_at=now+6
            elif now-self.stage_started>=180:
                self.stage='failed'
                self.emit('return_timeout',distance_to_goal=round(d,3),speed=round(self.speed,3),
                          target_seen=self.target_seen)
                self.request_stop('return_timeout')
                self.finish_at=now+6
        elif self.stage in ('complete','failed') and self.finish_at and now>=self.finish_at:
            self.log_file.close()
            rclpy.shutdown()

def main():
    import sys
    rclpy.init()
    n=TownReturnWatch(sys.argv[1])
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally:
        try: n.log_file.close()
        except Exception: pass
        if rclpy.ok(): rclpy.shutdown()

if __name__=='__main__': main()

