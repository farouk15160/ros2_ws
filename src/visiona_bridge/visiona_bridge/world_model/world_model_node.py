#!/usr/bin/env python3
import json, time, rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

class WorldModelNode(Node):
    def __init__(self):
        super().__init__("jarvis_world_model")
        self.declare_parameter("forget_timeout_sec", 15.0)
        self.declare_parameter("publish_rate", 1.0)
        self.declare_parameter("world_frame", "world")
        self.forget_timeout = self.get_parameter("forget_timeout_sec").value
        self.publish_rate = self.get_parameter("publish_rate").value
        self.world_frame = self.get_parameter("world_frame").value
        self.world = {}
        self.state_pub = self.create_publisher(String, "/jarvis/world_state", 10)
        self.markers_pub = self.create_publisher(MarkerArray, "/jarvis/world_model_markers", 10)
        self.create_subscription(String, "/jarvis/pose_status", self._on_pose_update, 10)
        self.create_timer(1.0 / self.publish_rate, self._publish_world)
        self.get_logger().info(f"[JARVIS World Model] Ready.\n   Forget: {self.forget_timeout}s\n   Rate: {self.publish_rate}Hz")

    def _on_pose_update(self, msg: String):
        try: objects = json.loads(msg.data)
        except: return
        now = time.time()
        for obj in objects:
            name = obj.get("name", "unknown")
            safe = obj.get("safe_name", name.replace(" ", "_").lower())
            e = {"name": name, "safe_name": safe, "tf_frame": f"detected_{safe}",
                 "x": obj.get("x", 0.0), "y": obj.get("y", 0.0), "z": obj.get("z", 0.0),
                 "depth_m": obj.get("depth_m", 0.0), "last_seen": now}
            is_new = safe not in self.world; self.world[safe] = e
            if is_new: self.get_logger().info(f"NEW: {name} @ ({e['x']:.3f},{e['y']:.3f},{e['z']:.3f})")

    def _publish_world(self):
        now = time.time()
        stale = [k for k,v in self.world.items() if now-v["last_seen"]>self.forget_timeout]
        for k in stale:
            self.get_logger().info(f"Forgot: {self.world[k]['name']}"); del self.world[k]
        wd = {k: {"name":v["name"],"tf_frame":v["tf_frame"],
                  "position":{"x":v["x"],"y":v["y"],"z":v["z"]},
                  "depth_m":v["depth_m"],"age_sec":round(now-v["last_seen"],1)}
              for k,v in self.world.items()}
        m = String(); m.data = json.dumps(wd, indent=2); self.state_pub.publish(m)
        ma = MarkerArray()
        clr = Marker(); clr.action = Marker.DELETEALL; clr.ns = "jw"
        clr.header.frame_id = self.world_frame; clr.header.stamp = self.get_clock().now().to_msg()
        ma.markers.append(clr)
        for i, (s, v) in enumerate(self.world.items()):
            age = now-v["last_seen"]; alpha = max(0.3, 1.0-age/self.forget_timeout)
            stamp = self.get_clock().now().to_msg()
            sp = Marker(); sp.header.frame_id = self.world_frame; sp.header.stamp = stamp
            sp.ns = "jw"; sp.id = i*2+1; sp.type = Marker.SPHERE; sp.action = Marker.ADD
            sp.pose.position.x = v["x"]; sp.pose.position.y = v["y"]; sp.pose.position.z = v["z"]
            sp.pose.orientation.w = 1.0; sp.scale.x = sp.scale.y = sp.scale.z = 0.05
            sp.color.r = 1.0; sp.color.g = 0.4; sp.color.a = alpha
            lb = Marker(); lb.header = sp.header; lb.ns = "jw"; lb.id = i*2+2
            lb.type = Marker.TEXT_VIEW_FACING; lb.action = Marker.ADD
            lb.pose.position.x = v["x"]; lb.pose.position.y = v["y"]; lb.pose.position.z = v["z"]+0.07
            lb.pose.orientation.w = 1.0; lb.scale.z = 0.03
            lb.color.r = lb.color.g = lb.color.b = lb.color.a = alpha; lb.text = v["name"]
            ma.markers.extend([sp, lb])
        self.markers_pub.publish(ma)

def main(args=None):
    rclpy.init(args=args); node = WorldModelNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__": main()
