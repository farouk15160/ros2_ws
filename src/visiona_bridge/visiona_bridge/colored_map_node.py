#!/usr/bin/env python3
"""
Colored Point Cloud Accumulator Node
Accumulates RGB-D camera data into a persistent colored 3D map.
Uses TF to transform points to world frame as the robot arm moves.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, CameraInfo
from std_msgs.msg import Header
import numpy as np
import struct
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
import tf2_geometry_msgs


class ColoredMapAccumulator(Node):
    def __init__(self):
        super().__init__('colored_map_accumulator')
        
        # Parameters
        self.declare_parameter('voxel_size', 0.02)  # 2cm voxel resolution
        self.declare_parameter('max_points', 500000)  # Max accumulated points
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('publish_rate', 2.0)  # Hz
        
        self.voxel_size = self.get_parameter('voxel_size').value
        self.max_points = self.get_parameter('max_points').value
        self.world_frame = self.get_parameter('world_frame').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # TF buffer for transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Accumulated point cloud (world frame)
        # Dictionary: voxel_key -> (x, y, z, r, g, b, count)
        self.voxel_map = {}
        
        # Subscriber to depth point cloud with RGB
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/ascamera_hp60c/camera_publisher/depth0/points',
            self.pointcloud_callback,
            10
        )
        
        # Publisher for accumulated colored map
        self.map_pub = self.create_publisher(
            PointCloud2,
            '/colored_map',
            10
        )
        
        # Timer to publish accumulated map
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_map)
        
        self.get_logger().info('🎨 Colored Map Accumulator Started')
        self.get_logger().info(f'   Voxel size: {self.voxel_size}m')
        self.get_logger().info(f'   Max points: {self.max_points}')
        self.get_logger().info(f'   World frame: {self.world_frame}')
    
    def get_voxel_key(self, x, y, z):
        """Convert point to voxel key"""
        return (
            int(x / self.voxel_size),
            int(y / self.voxel_size),
            int(z / self.voxel_size)
        )
    
    def pointcloud_callback(self, msg: PointCloud2):
        """Process incoming point cloud and accumulate"""
        try:
            # Get transform from camera frame to world frame
            transform = self.tf_buffer.lookup_transform(
                self.world_frame,
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
        except Exception as e:
            self.get_logger().debug(f'TF lookup failed: {e}')
            return
        
        # Extract translation and rotation
        t = transform.transform.translation
        r = transform.transform.rotation
        
        # Convert quaternion to rotation matrix
        rot_matrix = self.quaternion_to_matrix(r.x, r.y, r.z, r.w)
        translation = np.array([t.x, t.y, t.z])
        
        # Parse point cloud
        points = self.parse_pointcloud(msg)
        if points is None or len(points) == 0:
            return
        
        # Transform points to world frame and accumulate
        new_points = 0
        for p in points:
            x, y, z, r, g, b = p
            
            # Skip invalid points
            if np.isnan(x) or np.isnan(y) or np.isnan(z):
                continue
            if z <= 0 or z > 3.0:  # Skip too close/far points
                continue
            
            # Transform to world frame
            point_cam = np.array([x, y, z])
            point_world = rot_matrix @ point_cam + translation
            
            # Get voxel key
            key = self.get_voxel_key(point_world[0], point_world[1], point_world[2])
            
            # Accumulate (average colors)
            if key in self.voxel_map:
                old = self.voxel_map[key]
                count = old[6] + 1
                # Running average for position and color
                self.voxel_map[key] = (
                    (old[0] * old[6] + point_world[0]) / count,
                    (old[1] * old[6] + point_world[1]) / count,
                    (old[2] * old[6] + point_world[2]) / count,
                    int((old[3] * old[6] + r) / count),
                    int((old[4] * old[6] + g) / count),
                    int((old[5] * old[6] + b) / count),
                    count
                )
            else:
                self.voxel_map[key] = (
                    point_world[0], point_world[1], point_world[2],
                    int(r), int(g), int(b), 1
                )
                new_points += 1
        
        # Limit total points
        if len(self.voxel_map) > self.max_points:
            # Remove oldest entries (simple FIFO by deleting random keys)
            keys_to_delete = list(self.voxel_map.keys())[:len(self.voxel_map) - self.max_points]
            for k in keys_to_delete:
                del self.voxel_map[k]
        
        if new_points > 0:
            self.get_logger().debug(f'Added {new_points} new voxels, total: {len(self.voxel_map)}')
    
    def parse_pointcloud(self, msg: PointCloud2):
        """Parse PointCloud2 message to extract XYZ and RGB"""
        # Find field offsets
        fields = {f.name: f for f in msg.fields}
        
        if 'x' not in fields or 'y' not in fields or 'z' not in fields:
            return None
        
        x_offset = fields['x'].offset
        y_offset = fields['y'].offset
        z_offset = fields['z'].offset
        
        # Check for RGB field
        has_rgb = 'rgb' in fields or 'rgba' in fields
        rgb_offset = fields.get('rgb', fields.get('rgba', None))
        if rgb_offset:
            rgb_offset = rgb_offset.offset
        
        points = []
        point_step = msg.point_step
        data = msg.data
        
        # Sample every Nth point for performance
        sample_step = max(1, len(data) // (point_step * 5000))
        
        for i in range(0, len(data) - point_step, point_step * sample_step):
            x = struct.unpack_from('f', data, i + x_offset)[0]
            y = struct.unpack_from('f', data, i + y_offset)[0]
            z = struct.unpack_from('f', data, i + z_offset)[0]
            
            if has_rgb and rgb_offset is not None:
                # RGB is packed as a float representing uint32
                rgb_packed = struct.unpack_from('f', data, i + rgb_offset)[0]
                rgb_int = struct.unpack('I', struct.pack('f', rgb_packed))[0]
                r = (rgb_int >> 16) & 0xFF
                g = (rgb_int >> 8) & 0xFF
                b = rgb_int & 0xFF
            else:
                # Default gray color
                r, g, b = 128, 128, 128
            
            points.append((x, y, z, r, g, b))
        
        return points
    
    def quaternion_to_matrix(self, x, y, z, w):
        """Convert quaternion to 3x3 rotation matrix"""
        return np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]
        ])
    
    def publish_map(self):
        """Publish accumulated colored point cloud"""
        if not self.voxel_map:
            return
        
        # Create PointCloud2 message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.world_frame
        
        # Fields: x, y, z, rgb
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        point_step = 16  # 4 floats * 4 bytes
        points_data = []
        
        for voxel in self.voxel_map.values():
            x, y, z, r, g, b, _ = voxel
            
            # Pack RGB as float
            rgb_int = (int(r) << 16) | (int(g) << 8) | int(b)
            rgb_float = struct.unpack('f', struct.pack('I', rgb_int))[0]
            
            points_data.append(struct.pack('ffff', x, y, z, rgb_float))
        
        data = b''.join(points_data)
        
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(self.voxel_map)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = point_step
        msg.row_step = point_step * len(self.voxel_map)
        msg.data = data
        msg.is_dense = True
        
        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ColoredMapAccumulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
