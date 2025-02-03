import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from citrack_ros_msgs.msg import MultiTags
import hashlib

class Tag2MarkerPublisher(Node):
    def __init__(self):
        super().__init__('tag_to_marker_publisher')
        self.publisher = self.create_publisher(Marker, '/viz_marker_dwm1001', 100)
        self.publisher_kf = self.create_publisher(Marker,'viz_marker_dwm1001_kf', 100)
        # self.publisher = self.create_publisher(MarkerArray, '/viz_marker_dwm1001', 100)
        # self.publisher_kf = self.create_publisher(MarkerArray, '/viz_marker_dwm1001_kf', 100)

        self.subscription = self.create_subscription(MultiTags, '/dwm1001/multiTags', self.listener_callback, 10)
        self.subscription_kf = self.create_subscription(MultiTags, '/dwm1001/multiTags_kf', self.listener_callback_kf, 10)

    def frame_id_to_color(self, frame_id):
        """Generate a unique color based on the frame_id."""
        hash_object = hashlib.md5(frame_id.encode())
        hash_value = hash_object.hexdigest()
        r = int(hash_value[0:2], 16) / 255.0
        g = int(hash_value[2:4], 16) / 255.0
        b = int(hash_value[4:6], 16) / 255.0
        return ColorRGBA(r=r, g=g, b=b, a=0.8)  # Adjust alpha as needed

    def listener_callback(self, msg):
        # marker_array = MarkerArray()

        for tag in msg.tags_list:
            # Sphere Marker
            marker = Marker()
            marker.header.frame_id = "uwb_map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "sphere_marker"
            marker.id = hash(tag.header.frame_id) % 10000  # Unique ID
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = tag.pose_x
            marker.pose.position.y = tag.pose_y
            marker.pose.position.z = tag.pose_z
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color = self.frame_id_to_color(tag.header.frame_id)  # Unique color
            marker.lifetime.sec = 1  # Auto-remove old markers in RViz
            # marker_array.markers.append(marker)
            self.publisher.publish(marker)

            # Text Marker for displaying frame_id
            text_marker = Marker()
            text_marker.header.frame_id = "uwb_map"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "text_marker"
            text_marker.id = hash(tag.header.frame_id) % 10000 + 10000  # Unique ID offset for text
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = tag.pose_x
            text_marker.pose.position.y = tag.pose_y
            text_marker.pose.position.z = tag.pose_z + 0.7  # Slightly above the sphere
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.3  # Font size
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)  # White text
            text_marker.text = tag.header.frame_id  # Display frame_id
            text_marker.lifetime.sec = 1
            # marker_array.markers.append(text_marker)
            self.publisher.publish(text_marker)

        # self.publisher.publish(marker_array)

    def listener_callback_kf(self, msg):
        # marker_array = MarkerArray()

        for tag in msg.tags_list:
            # Sphere Marker for KF
            marker = Marker()
            marker.header.frame_id = "uwb_map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "kf_sphere_marker"
            marker.id = hash(tag.header.frame_id) % 10000 + 20000  # Unique ID for KF markers
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = tag.pose_x
            marker.pose.position.y = tag.pose_y
            marker.pose.position.z = tag.pose_z
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            # marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.5)  # Green color for KF
            marker.color = self.frame_id_to_color(tag.header.frame_id)  # Unique color  
            marker.lifetime.sec = 1  # Auto-remove old markers in RViz
            # marker_array.markers.append(marker)          
            self.publisher_kf.publish(marker)

            # Text Marker for displaying frame_id
            text_marker = Marker()
            text_marker.header.frame_id = "uwb_map"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "text_marker"
            text_marker.id = hash(tag.header.frame_id) % 10000 + 40000  # Unique ID offset for text
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = tag.pose_x
            text_marker.pose.position.y = tag.pose_y
            text_marker.pose.position.z = tag.pose_z + 0.7  # Slightly above the sphere
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.3  # Font size
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)  # White text
            text_marker.text = tag.header.frame_id  # Display frame_id
            text_marker.lifetime.sec = 1
            # marker_array.markers.append(text_marker)
            self.publisher_kf.publish(text_marker)

        # self.publisher_kf.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    tag_to_marker_publisher = Tag2MarkerPublisher()
    rclpy.spin(tag_to_marker_publisher)
    tag_to_marker_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

