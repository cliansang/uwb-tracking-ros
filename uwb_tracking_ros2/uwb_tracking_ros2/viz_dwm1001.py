import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from citrack_ros_msgs.msg import MultiTags

class Tag2MarkerPublisher(Node):
    def __init__(self):
        super().__init__('tag_to_marker_publisher')
        self.publisher = self.create_publisher(Marker, '/viz_marker_dwm1001', 10)
        self.subscription = self.create_subscription(MultiTags, '/dwm1001/multiTags', self.listener_callback, 10)
        # self.subscription_kf = self.create_subscription(MultiTags, '/dwm1001/multiTags_kf', self.listener_callback, 10)

    def listener_callback(self, msg):
        # print(msg.tags_list)
        for tag in msg.tags_list:
            # print(tag.header.frame_id)
            marker = Marker()
            marker.header.frame_id = "uwb_map"  # replace with the appropriate frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = tag.header.frame_id
            # marker.ns = tag.id
            # marker.id = int(tag.id)  # assuming tag id can be cast to an int            
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = tag.pose_x
            marker.pose.position.y = tag.pose_y
            marker.pose.position.z = tag.pose_z
            marker.pose.orientation.w = 1.0  # identity quaternion
            marker.scale.x = 0.1  # adjust size as needed
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.5)  # red color, adjust as needed
            self.publisher.publish(marker)

    def listener_callback_kf(self, msg):
        # print(msg.tags_list)
        for tag in msg.tags_list:
            # print(tag.header.frame_id)
            marker = Marker()
            marker.header.frame_id = "uwb_map"  # replace with the appropriate frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = tag.header.frame_id
            marker.type = Marker.SPHERE
            maker.action = Marker.ADD
            marker.pose.position.x = tag.pose_x 
            marker.pose.position.y = tag.pose_y
            marker.pose.position.z = tag.pose_z
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.5)
            self.publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    tag_to_marker_publisher = Tag2MarkerPublisher()
    rclpy.spin(tag_to_marker_publisher)
    tag_to_marker_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
