import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('initial_pose_publisher')

    publisher = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.pose.pose.position.x = 0.0
    msg.pose.pose.position.y = 0.0
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation.x = 0.0
    msg.pose.pose.orientation.y = 0.0
    msg.pose.pose.orientation.z = 0.0
    msg.pose.pose.orientation.w = 1.0

    while rclpy.ok():
        publisher.publish(msg)
        node.get_logger().info('Publishing: %s' % msg)
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

