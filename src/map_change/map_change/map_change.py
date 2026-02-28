import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import time
import yaml

class Nav2Client(Node):

    def __init__(self, initial_count=0):
        super().__init__('nav2_send_goal')
        self.count = initial_count
        self.renew = 0

        self.publisher = self.create_publisher(String, 'waypoint_count', 10)
        self.publisher_current_pose = self.create_publisher(PoseStamped, 'current_pose_info', 10)

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.file_path = "/home/kuri-tadaoki/turtlebot3_ws/src/sirius_navigation/src/example_point2.yaml"
        with open(self.file_path, 'r') as stream:
            self.node = yaml.safe_load(stream)
        self.goal_points = self.node["points"]

        self.set_feedback()

    def set_feedback(self):
        self.send_goal_options = None
        self.send_goal_options.feedback_callback = self.feedback_callback
        self.send_goal_options.result_callback = self.result_callback

    def send_goal(self):
        while not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for action server...')

        self.get_logger().info('count = %d', self.count)

        if self.count < len(self.goal_points):
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.header.frame_id = 'map'

            goal_msg.pose.pose.position.x = self.goal_points[self.count][0]
            goal_msg.pose.pose.position.y = self.goal_points[self.count][1]
            goal_msg.pose.pose.position.z = self.goal_points[self.count][2]
            goal_msg.pose.pose.orientation.x = self.goal_points[self.count][3]
            goal_msg.pose.pose.orientation.y = self.goal_points[self.count][4]
            goal_msg.pose.pose.orientation.z = self.goal_points[self.count][5]
            goal_msg.pose.pose.orientation.w = self.goal_points[self.count][6]

            # AsyncSendGoalを使用して非同期でゴールを送信
            goal_handle_future = self.client.async_send_goal(goal_msg)
            goal_handle_future.add_done_callback(self.result_callback)
            goal_handle_future.result().add_done_callback(self.feedback_callback)
            self.get_logger().info('send goal')

        else:
            subprocess.run(["bash", "nav2_nodekill.bash"])
            subprocess.run(["bash", "nav2_bringup.bash"])

    def feedback_callback(self, feedback):
        self.get_logger().info('current_pose = %f', feedback.current_pose.pose.position.x)
        self.get_logger().info('Distance remaining = %f', feedback.distance_remaining)

        pose_message = PoseStamped()
        pose_message.header = feedback.current_pose.header
        pose_message.pose = feedback.current_pose.pose
        self.publisher_current_pose.publish(pose_message)

        if feedback.distance_remaining > 0.5:
            self.renew = 1

        if feedback.distance_remaining < 0.5 and self.renew == 1:
            self.count += 1
            message = String()
            message.data = str(self.count)
            self.publisher.publish(message)
            self.renew = 0

            self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
            self.send_goal()
            self.set_feedback()

    def result_callback(self, future):
        result = future.result()
        if result.code == ActionClient.ResultCode.SUCCEEDED:
            self.get_logger().info('Success!!!')
        elif result.code == ActionClient.ResultCode.ABORTED:
            self.get_logger().error('Goal was aborted')
        elif result.code == ActionClient.ResultCode.CANCELED:
            self.get_logger().error('Goal was canceled')
        else:
            self.get_logger().error('Unknown result code')

def main(args=None):
    rclpy.init(args=args)

    initial_count = 0
    if args is not None and len(args) > 1:
        try:
            initial_count = int(args[1])
        except ValueError as e:
            print(f'Invalid argument for count: {e}')
            return 1

    node = Nav2Client(initial_count)
    node.send_goal()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

