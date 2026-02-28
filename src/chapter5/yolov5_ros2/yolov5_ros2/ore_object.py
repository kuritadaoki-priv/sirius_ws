import sys
import time
import subprocess
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from rclpy.utilities import remove_ros_args
from tf2_ros import TransformBroadcaster
from yolov5_ros2.detector import Detector, parse_opt
from nav2_msgs.action import NavigateToPose
from message_filters import ApproximateTimeSynchronizer, Subscriber
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from action_msgs.srv import CancelGoal

class ObjectDetection(Node):

    def __init__(self, **args):
        super().__init__('object_detection')
        self.target_visible = False

        self.client = ActionClient(self, CancelGoal, 'cancel_goal')

        self.subscription = self.create_subscription(
            String,'waypoint_count', self.count_callback, 10)
        
        self.pub = self.create_publisher(Twist, "/cmd_vel",qos_profile=rclpy.qos.qos_profile_system_default)   
        self.twist = Twist()
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        l = 0.0
        a = 0.0
        try:
            self.linear_speed = float(l)
            self.linear_speed = float(a)
        except ValueError:
            print("Invalid input. Using default values for linear and angular speeds.")
        self.run(l,a)
        
        
        self.target_name = 'bottle'
        self.frame_id = 'target'

        self.detector = Detector(**args)
        self.goal_msg = None 
        self.bridge = CvBridge()

        self.ts = ApproximateTimeSynchronizer(
            [Subscriber(self, CameraInfo, 'camera/aligned_depth_to_color/camera_info'),
             Subscriber(self, Image, 'camera/color/image_raw'),
             Subscriber(self, Image, 'camera/aligned_depth_to_color/image_raw')],
            slop=0.1, queue_size=10) 
        self.ts.registerCallback(self.images_callback)

        self.broadcaster = TransformBroadcaster(self)
        
        self.count_value = 0

    def count_callback(self, msg):
        count_value = int(msg.data)
        self.get_logger().info(f"受信したカウント: {count_value}")
        self.count_value_list = [0] 
        self.count_value_list.append(count_value)
        self.count_value_list.pop(0)


    def run(self, l, a):
        self.twist.linear.x = float(l)
        self.twist.angular.z = float(a)
        self.pub.publish(self.twist)



    async def cancel_navigation_goal(self):
        goal_handle = await self.client.wait_for_server()
        cancel_goal_msg = CancelGoal.Goal()
        future = self.client.async_send_goal(cancel_goal_msg)
        await future
        if future.done():
            cancel_response = future.result()
            self.get_logger().info(f"Navigation goal cancellation response: {cancel_response}")


    def images_callback(self, msg_info, msg_color, msg_depth):
        #self.goal_msg = NavigateToPose.Goal()
        #self.get_logger().info("Callback called")
        try:
            img_color = self.bridge.imgmsg_to_cv2(msg_color, 'bgr8')
            img_depth = self.bridge.imgmsg_to_cv2(msg_depth, 'passthrough')
        except CvBridgeError as e:
            self.get_logger().warn(str(e))
            return

        if img_color.shape[0:2] != img_depth.shape[0:2]:
            self.get_logger().warn('カラーと深度の画像サイズが異なる')
            return

        img_color, result = self.detector.detect(img_color)

        cv2.imshow('color', img_color)

        
        target = None
        if self.target_visible == False:
            for r in result:
                if r.name == self.target_name:
                    target = r
                    break

        if target is not None:
            self.cancel_navigation_goal()
            u1 = round(target.u1)
            u2 = round(target.u2)
            v1 = round(target.v1)
            v2 = round(target.v2)
            u = round((target.u1 + target.u2) / 2)
            v = round((target.v1 + target.v2) / 2)
            depth = np.median(img_depth[v1:v2+1, u1:u2+1])
            if depth != 0:
                z = depth * 1e-3
                fx = float(msg_info.k[0])
                fy = float(msg_info.k[4])
                cx = float(msg_info.k[2])
                cy = float(msg_info.k[5])
                x = z / fx * (u - cx)
                y = z / fy * (v - cy)
                self.get_logger().info(
                    f'{target.name} ({x:.3f}, {y:.3f}, {z:.3f})')
                ts = TransformStamped()
                ts.header = msg_depth.header
                ts.child_frame_id = self.frame_id
                ts.transform.translation.x = x
                ts.transform.translation.y = y
                ts.transform.translation.z = z
                self.broadcaster.sendTransform(ts)

                #ロボット制御部分
                if ts.transform.translation.x >= 0.03 and ts.transform.translation.z >=0.5:
                    self.twist.linear.x = 0.1
                    self.twist.angular.z = -0.7
                    self.pub.publish(self.twist)
                    time.sleep(3)

                elif ts.transform.translation.x <= -0.03 and ts.transform.translation.z >=0.5:
                    self.twist.linear.x = 0.1
                    self.twist.angular.z = 0.7
                    self.pub.publish(self.twist)
                    time.sleep(3)

                elif abs(ts.transform.translation.x) <= 0.03 and ts.transform.translation.z >=0.5:
                    self.twist.linear.x = 0.2
                    self.twist.angular.z = 0.0
                    self.pub.publish(self.twist)
                    time.sleep(3)

                elif abs(ts.transform.translation.x) >= 0.03 and ts.transform.translation.z <=0.7:
                    self.twist.linear.x = -0.2
                    self.twist.angular.z = 0.0
                    self.pub.publish(self.twist)
                    time.sleep(3)



                elif abs(ts.transform.translation.x) <= 0.03 and ts.transform.translation.z <=0.5:
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = 0.0
                    self.pub.publish(self.twist)
                    time.sleep(5)
                    #sys.exit()
                    print("fin!!")
                    self.target_visible = True

        elif self.target_visible == True:
            subprocess.Popen(["bash", "move_goal.bash", str(self.count_value_list[-1])])
            self.target_visible = False
                    


        img_depth *= 16
        if target is not None:
            pt1 = (int(target.u1), int(target.v1))
            pt2 = (int(target.u2), int(target.v2))
            cv2.rectangle(img_depth, pt1=pt1, pt2=pt2, color=0xffff)

        cv2.imshow('depth', img_depth)
        cv2.waitKey(1)

def main():
    rclpy.init()
    opt = parse_opt(remove_ros_args(args=sys.argv))
    node = ObjectDetection(**vars(opt))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()

