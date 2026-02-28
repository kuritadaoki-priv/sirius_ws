import sys
import playsound
import time
import subprocess
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge, CvBridgeError
from rclpy.utilities import remove_ros_args
from tf2_ros import TransformBroadcaster
from yolov5_ros2.detector import Detector, parse_opt
from nav2_msgs.action import NavigateToPose
from message_filters import ApproximateTimeSynchronizer, Subscriber
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class ObjectDetection(Node):

    def __init__(self, **args):
        super().__init__('object_detection')
        self.target_visible = False
        
        self.subscription = self.create_subscription(
            String,'waypoint_count', self.count_callback, 10)
        
       
#        self.odom_subscription = self.create_subscription(
#            Odometry, 'odom', self.odom_callback, 10)

        self.current_pose_subscription = self.create_subscription(
            PoseStamped, 'current_pose_info', self.odom_callback, 10)

        self.current_msg = NavigateToPose.Feedback()
        #self.subscription
        #self.odom_subscription
        
        
        self.target_name = 'bottle'
        self.frame_id = 'target'

        self.detector = Detector(**args)
        self.goal_msg = None 
        self.bridge = CvBridge()

        self.node = rclpy.create_node('nav2_send_goal')
        self.client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')

        self.ts = ApproximateTimeSynchronizer(
            [Subscriber(self, CameraInfo, 'camera/aligned_depth_to_color/camera_info'),
             Subscriber(self, Image, 'camera/color/image_raw'),
             Subscriber(self, Image, 'camera/aligned_depth_to_color/image_raw')],
            slop=0.1, queue_size=10) 
        self.ts.registerCallback(self.images_callback)

        self.broadcaster = TransformBroadcaster(self)
        
        self.count_value = 0 
 
        self.odom_list = [[0,0,0,0,0,0,0]]
        self.target_list = [[0,0,0]]

    def odom_callback(self, geometry_msgs):
        current_x = geometry_msgs.pose.position.x
        current_y = geometry_msgs.pose.position.y
        current_z = geometry_msgs.pose.position.z
        current_qx = geometry_msgs.pose.orientation.x
        current_qy = geometry_msgs.pose.orientation.y
        current_qz = geometry_msgs.pose.orientation.z
        current_qw = geometry_msgs.pose.orientation.w

        self.odom_list.append([current_x, current_y, current_z, current_qx, current_qy, current_qz, current_qw])
        self.odom_list.pop(0)

    def feedback_callback(self, feedback_msg):
        if feedback_msg is not None and feedback_msg.feedback is not None and feedback_msg.feedback.current_pose is not None:
            current_pose = feedback_msg.feedback.current_pose
            current_x = current_pose.pose.position.x
            current_y = current_pose.pose.position.y
            current_z = current_pose.pose.position.z
            current_qx = current_pose.pose.orientation.x
            current_qy = current_pose.pose.orientation.y
            current_qz = current_pose.pose.orientation.z
            current_qw = current_pose.pose.orientation.w

            self.get_logger().info(f"現在地x: {current_x}")
            self.get_logger().info(f"現在地y: {current_y}")
            self.get_logger().info(f"姿勢qz: {current_qz}")
            self.get_logger().info(f"姿勢qw: {current_qw}")

            self.odom_list.append([current_x, current_y, current_z, current_qx, current_qy, current_qz, current_qw])
            self.odom_list.pop(0)
        else:
            self.get_logger().warn("feedback_msg is None or feedback_msg.feedback is None or feedback_msg.feedback.current_pose is None")


    def count_callback(self, msg):
        count_value = int(msg.data)
        self.get_logger().info(f"受信したカウント: {count_value}")
        self.count_value_list = [0] 
        self.count_value_list.append(count_value)
        #self.count_value_list.pop(0)

    def send_goal(self, goal_msg):
        front = 0.3
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'map'
        #原点x方向正向きのとき
        if abs(self.odom_list[-1][5])<0.25 and abs(self.odom_list[-1][6])>0.85:
            goal_msg.pose.pose.position.x = float(self.odom_list[-1][0] + self.target_list[-1][2] - front)
            goal_msg.pose.pose.position.y = float(self.odom_list[-1][1] - ((self.target_list[-1][0])*(self.target_list[-1][2]-front)/self.target_list[-1][2]))
            goal_msg.pose.pose.position.z = 0.2
            goal_msg.pose.pose.orientation.x = float(self.odom_list[-1][3])
            goal_msg.pose.pose.orientation.y = float(self.odom_list[-1][4])
            goal_msg.pose.pose.orientation.z = float(self.odom_list[-1][5])
            goal_msg.pose.pose.orientation.w = float(self.odom_list[-1][6])

        #原点x方向負向きのとき
        if abs(self.odom_list[-1][6])<0.25 and abs(self.odom_list[-1][5])>0.85:
            goal_msg.pose.pose.position.x = float(self.odom_list[-1][0] - self.target_list[-1][2] + front)
            goal_msg.pose.pose.position.y = float(self.odom_list[-1][1] + ((self.target_list[-1][0])*(self.target_list[-1][2]-front)/self.target_list[-1][2]))
            goal_msg.pose.pose.position.z = 0.2
            goal_msg.pose.pose.orientation.x = float(self.odom_list[-1][3])
            goal_msg.pose.pose.orientation.y = float(self.odom_list[-1][4])
            goal_msg.pose.pose.orientation.z = float(self.odom_list[-1][5])
            goal_msg.pose.pose.orientation.w = float(self.odom_list[-1][6])
            

        #原点y軸方向正の向き
        if(0.5 <= abs(self.odom_list[-1][5]) <= 0.85 and  
            self.odom_list[-1][5] * self.odom_list[-1][6] > 0
            ):
            goal_msg.pose.pose.position.x = float(self.odom_list[-1][0] + ((self.target_list[-1][0])*(self.target_list[-1][2]-front)/self.target_list[-1][2]))
            goal_msg.pose.pose.position.y = float(self.odom_list[-1][1] + self.target_list[-1][2] -front)
            goal_msg.pose.pose.position.z = 0.2
            goal_msg.pose.pose.orientation.x = float(self.odom_list[-1][3])
            goal_msg.pose.pose.orientation.y = float(self.odom_list[-1][4])
            goal_msg.pose.pose.orientation.z = float(self.odom_list[-1][5])
            goal_msg.pose.pose.orientation.w = float(self.odom_list[-1][6])
            

        #原点y軸方向負の向き
        if(0.5 <= abs(self.odom_list[-1][5]) < 0.85 and 
            self.odom_list[-1][5] * self.odom_list[-1][6] < 0
            ):
            goal_msg.pose.pose.position.x = float(self.odom_list[-1][0] - ((self.target_list[-1][0])*(self.target_list[-1][2]-front)/self.target_list[-1][2]))
            goal_msg.pose.pose.position.y = float(self.odom_list[-1][1] - self.target_list[-1][2] + front)
            goal_msg.pose.pose.position.z = 0.2
            goal_msg.pose.pose.orientation.x = float(self.odom_list[-1][3])
            goal_msg.pose.pose.orientation.y = float(self.odom_list[-1][4])
            goal_msg.pose.pose.orientation.z = float(self.odom_list[-1][5])
            goal_msg.pose.pose.orientation.w = float(self.odom_list[-1][6])
            
        send_goal_future = self.client.send_goal_async(goal_msg,  feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self.node, send_goal_future)


    def images_callback(self, msg_info, msg_color, msg_depth):
        self.goal_msg = NavigateToPose.Goal()
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
                self.target_list.append([
                    x, y, z
                ])
                self.target_list.pop(0)
                ts = TransformStamped()
                ts.header = msg_depth.header
                ts.child_frame_id = self.frame_id
                ts.transform.translation.x = x
                ts.transform.translation.y = y
                ts.transform.translation.z = z
                self.broadcaster.sendTransform(ts)


                if len(self.odom_list) > 0:
                    self.send_goal(self.goal_msg)

                if abs(ts.transform.translation.x) <= 0.3 and ts.transform.translation.z <=0.5:
                    playsound.playsound("eva_fla.mp3")
                    time.sleep(3)
                    self.target_visible = True
        
        elif self.target_visible == True:
            subprocess.Popen(["ros2", "run", "sirius_navigation", "move_goal", str(self.count_value_list[-1])])
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

#kuri-tadaoki
