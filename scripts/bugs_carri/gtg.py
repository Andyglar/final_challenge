#!/usr/bin/env python 
import rospy  
import numpy as np 
from geometry_msgs.msg import Twist , PoseStamped , PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class GoToGoal:  
    def __init__(self): 
        ###--- Inicio del Nodo ---###
        rospy.init_node('go_to_goal')
        rospy.on_shutdown(self.cleanup)

        ###--- Subscriptores ---###
        rospy.Subscriber("/odom", Odometry, self.odom_cb)  

        ###--- Publishers ---###
        self.pub_cmd_vel = rospy.Publisher('/gtg_twist', Twist, queue_size=1)  
        self.target_pub = rospy.Publisher('/target', PointStamped, queue_size=1)
        self.gtg_vector_pub = rospy.Publisher('/gtg_vector', PoseStamped, queue_size=1)
        self.at_goal_flag_pub = rospy.Publisher('/finish_path', Bool, queue_size=1)

        ###--- Constants ---###
        self.dt = 0.02
        self.path_points = np.array([[1.0, 1.0], [2.0, 1.0], [3.0, 1.0]] )
        self.index = 0
        self.x_target = self.path_points[self.index][0]
        self.y_target = self.path_points[self.index][1]
        print(self.x_target , self.y_target)

        ###--- Objetos ---###
        self.v_msg = Twist()
        self.flag_msg = Bool()
        self.target = PointStamped()
        self.pose_target = PoseStamped()
        rate = rospy.Rate(int(1.0/self.dt))

        ###--- Variables ---###
        self.finish_track = False
        self.clear_path = False
        self.distance = 6
        self.x_pos = 0.0 
        self.y_pos = 0.0
        self.theta_robot = 0.0
        self.v = 0
        self.w = 0
        self.th_gtg = 0

        while rospy.get_time() == 0: pass 

        while not rospy.is_shutdown(): 
            self.calc_gtg()
            self.fill_messages()
            flag = self.at_goal()

            if flag and self.index < len(self.path_points) - 1:
                self.index += 1
                self.x_target = self.path_points[self.index][0]
                self.y_target = self.path_points[self.index][1]

            elif flag and self.index == len(self.path_points) - 1:
                self.finish_track = True

            self.print_state()

            self.pub_cmd_vel.publish(self.v_msg)
            self.target_pub.publish(self.target)
            self.gtg_vector_pub.publish(self.pose_target)
            self.at_goal_flag_pub.publish(self.flag_msg)
            rate.sleep()

    def calc_gtg(self):
        kvmax = 0.3
        kwmax = 1.7
        av = 2
        aw = 2

        ed = np.sqrt((self.x_target - self.x_pos)**2 + (self.y_target - self.y_pos)**2)
        theta_target = np.arctan2(self.y_target - self.y_pos, self.x_target - self.x_pos)
        e_theta = theta_target - self.theta_robot
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta))

        kw = kwmax * (1 - np.exp(-aw * e_theta**2)) / abs(e_theta)
        w = kw * e_theta
        
        if abs(e_theta) > np.pi / 8:
            v = 0 
        else:
            kv = kvmax * (1 - np.exp(-av * ed**2)) / abs(ed)
            v = kv * ed
            v = min(v, 0.2)

        self.v = v
        self.w = w
        self.th_gtg = theta_target

    def at_goal(self):
        tolerance = 0.15
        self.distance = np.sqrt((self.x_target - self.x_pos)**2 + (self.y_target - self.y_pos)**2)
        at_goal_flag = self.distance <= tolerance
        return at_goal_flag

    def fill_messages(self):
        ###--- Finished track ---###
        self.flag_msg.data = self.finish_track

        ###--- GTG velocoties ---###
        self.v_msg.linear.x = self.v
        self.v_msg.angular.z = self.w

        ###--- Target ---###
        self.target.header.frame_id = "odom"
        self.target.point.x = self.x_target
        self.target.point.y = self.y_target

        ###--- GTG Reference ---###
        quat = quaternion_from_euler(0, 0, self.th_gtg)
        self.pose_target.header.frame_id = "odom"
        self.pose_target.pose.position.x = self.x_pos
        self.pose_target.pose.position.y = self.y_pos
        self.pose_target.pose.orientation.x = quat[0]
        self.pose_target.pose.orientation.y = quat[1]
        self.pose_target.pose.orientation.z = quat[2]
        self.pose_target.pose.orientation.w = quat[3]

    def odom_cb(self, msg):
        self.x_pos = msg.pose.pose.position.x
        self.y_pos = msg.pose.pose.position.y
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        _, _, self.theta_robot = euler_from_quaternion([x, y, z, w])

    def print_state(self):
        print("###################################")
        print("Posicion robot: " , self.x_pos , self.y_pos)
        print("Posicion target: " , self.x_target , self.y_target)
        print("Indice de coordenada: " , self.index)
        print("Distance: " , self.distance , 0.15)
        print("Flag target: " , self.at_goal())
        print("Flag recorrido: " , self.finish_track)
        print("###################################")

    def cleanup(self):  
        vel_msg = Twist()
        self.pub_cmd_vel.publish(vel_msg)

if __name__ == "__main__":
    GoToGoal()
