#!/bin/python3

import math
import rclpy
import rclpy.node
import threading
import time
import numpy as np


from pymavlink import mavutil

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3Stamped
from arfour_msgs.msg import Setpoint
#from arfour_msgs.action import Takeoff

IDLE = -1
READY = 0
TAKEOFF = 1
HOVER = 2
AUTO = 3
MANUAL = 4
LAND = 5
ERROR = 6

def saturate(val,min,max):
    if(val > max):
        val = max

    if(val < min):
        val = min
    return val

def wrapToPi(ang):

    if(ang < -math.pi):
        ang += math.pi*2

    if(ang > math.pi):
        ang += math.pi*2

    return ang

class arfourInterface(rclpy.node.Node):
    def __init__(self):
        super().__init__('drone_interface_node')

        self.max_roll = 24*math.pi/180
        self.max_pitch = 24*math.pi/180
        self.max_yawrate = 30*math.pi/180
        self.max_spd_xy = 1.0
        self.max_spd_z = 1.0
        self.max_thrust_i = 0.1
        self.wp_rad = 0.3

        self.hover_pos = [0.0, 0.0, 0.0]
        self.roll_cmd = 0.0
        self.pitch_cmd = 0.0
        self.yaw_cmd = 0.0
        self.thrust_cmd = 0.0

        self.roll_joy = 0.0
        self.pitch_joy = 0.0
        self.yaw_joy = 0.0
        self.thrust_joy = 0.0

        self.pos_err_x = 0.0
        self.pos_err_y = 0.0
        self.pos_err_z = 0.0
        self.pos_err_z_int = 0.0
        self.vel_err_x = 0.0
        self.vel_err_y = 0.0
        self.vel_err_z = 0.0
        self.vel_err_z_int = 0.0


        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.vel_xg = 0.0
        self.vel_yg = 0.0
        self.vel_zg = 0.0
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_z = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.takeoff_land_spd = 1.0
        self.thrust_hover = 0.55
        self.thrust_idle = 0.2
        self.takeoff_alt = 1.0
        self.Kyaw = 0.5
        self.Kpos_xy = 0.28
        self.Kpos_z = 0.06
        self.Kpos_z_d = 0.06
        self.Kpos_z_i = 0.02
        self.Kvel_x = 0.28
        self.Kvel_y = 0.28
        self.Kvel_z = 0.025
        self.Kvel_i_z = 0.02
        self.dt = 0.05
        self.kk = 0
        self.waypoints = np.array([[0,1.5,1.0],[2,1.5,1.0],[2,1.5,2.0],[0,1.5,2.0]])
        self.wp_n,m = np.shape(self.waypoints)

        self.flying = False        

        self.mode = IDLE
        self.prev_mode = IDLE
        self.mode_pos = [0.0,0.0,0.0]
    #    self.declare_parameter('my_parameter', 'world')

        self.imu_pub = self.create_publisher(Imu,'imu',10)

        self.set_pub = self.create_publisher(Setpoint,'setpoint',10)
        self.joy_sub = self.create_subscription(Joy,'joy',self.joy_callback,1)
        self.mocap_sub = self.create_subscription(Odometry,'/daisy/mocap/odom',self.mocap_callback,1)
        self.eul_sub = self.create_subscription(Vector3Stamped,'/daisy/eul',self.euler_callback,1)
        self.vel_sub = self.create_subscription(Vector3Stamped,'/daisy/vel',self.velocity_callback,1)

        self.timer = self.create_timer(0.05, self.send_loop)

        self.init_time = self.get_clock().now()



    def mocap_callback(self,msg):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        self.pos_z = msg.pose.pose.position.z

        self.vel_x = msg.twist.twist.linear.x
        self.vel_y = msg.twist.twist.linear.y
        self.vel_z = msg.twist.twist.linear.z

        if(self.pos_z > 0.1):
            self.flying = True
        else:
            self.flying = False
        #print(self.pos_x,self.pos_y,self.pos_z)

    def velocity_callback(self,msg):
        self.vel_xg = msg.vector.x
        self.vel_yg = msg.vector.y
        self.vel_zg = msg.vector.z

    def euler_callback(self,msg):
        self.roll = msg.vector.x
        self.pitch = msg.vector.y
        self.yaw = msg.vector.z      



    def send_loop(self):

        time_from_boot = self.get_clock().now() - self.init_time

        time_ms = int(time_from_boot.nanoseconds*1e-6)

        
        self.des_pos_x = self.waypoints[self.kk,0]       
        self.des_pos_y = self.waypoints[self.kk,1] 
        self.des_pos_z = self.waypoints[self.kk,2] 

        self.des_yaw = 0.0
        self.pos_err_x = self.des_pos_x - self.pos_x
        self.pos_err_y = self.des_pos_y - self.pos_y
        self.pos_err_z = self.des_pos_z - self.pos_z



        
        vel_err_xg =  self.Kpos_xy*self.pos_err_x     
        vel_err_yg =  self.Kpos_xy*self.pos_err_y
        des_vel_z = self.Kpos_z*self.pos_err_z

        self.vel_err_x = math.cos(self.yaw)*vel_err_xg + math.sin(self.yaw)*vel_err_yg
        self.vel_err_y = -math.sin(self.yaw)*vel_err_xg + math.cos(self.yaw)*vel_err_yg
        self.vel_err_z = des_vel_z - self.vel_zg

        if(self.flying):
            self.vel_err_z_int += self.vel_err_z*self.dt
            self.pos_err_z_int += self.pos_err_z*self.dt
        else:
            self.vel_err_z_int = 0.0
            self.pos_err_z_int = 0.0

        self.yaw_cmd = self.Kyaw*wrapToPi(self.des_yaw-self.yaw)
        self.pitch_cmd = self.Kvel_x*(self.vel_err_x)
        self.roll_cmd = -(self.Kvel_y*self.vel_err_y)        
        # self.thrust_cmd = self.Kvel_z*(self.vel_err_z) + self.Kvel_i_z*(self.vel_err_z_int) + self.thrust_hover
        self.thrust_cmd = self.Kpos_z*(self.pos_err_z) + self.Kpos_z_i*(self.pos_err_z_int) - self.Kpos_z_d*(self.vel_zg) + self.thrust_hover

        set_msg = Setpoint()
        set_msg.roll = self.roll_cmd
        set_msg.pitch = self.pitch_cmd
        set_msg.yaw = self.yaw_cmd
        set_msg.thrust = self.thrust_cmd

        self.set_pub.publish(set_msg)

        dist = math.sqrt(self.pos_err_x**2 + self.pos_err_y**2 + self.pos_err_z**2)

        if(dist < self.wp_rad):
            self.kk +=1
            if(self.kk == self.wp_n):
                self.kk =0

    def joy_callback(self,msg):
        pass
                                          

    def setpoint_callback(self,msg):

        time_from_boot = self.get_clock().now() - self.init_time

        time_ms = int(time_from_boot.nanoseconds*1e-6)
        self.roll_cmd = msg.roll
        self.pitch_cmd = msg.pitch
        self.yaw_cmd = msg.yaw
        self.thrust_cmd = msg.thrust
        #mode_cmd = msg.mode
        
        #self.mav_conn.mav.manual_setpoint_send(time_ms,roll_cmd,pitch_cmd,yaw_cmd,thrust_cmd, mode_cmd,0)



def main():
    rclpy.init()
    node = arfourInterface()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
