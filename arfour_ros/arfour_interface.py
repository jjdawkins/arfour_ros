#!/bin/python3

import math
import rclpy
import rclpy.node
import threading
import time


from pymavlink import mavutil

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import Float32MultiArray
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

class arfourInterface(rclpy.node.Node):
    def __init__(self):
        super().__init__('drone_interface_node')

        self.max_roll = 24*math.pi/180
        self.max_pitch = 24*math.pi/180
        self.max_yawrate = 160*math.pi/180

        self.roll_cmd = 0.0
        self.pitch_cmd = 0.0
        self.yaw_cmd = 0.0
        self.thrust_cmd = 0.0

        self.roll_joy = 0.0
        self.pitch_joy = 0.0
        self.yaw_joy = 0.0
        self.thrust_joy = 0.0

        self.pos_err_z = 0.0
        self.vel_err_x = 0.0
        self.vel_err_y = 0.0
        self.vel_err_z = 0.0

        self.takeoff_land_spd = 0.5
        self.thrust_hover = 0.5
        self.takeoff_alt = 1.0
        self.Kvel_z = 0.1
        #self.Kpos_z = 

        self.flying = False        

        self.mode = IDLE

    #    self.declare_parameter('my_parameter', 'world')
        self.mav_conn = mavutil.mavlink_connection("/dev/ttyS0",baud=115200,input=False)

        self.imu_pub = self.create_publisher(Imu,'imu',10)

        self.set_sub = self.create_subscription(Setpoint,'setpoint',self.setpoint_callback,1)
        self.joy_sub = self.create_subscription(Joy,'joy',self.joy_callback,1)
        self.mocap_sub = self.create_subscription(Odometry,'/daisy/mocap/odom',self.mocap_callback,1)

        self.timer = self.create_timer(0.05, self.send_loop)

        self.read_thread = threading.Thread(target=self.read_loop,daemon=True)
        self.read_thread.start()
        self.init_time = self.get_clock().now()


    def mocap_callback(self,msg):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        self.pos_z = msg.pose.pose.position.z

        self.vel_x = msg.twist.twist.linear.x
        self.vel_y = msg.twist.twist.linear.y
        self.vel_z = msg.twist.twist.linear.z

        #print(self.vel_x,self.vel_y,self.vel_z)


    def takeoff(self):

        self.vel_err_x = 0 - self.vel_x
        self.vel_err_y = 0 - self.vel_y
        self.vel_err_z = self.takeoff_land_spd - self.vel_z

        self.pitch_cmd = self.Kv_x*(self.vel_err_x)
        self.roll_cmd = -(self.Kv_y*self.vel_err_y)
        self.yaw_cmd = 0

        self.thrust_cmd = self.Kvel_z*(self.vel_err_z) + self.thrust_hover
        
        time_from_boot = self.get_clock().now() - self.init_time
        time_ms = int(time_from_boot.nanoseconds*1e-6)
        self.mav_conn.mav.manual_setpoint_send(time_ms,self.roll_cmd,self.pitch_cmd,self.yaw_cmd,self.thrust_cmd, self.mode,0)

        if(self.vel_z > self.takeoff_alt):
            self.mode = HOVER


    def land(self):
        pass 

    def send_loop(self):

        time_from_boot = self.get_clock().now() - self.init_time

        time_ms = int(time_from_boot.nanoseconds*1e-6)
        print(self.mode)

        if(self.mode == TAKEOFF):
            self.takeoff()

        if(self.mode == LAND):
            self.land()
            
        if(self.mode == MANUAL):

            roll_cmd = self.roll_joy
            pitch_cmd = self.pitch_joy
            yaw_cmd = self.yaw_joy
            thrust_cmd = self.thrust_joy
            mode_cmd = self.mode
            #print(roll_cmd,pitch_cmd,yaw_cmd,thrust_cmd)
            self.mav_conn.mav.manual_setpoint_send(time_ms,roll_cmd,pitch_cmd,yaw_cmd,thrust_cmd, mode_cmd,0)
        
        if(self.mode == AUTO):
            roll_cmd = self.roll_cmd
            pitch_cmd = self.pitch_cmd
            yaw_cmd = self.yaw_cmd
            thrust_cmd = self.thrust_cmd
            mode_cmd = self.mode
            #print(roll_cmd,pitch_cmd,yaw_cmd,thrust_cmd)
            self.mav_conn.mav.manual_setpoint_send(time_ms,roll_cmd,pitch_cmd,yaw_cmd,thrust_cmd, mode_cmd,0)            

    def joy_callback(self,msg):

        self.roll_joy = (-msg.axes[3])*self.max_roll
        self.pitch_joy = msg.axes[4]*self.max_pitch
        self.yaw_joy = msg.axes[0]*self.max_yawrate
        self.thrust_joy = msg.axes[1]
        self.thrust_joy = saturate(self.thrust_joy,0.0,1.0)

        if(msg.buttons[0]):     # A Button
            pass

        if(msg.buttons[1]):     # B Button
            pass

        if(msg.buttons[2]):     # X Button
            if(self.flying):
                self.mode = LAND
            pass

        if(msg.buttons[3]):     # Y Button
            if(not self.flying):
                self.mode = TAKEOFF


        if(msg.buttons[4]):     # L Button
            if(self.flying):
                self.mode = AUTO
            pass

        if(msg.buttons[5]):     # R Button
            #if(self.flying):
            self.mode = MANUAL
                                        

        

    def setpoint_callback(self,msg):

        time_from_boot = self.get_clock().now() - self.init_time

        time_ms = int(time_from_boot.nanoseconds*1e-6)
        self.roll_cmd = msg.roll
        self.pitch_cmd = msg.pitch
        self.yaw_cmd = msg.yaw
        self.thrust_cmd = msg.thrust
        #mode_cmd = msg.mode
        
        #self.mav_conn.mav.manual_setpoint_send(time_ms,roll_cmd,pitch_cmd,yaw_cmd,thrust_cmd, mode_cmd,0)



    def read_loop(self):
        
        while(True):

            msg = self.mav_conn.recv_match(blocking=False)

            if(msg is None):
                pass
            elif(msg.get_type() == 'HEARTBEAT'):
                pass
            elif(msg.get_type() == 'SCALED_IMU'):
                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.linear_acceleration.x = msg.xacc*1e-3
                imu_msg.linear_acceleration.y = msg.yacc*1e-3
                imu_msg.linear_acceleration.z = msg.zacc*1e-3
                imu_msg.angular_velocity.x = msg.xgyro*1e-3
                imu_msg.angular_velocity.y = msg.ygyro*1e-3
                imu_msg.angular_velocity.z = msg.zgyro*1e-3

                self.imu_pub.publish(imu_msg)                                

            time.sleep(0.001)


def main():
    rclpy.init()
    node = arfourInterface()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
