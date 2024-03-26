#!/bin/python3

import math
import rclpy
import rclpy.node
import threading
import time
from pymavlink import mavutil

from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import Float32MultiArray
from arfour_msgs.msg import Setpoint

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

    #    self.declare_parameter('my_parameter', 'world')
        self.mav_conn = mavutil.mavlink_connection("/dev/ttyUSB0",baud=115200,input=False)

        self.imu_pub = self.create_publisher(Imu,'imu',10)

        self.set_sub = self.create_subscription(Setpoint,'setpoint',self.setpoint_callback,1)
        self.joy_sub = self.create_subscription(Joy,'joy',self.joy_callback,1)

        self.timer = self.create_timer(0.05, self.send_loop)

        self.read_thread = threading.Thread(target=self.read_loop,daemon=True)
        self.read_thread.start()
        self.init_time = self.get_clock().now()

        

    def send_loop(self):

        time_from_boot = self.get_clock().now() - self.init_time

        time_ms = int(time_from_boot.nanoseconds*1e-6)
        roll_cmd = self.roll_cmd
        pitch_cmd = self.pitch_cmd
        yaw_cmd = self.yaw_cmd
        thrust_cmd = self.thrust_cmd
        mode_cmd = 0
        print(roll_cmd,pitch_cmd,yaw_cmd,thrust_cmd)

        self.mav_conn.mav.manual_setpoint_send(time_ms,roll_cmd,pitch_cmd,yaw_cmd,thrust_cmd, mode_cmd,0)
        
     

    def joy_callback(self,msg):

        self.roll_cmd = (-msg.axes[3])*self.max_roll
        self.pitch_cmd = msg.axes[4]*self.max_pitch
        self.yaw_cmd = msg.axes[0]*self.max_yawrate
        self.thrust_cmd = msg.axes[1]
        self.thrust_cmd = saturate(self.thrust_cmd,0.0,1.0)
        

    def setpoint_callback(self,msg):

        time_from_boot = self.get_clock().now() - self.init_time

        time_ms = int(time_from_boot.nanoseconds*1e-6)
        roll_cmd = msg.roll
        pitch_cmd = msg.pitch
        yaw_cmd = msg.yaw
        thrust_cmd = msg.thrust
        mode_cmd = msg.mode
        
        self.mav_conn.mav.manual_setpoint_send(time_ms,roll_cmd,pitch_cmd,yaw_cmd,thrust_cmd, mode_cmd,0)



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
