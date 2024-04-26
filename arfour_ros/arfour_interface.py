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
        self.max_spd_xy = 1.3
        self.max_spd_z = 1.0
        self.max_thrust_i = 0.1

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
        self.thrust_hover = 0.58 #0.55
        self.thrust_idle = 0.2
        self.takeoff_alt = 1.0
        self.Kyaw = 0.5
        self.Kpos_xy = 0.3
        self.Kpos_z = 0.1
        self.Kpos_z_d = 0.05
        self.Kpos_z_i = 0.01
        self.Kvel_x = 0.3
        self.Kvel_y = 0.3
        self.Kvel_z = 0.025
        self.Kvel_i_z = 0.02
        self.dt = 0.05


        self.flying = False        

        self.mode = IDLE
        self.prev_mode = IDLE
        self.mode_pos = [0.0,0.0,0.0]
    #    self.declare_parameter('my_parameter', 'world')
        self.mav_conn = mavutil.mavlink_connection("/dev/ttyS0",baud=115200,input=False)

        self.imu_pub = self.create_publisher(Imu,'imu',10)

        self.set_sub = self.create_subscription(Setpoint,'setpoint',self.setpoint_callback,1)
        self.joy_sub = self.create_subscription(Joy,'joy',self.joy_callback,1)
        self.mocap_sub = self.create_subscription(Odometry,'/daisy/mocap/odom',self.mocap_callback,1)
        self.eul_sub = self.create_subscription(Vector3Stamped,'/daisy/eul',self.euler_callback,1)
        self.vel_sub = self.create_subscription(Vector3Stamped,'/daisy/vel',self.velocity_callback,1)

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

    def takeoff(self):

        self.des_vel_z = 1.5*(self.takeoff_alt - self.pos_z)
        self.vel_err_x = 0 - self.vel_x
        self.vel_err_y = 0 - self.vel_y
        self.vel_err_z = self.des_vel_z - self.vel_z


        if(self.flying):
            self.vel_err_z_int += self.vel_err_z*self.dt
        else:
            self.vel_err_z_int = 0.0

        self.pitch_cmd = self.Kvel_x*(self.vel_err_x)
        self.roll_cmd = -(self.Kvel_y*self.vel_err_y)
        self.yaw_cmd = 0

        self.thrust_cmd = self.Kvel_z*(self.vel_err_z) + self.Kvel_i_z*(self.vel_err_z_int) + self.thrust_hover
        
        time_from_boot = self.get_clock().now() - self.init_time
        time_ms = int(time_from_boot.nanoseconds*1e-6)
        self.mav_conn.mav.manual_setpoint_send(time_ms,self.roll_cmd,self.pitch_cmd,self.yaw_cmd,self.thrust_cmd, self.mode,0)

        # if(self.pos_z > self.takeoff_alt):
        #     self.mode = HOVER


    def land(self):
        self.vel_err_x = 0.0 - self.vel_x
        self.vel_err_y = 0.0 - self.vel_y
        self.vel_err_z = (-0.3*self.takeoff_land_spd) - self.vel_z

        if(self.flying):
            self.vel_err_z_int += self.vel_err_z*self.dt
        else:
            self.vel_err_z_int = 0.0

        self.pitch_cmd = self.Kvel_x*(self.vel_err_x)
        self.roll_cmd = -(self.Kvel_y*self.vel_err_y)
        self.yaw_cmd = 0.0

        self.thrust_cmd = self.Kvel_z*(self.vel_err_z) + self.Kvel_i_z*(self.vel_err_z_int) + self.thrust_hover
        
        time_from_boot = self.get_clock().now() - self.init_time
        time_ms = int(time_from_boot.nanoseconds*1e-6)
        self.mav_conn.mav.manual_setpoint_send(time_ms,self.roll_cmd,self.pitch_cmd,self.yaw_cmd,self.thrust_cmd, self.mode,0)

        if(self.pos_z < 0.1):
            self.mode = READY     

    def idle(self):
        time_from_boot = self.get_clock().now() - self.init_time
        time_ms = int(time_from_boot.nanoseconds*1e-6)
        self.mav_conn.mav.manual_setpoint_send(time_ms,0.0,0.0,0.0,0.0,0,0)

    def ready(self):
        time_from_boot = self.get_clock().now() - self.init_time
        time_ms = int(time_from_boot.nanoseconds*1e-6)
        self.mav_conn.mav.manual_setpoint_send(time_ms,0.0,0.0,0.0,self.thrust_idle, self.mode,0)

    def manual(self):
        des_vel_y = -self.roll_joy*self.max_spd_xy
        des_vel_x = self.pitch_joy*self.max_spd_xy
        des_vel_z = self.thrust_joy*self.max_spd_z
        self.yaw_cmd = self.yaw_joy*self.max_yawrate

        self.vel_err_x = des_vel_x - self.vel_x
        self.vel_err_y = des_vel_y - self.vel_y
        self.vel_err_z = des_vel_z - self.vel_zg


        if(self.flying):
            self.vel_err_z_int += self.vel_err_z*self.dt
        else:
            self.vel_err_z_int = 0.0
        
        # Only integrate if integral term is less than some max to avoid windup
       # if(abs(self.vel_err_z_int*self.Kvel_i_z) > self.max_thrust_i):
       #     self.vel_err_z_int = self.vel_err_z_int            
           

        self.pitch_cmd = self.Kvel_x*(self.vel_err_x)
        self.roll_cmd = -(self.Kvel_y*self.vel_err_y)        
        self.thrust_cmd = self.Kvel_z*(self.vel_err_z) + self.Kvel_i_z*(self.vel_err_z_int) + self.thrust_hover


        #print(des_vel_z,self.vel_z,self.thrust_cmd)
        time_from_boot = self.get_clock().now() - self.init_time
        time_ms = int(time_from_boot.nanoseconds*1e-6)
        self.mav_conn.mav.manual_setpoint_send(time_ms,self.roll_cmd,self.pitch_cmd,self.yaw_cmd,self.thrust_cmd, self.mode,0)
    
    def hover(self):
        self.des_yaw = 0.0
        self.pos_err_x = self.mode_pos[0] - self.pos_x
        self.pos_err_y = self.mode_pos[1] - self.pos_y
        self.pos_err_z = self.mode_pos[2] - self.pos_z
        #self.pos_err_z = self.takeoff_alt - self.pos_z  

        
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

        time_from_boot = self.get_clock().now() - self.init_time
        time_ms = int(time_from_boot.nanoseconds*1e-6)
        self.mav_conn.mav.manual_setpoint_send(time_ms,self.roll_cmd,self.pitch_cmd,self.yaw_cmd,self.thrust_cmd, self.mode,0)
    
    # def velocity_controller(self,des_vel_g,des_psi):


    # def position_controller(self,des_pos,des_psi):



    def send_loop(self):

        time_from_boot = self.get_clock().now() - self.init_time

        time_ms = int(time_from_boot.nanoseconds*1e-6)
        
        if(self.mode == IDLE):
            self.idle()

        if(self.mode == READY):
            self.ready()
            
        if(self.mode == HOVER):
            self.hover()

        if(self.mode == TAKEOFF):
            self.takeoff()
            if(self.pos_z > self.takeoff_alt):
                self.mode = HOVER             

        if(self.mode == LAND):
            self.land()
            
        if(self.mode == MANUAL):
            self.manual()
 
        if(self.mode == AUTO):
            roll_cmd = self.roll_cmd
            pitch_cmd = self.pitch_cmd
            yaw_cmd = self.yaw_cmd
            thrust_cmd = self.thrust_cmd
            mode_cmd = self.mode
            print(roll_cmd,pitch_cmd,yaw_cmd,thrust_cmd)
            self.mav_conn.mav.manual_setpoint_send(time_ms,roll_cmd,pitch_cmd,yaw_cmd,thrust_cmd, mode_cmd,0)            
        
        if(self.mode != self.prev_mode):
            self.mode_pos[0] = self.pos_x
            self.mode_pos[1] = self.pos_y
            self.mode_pos[2] = self.pos_z
            print(self.mode,self.mode_pos) 

        self.prev_mode = self.mode

    def joy_callback(self,msg):

        self.roll_joy = (-msg.axes[3])*self.max_roll
        self.pitch_joy = msg.axes[4]*self.max_pitch
        self.yaw_joy = msg.axes[0]*self.max_yawrate
        self.thrust_joy = msg.axes[1]
        #self.thrust_joy = saturate(self.thrust_joy,0.0,1.0)

        if(msg.buttons[0]):     # A Button
            if(self.flying):
                self.mode = LAND            

        if(msg.buttons[1]):     # B Button
            pass

        if(msg.buttons[2]):     # X Button
            pass

        if(msg.buttons[3]):     # Y Button
            if(not self.flying):
                self.mode = TAKEOFF


        if(msg.buttons[4]):     # L Button
            if(self.flying):
                self.mode = AUTO

        if(msg.buttons[5]):     # R Button
            #if(self.flying):
            self.mode = MANUAL

        if(msg.buttons[7]):     # Start Button
            self.mode = READY                                        

        

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
