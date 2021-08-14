#!/usr/bin/env python3
import rclpy
import threading
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from motor_drive import Motor_Init
from shared_param.common_param import WHEELBASE_, TRACK_, WHEELRADIUS_, BOT_MAX_SPEED
#import RPi.GPIO as GPIO
#from gpioconfig import WHEEL_PORTS

motors = Motor_Init()
# Place holder for shared speeds
##vx_o = 0.0
##vy_o = 0.0
##wz_o = 0.0
##timestamp = 0
##i_w1 = 0.0
##i_w2 = 0.0
##i_w3 = 0.0
##i_w4 = 0.0

class Bot_control(Node):
    def __init__(self):
        super().__init__('bot_control')

        qos_profile = QoSProfile(depth=10)
        self.cmd_vel_sub_ = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            qos_profile)
        ##motors
        ##self.wheel_vel_sub = self.create_subscription(
        ##    Odometry,
        ##    'odom',
        ##    self.current_speed_callback,
        ##    qos_profile)
        

    ##def current_speed_callback(self, msg):
    ##    # Update wheel speed
    ##    global vx_o, vy_o, wz_o, timestamp

    ##    timestamp = msg.header.stamp.nanosec
    ##    vx_o =  msg.twist.twist.linear.x
    ##    vy_o =  msg.twist.twist.linear.y
    ##    wz_o =  msg.twist.twist.angular.z
    ##    #self.get_logger().info("I heard {}".format(msg))


    def cmd_vel_callback(self, msg):
        # Store velocity from teleop
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        ##global vx_o, vy_o, wz_o, timestamp
        ##global i_w1, i_w2, i_w3, i_w4
        ##dt = float(self.get_clock().now().to_msg().nanosec - timestamp) / (10 ** 9)

        w1, w2, w3, w4 = mecanum_IK(vx, vy, wz)
        ##cw1, cw2, cw3, cw4 = mecanum_IK(vx, vy, wz)
        ##w1_o, w2_o, w3_o, w4_o = mecanum_IK(vx_o, vy_o, wz_o)

        ##w1, i_w1 = pid_control(cw1, w1_o, dt, i_w1)
        ##w2, i_w2 = pid_control(cw2, w2_o, dt, i_w2)
        ##w3, i_w3 = pid_control(cw3, w3_o, dt, i_w3)
        ##w4, i_w4 = pid_control(cw4, w4_o, dt, i_w4)

        motors[0].simple_linear_speed_control(w1)
        motors[1].simple_linear_speed_control(w2)
        motors[2].simple_linear_speed_control(w3)
        motors[3].simple_linear_speed_control(w4)

        #self.get_logger().info('"%s" "%s" "%s"  \r' % (vx, vy, wz))

def mecanum_IK(vx, vy, wz, \
    wheelbase = WHEELBASE_, track = TRACK_, wheel_radius = WHEELRADIUS_):
    # calculate wheel speed (rps) from given speeds (m/s, rps)
    lxly = (track + wheelbase) / 2
    fwradius = 1/wheel_radius
    w1 = fwradius * (vx - vy - lxly * wz)
    w2 = fwradius * (vx + vy + lxly * wz)
    w3 = fwradius * (vx + vy - lxly * wz)
    w4 = fwradius * (vx - vy + lxly * wz)

    return w1, w2, w3, w4

def simple_p_control(ctrl, feedback, Kp = 1.85):
    # Pout = Kp * e + R, from wikipedia.
    ## not working
    return Kp * (ctrl - feedback) + ctrl
#TODO: Refactor P control to work. using speed

##def pid_control(r, f, dt, integral, \
##    Kp = 1.22, Kd = 0.022, Ki = 1.45): #Ziegler-Nichols ,u =6, T =.05
##    # from wiki psuedo code 
##    # integral clamping
##    e = r - f
##    derivative = e / dt
##    integral = integral + e * dt
##    return Kp * e + Kd * derivative + Ki * integral, integral 

def main(args=None):
    rclpy.init(args=args)
    cmd_vel_subscriber = Bot_control()
    rate = cmd_vel_subscriber.create_rate(2) # update over 2Hz
    # Spin in a seperate thread
    thread = threading.Thread(target=rclpy.spin, args=(cmd_vel_subscriber, ), daemon=True)
    thread.start()
    try:
        while rclpy.ok():
            #rclpy.spin_some(cmd_vel_subscriber)
            cmd_vel_subscriber
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        cmd_vel_subscriber.destroy_node()
        rclpy.shutdown()
        motors[0].shutdown()
        thread.join()

if __name__ == '__main__':
    main()