#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.duration import Duration

from math import sin, cos
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Vector3, TransformStamped
from tf2_ros import TransformBroadcaster#, TransformStamped
from sensor_msgs.msg import JointState, Imu


from Mpu6050 import Mpu6050
from shared_param.common_param import WHEEL_PORTS
from encoder2 import Encoder2

class OdometryPublisher(Node):

    def __init__(self):
        super().__init__('bot_state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        loop_rate = self.create_rate(30) #20Hz???

        # message declarations
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'base_link' #'base_link'

        odom = Odometry()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link' #'base_link'

        # joint state declarations
        joint_state = JointState()
        joint_state.header.frame_id = 'joint_states'
        wheel_names= ['drivewhl_fl_joint','drivewhl_fr_joint', \
            'drivewhl_l_joint', 'drivewhl_r_joint']
        joint_state.name = wheel_names

        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', qos_profile)

        # IMU declarations
        imu_trans = Imu()
        imu_trans.header.frame_id = 'imu_link'
        self.imu_pub = self.create_publisher(Imu, 'imu', qos_profile)
        mpu6050 = Mpu6050()

        #Encoder declarations
        try:
            en_all = Encoder2(WHEEL_PORTS)
        except:
            time.sleep(3)
            en_all = Encoder2(WHEEL_PORTS)

        x = 0.0
        y = 0.0
        th = 0.0
        vx = 0.0
        vy = 0.0
        vth = 0.0

        current_time = 0
        last_time = Duration(seconds=0, nanoseconds=0)
        try:
            while rclpy.ok():
                rclpy.spin_once(self)
                #update time
                current_time = self.get_clock().now()


                
                # compute odometry in a typical way given velocities
                dt = float((current_time - last_time).nanoseconds) / (10 ** 9) # to sec
                #get velocities
                vx, vy, vth = en_all.cal_r_velocity(dt * 1000) # convert back to miliseconds
                ## debug
                ##print("x: {} y: {} z{}".format(vx, vy, vth), end='\r')
                delta_x = (vx * cos(th) - vy * sin(th)) * dt
                delta_y = (vx * sin(th) + vy * cos(th)) * dt
                delta_th = vth * dt

                x += delta_x
                y += delta_y
                th += delta_th

                #update odom
                odom.header.stamp = current_time.to_msg()
                odom.pose.pose.position.x = x
                odom.pose.pose.position.y = y
                odom.pose.pose.position.z = 0.0

                odom.pose.pose.orientation = \
                    euler_to_quaternion(0, 0, th)
                
                odom.twist.twist.linear.x = vx
                odom.twist.twist.linear.y = vy
                odom.twist.twist.angular.z = vth

                self.odom_pub.publish(odom)
                
                #update transform
                odom_trans.header.stamp = current_time.to_msg()
                odom_trans.transform.translation.x = x        
                odom_trans.transform.translation.y = y
                odom_trans.transform.translation.z = 0.0

                odom_trans.transform.rotation = \
                    euler_to_quaternion(0, 0, th)



                self.broadcaster.sendTransform(odom_trans)

                joint_state.header.stamp = current_time.to_msg()
                joint_state.position = \
                    en_all.get_encoder_rad()

                self.joint_state_pub.publish(joint_state)

                #update imu
                imu_trans.header.stamp = current_time.to_msg()
                imu_trans.orientation = euler_to_quaternion(0, 0, th)
                gyro = mpu6050.get_gyro_rad()
                acc = mpu6050.get_acc()
                imu_trans.angular_velocity = Vector3(x=gyro[0], y=gyro[1], z=gyro[2])
                imu_trans.linear_acceleration = Vector3(x=acc[0], y=acc[1], z=acc[2])
                self.imu_pub.publish(imu_trans)
                

                #try updating lasttime 
                last_time = current_time
                # This will adjust as needed per iteration
                loop_rate.sleep()

        except KeyboardInterrupt:
            self.destroy_node()
            rclpy.shutdown()

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main(args = None):

    rclpy.init(args= args)
    OdometryPublisher()

if __name__ == '__main__':
    main()