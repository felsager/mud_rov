#!/usr/bin/env python3

''' ROS module '''
import rospy

''' Math modules '''
import numpy as np

''' Timing module '''
import time

''' I2C modules '''
import board 
import busio

''' PWM driver module'''
from adafruit_servokit import ServoKit

''' IMU modules '''
from robohat_mpu9250.mpu9250 import MPU9250
from robohat_mpu9250.mpu6500 import MPU6500
from robohat_mpu9250.ak8963 import AK8963

''' ROS messages '''
from sensor_msgs.msg import Joy


class ControllerNode:

    def __init__(self):
        rospy.init_node('controller_node', anonymous=True)
        
        self.t_control = 0.02 # control period [s] - limited by esc 50Hz - delta t
        self.rate_control = int(1/self.t_control)
        
        self.pwm_driver = self.init_pwm_driver()
        self.imu = self.init_imu()
        self.init_esc()

        ''' Desired state '''
        self.pos_x_des = 0 # [m/s] forward velocity - left stick vertical
        self.yaw_des = 0 # [rad] yaw angle - left stick horizontal
        self.pos_z_des = 0 # [m/s] vertical velocity - triggers
        self.roll_des = 0 # [rad] roll angle - right stick horizontal
        self.pitch_des = 0 # [rad] pitch angle - right stick vertical

        self.joy_state = Joy()
        self.joy_state.axes = [0, 0, 0, 0, 0, 0, 0, 0]
        self.joy_state.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.joystick_sub = rospy.Subscriber('joy', Joy, callback=self.joystick_callback, queue_size=10)

        ''' Done last to ensure all other initializations are done'''
        print('Rate control = ', self.rate_control)
        self.control_timer = rospy.Timer(rospy.Duration(self.t_control), self.control_callback)

    def init_pwm_driver(self):
        min_pw = 1000 # minimum pulsewidth [us]
        max_pw = 2000 # maximum pulsewidth [us]
        no_chnls = 16 # number of pwm channels
        i2c_0 = busio.I2C(board.SCL7, board.SDA7)
        pwm_driver = ServoKit(channels=no_chnls, i2c=i2c_0)
        for i in range(no_chnls):
            pwm_driver.continuous_servo[i].set_pulse_width_range(min_pw, max_pw)    
        return pwm_driver
    
    def init_esc(self):
        rate = rospy.Rate(self.rate_control)
        N = int(5/self.t_control) # number of iterations to initialize esc (5s)
        for i in range(N):
            self.set_thrusters([0, 0, 0, 0, 0, 0])
            rate.sleep()

    def set_thrusters(self, thrust_inputs):
        for t in range(len(thrust_inputs)):
            self.pwm_driver.continuous_servo[t].throttle = thrust_inputs[t]

    def init_imu(self):
        i2c_1 = busio.I2C(board.SCL2, board.SDA2)
        mpu = MPU6500(i2c_1, busnum=1,  address=0x68,
            gyro_offset=(3.732, -1.407, -3.589))
        ak = AK8963(i2c_1, 
            offset=(12.210, 12.825, -23.183),
            scale=(0.966, 1.132, 0.924))
        return MPU9250(mpu, ak)
    
    def control_callback(self, event):
        self.update_desired_state(self.t_control, self.joy_state)
        thrust_inputs = [0, 0, 0, 0, 0, 0] # TODO
        self.set_thrusters(thrust_inputs)
        #print("Test")

    def joystick_callback(self, data):
        self.joy_state = data
        if data.buttons[2] == 1:
            self.killswitch()

    def update_desired_state(self, dt, joy_state):
        self.pos_x_des += dt*joy_state.axes[1] # maybe need to flip sign - left_stick_vert
        self.pos_z_des += dt*(joy_state.axes[2] - joy_state.axes[5])/2 # left_trigger - right_trigger
        self.pitch_des += dt*joy_state.axes[4] # right_stick_vert
        self.roll_des += dt*joy_state.axes[3] # right_stick_horz
        self.yaw_des += dt*joy_state.axes[0] # left_stick_horz
        print("pitch_des = ", self.pitch_des)
        self.pitch_des = self.wrap_angle(self.pitch_des)
        self.roll_des = self.wrap_angle(self.roll_des)
        self.yaw_des = self.wrap_angle(self.yaw_des)
        print("pitch_des_sat = ", self.pitch_des)
        # TODO : add scaling and saturation to desired state

    def wrap_angle(self, angle): # wrap angle to the range [-1, 1]
        return np.mod(angle + 1, 2) - 1
    
    def killswitch(self):
        self.init_esc()
        rospy.signal_shutdown("Killed")

    # TODO - add function to calculate error between desired and actual state
    # TODO - add function to calculate thrust inputs from error - remember to saturate and scale according to thruster locations
    # TODO - add function to estimate actual state from imu data

def main():
    try: 
        controller_node = ControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("error!")

if __name__ == "__main__":
    main()
