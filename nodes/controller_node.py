#!/usr/bin/env python3

''' Ros module '''
import rospy

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


class ControllerNode:

    def __init__(self):
        rospy.init_node('controller_node', anonymous=True)
        
        self.pwm_driver = self.init_pwmdriver()
        self.imu = self.init_imu()

        ''' Done last to ensure all other initializations are done'''
        self.t_control = 0.02 # control period [s] - limited by esc 50Hz
        self.control_timer = rospy.Timer(rospy.Duration(self.t_control), self.control_callback)
        

    def init_pwmdriver(self):
        min_pw = 1000 # minimum pulsewidth [us]
        max_pw = 2000 # maximum pulsewidth [us]
        no_chnls = 16 # number of pwm channels
        i2c_0 = busio.I2C(board.SCL7, board.SDA7)
        pwm_driver = ServoKit(channels=no_chnls, i2c=i2c_0)
        for i in range(no_chnls):
            pwm_driver.continuous_servo[i].set_pulse_width_range(min_pw, max_pw)    
        return pwm_driver

    def set_thrusters(self, thrust_values):
        for t in range(len(thrust_values)):
            self.pwm_driver.continuous_servo[t].throttle = thrust_values[t]

    def init_imu(self):
        i2c_1 = busio.I2C(board.SCL2, board.SDA2)
        mpu = MPU6500(i2c_1, busnum=1,  address=0x68,
            gyro_offset=(3.732, -1.407, -3.589))
        ak = AK8963(i2c_1, 
            offset=(12.210, 12.825, -23.183),
            scale=(0.966, 1.132, 0.924))
        return MPU9250(mpu, ak)
    
    def control_callback(self, event):
        thrust_values = [0, 0, 0, 0, 0, 0] # TODO
        self.set_thrusters(thrust_values)
        print("Test")

def main():
    try: 
        controller_node = ControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("error!")

if __name__ == "__main__":
    main()
