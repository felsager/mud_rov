#!/usr/bin/env python3

''' ROS module '''
import rospy

''' Math modules '''
import numpy as np
import cvxpy as cx

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
        
        self.T_alloc = np.array([
            [0, 0, 0, 0, 0.5, 0.5],  # F_x - scale by 2 to get max thrust
            [0.25, 0.25, 0.25, 0.25, 0, 0],  # F_z - scale by 4 to get max thrust
            [-0.217, 0.217, -0.217, 0.217, 0, 0],  # M_roll
            [-0.152, -0.152, 0.152, 0.152, 0, 0],  # M_pitch
            [0, 0, 0, 0, -0.185, 0.185]  # M_yaw - only controlled by two horizontal thrusters
        ])
        self.T_alloc_pinv = np.linalg.pinv(self.T_alloc) # inverse of thruster allocation matrix

        self.pwm_driver = self.init_pwm_driver()
        self.imu = self.init_imu()
        self.init_esc()

        ''' Desired state '''
        self.F_x = 0 # [m/s] forward velocity - left stick vertical
        self.F_z = 0 # [m/s] vertical velocity - triggers
        self.tau_r = 0 # [rad/s] roll rate - right stick horizontal
        self.tau_p = 0 # [rad/s] pitch rate - right stick vertical
        self.tau_y = 0 # [rad/s] yaw rate - left stick horizontal

        self.pre_thrust_inputs = np.zeros(6) # previous thruster values

        self.joy_state = Joy()
        self.joy_state.axes = [0, 0, 0, 0, 0, 0, 0, 0]
        self.joy_state.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.joystick_sub = rospy.Subscriber('joy', Joy, callback=self.joystick_callback, queue_size=10)

        ''' Done last to ensure all other initializations are done'''
        print('Rate control = ', self.rate_control)
        self.control_loop = rospy.Timer(rospy.Duration(self.t_control), self.control_callback)

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
        thrust_inputs = np.clip(thrust_inputs, -1, 1)
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
        self.update_desired_state(self.joy_state)
        thrust_inputs = self.thrust_allocation(self.F_x, 
            self.F_z, self.tau_r, self.tau_p, self.tau_y)
        thrust_inputs = self.actuator_filter(thrust_inputs, 
                                             self.pre_thrust_inputs)
        self.set_thrusters(thrust_inputs)
        self.pre_thrust_inputs = thrust_inputs
        print(f'thrust inputs = {np.round(thrust_inputs, 3)}')

    def thrust_allocation(self, F_x, F_z, M_roll, M_pitch, M_yaw):
        ''' Thrust allocation '''
        desired_force_torque = np.array([F_x, F_z, M_roll, M_pitch, M_yaw])
        initial_thruster_values = self.T_alloc_pinv @ desired_force_torque
        thruster_values = cx.Variable(6) # 6 thrusters
        objective = cx.Minimize(cx.norm(self.T_alloc @ thruster_values - desired_force_torque, "inf"))
        constraints = [thruster_values >= -1, thruster_values <= 1]
        problem = cx.Problem(objective, constraints)
        problem.solve()
        thrust_inputs = thruster_values.value
        return thrust_inputs
    
    def actuator_filter(self, thrust_inputs, pre_thrust_inputs):
        ''' Actuator filter '''
        alpha = 0.986 # time constant of ~ 1.4s
        filtered_thrust_inputs = (1 - alpha)*thrust_inputs + alpha*pre_thrust_inputs
        for i in range(len(filtered_thrust_inputs)): # deadband
            if abs(thrust_inputs[i]) < abs(pre_thrust_inputs[i]) and filtered_thrust_inputs < 0.05:
                filtered_thrust_inputs[i] = 0
        return filtered_thrust_inputs

    def joystick_callback(self, data):
        self.joy_state = data
        if data.buttons[2] == 1:
            self.killswitch()

    def update_desired_state(self, joy_state):
        self.F_x = joy_state.axes[1] # maybe need to flip sign - left_stick_vert
        self.F_z = (joy_state.axes[2] - joy_state.axes[5])/2 # left_trigger - right_trigger
        self.tau_r = joy_state.axes[3] # right_stick_horz - roll rate
        self.tau_p = joy_state.axes[4] # right_stick_vert - pitch rate
        self.tau_y = joy_state.axes[0] # left_stick_horz - yaw rate
    
    def killswitch(self):
        self.init_esc()
        rospy.signal_shutdown("Killed")

 
def main():
    try: 
        controller_node = ControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("error!")

if __name__ == "__main__":
    main()
