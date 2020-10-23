import time
import math
import numpy as np
from dynio import *

class RobotArm:

    def __init__(self):
        self.motors = [] # Motors are ordered from bottem to top
        self.home_angle = [] # Motors are ordered from bottem to top
        self.robot_arm_dxl = None
        # Use the following for OSX/Linux systems to select serial ports and use COMM ports for windows
        self.robot_arm_dxl = dxl.DynamixelIO("/dev/ttyUSB0")
        # These are constants
        self.BASE_HEIGHT = 4.3897638
        self.UPPER_ARM = 5.6929134
        self.FOREARM = 5.6889764
        self.WRIST = 5.4015748
        # Motor IDs must start from 1
        for i in range(1,9):
                self.motors.append(self.robot_arm_dxl.new_ax12(i))
                self.motors[-1].torque_enable()
                self.motors[-1].set_position_mode(goal_current=980) #Sets the motors to position mode and sets there max torque
        self.calibrate()


    def __del__(self):
        self.home()

        # Matrix for entire arm (I can not explain the math you will have to go learn it)
        self.H0_3 = np.dot(np.dot(H0_1,H1_2),H2_3)
        print(np.matrix(self.H0_3))

    # Gets the home position of the motors (Full Retracted)
    def calibrate(self,show_pos=False):
        for motor in self.motors:
            if show_pos:
                print(motor.get_position())
            self.home_angle.append(motor.get_angle())

    # Sets each motor to there home position
    def home(self):
        for i in reversed(range(0,len(self.motors))):
            self.motors[i].set_angle(self.home_angle[i])
            time.sleep(.01)

    # Sets the base motor
    def set_base(self,angle,show_pos=False):
        self.motors[0].set_angle(angle)

    # Sets the angles for the shoulder motors
    def set_shoulder(self,angleRef,show_pos=False):
        if angle < 58 or angle > 240:
            print("Out of range shoulder")
        else:
            self.motors[1].set_angle(angleRef + 59)
            self.motors[2].set_angle(self.home_angle[2]-angleRef) # Motor 3 is about 180 degrees from motor 2

    # Sets the motor angles for the elbow motors
    def set_elbow(self,angleRef,show_pos=False):
        if angle < 58 or angle > 264:
            print("Out of range elbow")
        else:
            self.motors[3].set_angle(angleRef + 60)
            self.motors[4].set_angle(self.home_angle[4]-angleRef)

    # Sets Wrist veritcal motor
    def set_wrist_vertical(self,angleRef,show_pos=False,actual=False,radians=False):
        if radians:
            angle = np.degrees(angle)
        if not actual:
            angle = angleRef + 57
        if angle < 54 or angle > 244:
            print("Out of range wrist")
        else:
            self.motors[5].set_angle(angle)

    # Sets wrist rotation
    def set_wrist_angle(self,angle,show_pos=False):
        self.motors[6].set_angle(angle)

    # Claw open and close
    def hand(self,state="",angleRef=0):
        if state != "":
            if state == "open":
                self.motors[-1].set_angle(150)
            if state == "close":
                self.motors[-1].set_angle(0)
        else:
            self.motor[-1].set_angle(angleRef)

    # Fully extends the arm
    def full_extend(self):
        self.set_elbow(240)
        time.sleep(.15)
        self.set_shoulder(150)
        self.set_wrist_vertical(150)
