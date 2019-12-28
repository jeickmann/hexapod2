from functools import reduce
import math

class Leg:
    def __init__(self):
        self.coxa_height = 0.03
        self.femur_length = 0.053
        self.tibia_length = 0.088

        self.coxa_angle = 0
        self.femur_angle = 0
        self.tibia_angle = -math.pi / 2

        self.coxa_offset = 0
        self.femur_offset = 0
        self.tibia_offset = 0

        self.coxa_dir = 1
        self.femur_dir = 1
        self.tibia_dir = 1

    def getCoxa(self):
        return self.coxa_angle*self.coxa_dir + self.coxa_offset

    def getFemur(self):
        return self.femur_angle*self.femur_dir + self.femur_offset

    def getTibia(self):
        return self.tibia_angle*self.tibia_dir + self.tibia_offset
        #h-1.5*-1-1.5

class Hardware:
    def __init__(self, serial):
        self.serial = serial
        self.legs = [Leg() for i in range(6)]
        self.legs[0].startServoIndex = 0
        self.legs[1].startServoIndex = 4
        self.legs[2].startServoIndex = 8
        self.legs[3].startServoIndex = 16
        self.legs[4].startServoIndex = 20
        self.legs[5].startServoIndex = 24
        
        self.max_pwm = 2500
        self.min_pwm = 500
    
    def updateJointCmd(self, servoIndex, angle):
        pwm = (angle + math.pi/2)/math.pi*(self.max_pwm - self.min_pwm)+self.min_pwm
        cmdStr = "#{} P{}".format(servoIndex, pwm)
        return cmdStr

    def updateLegCmd(self, leg):
        return self.updateJointCmd(leg.startServoIndex, leg.getCoxa()) +  " " + self.updateJointCmd(leg.startServoIndex+1, leg.getFemur()) +  " " + self.updateJointCmd(leg.startServoIndex+2, leg.getTibia())

    def update(self):
        cmd = ""
        for leg in self.legs:
            cmd = cmd + self.updateLegCmd(leg) + " "
        cmd = cmd + "\r\n"
        #print(cmd)
        self.serial.write(cmd.encode('utf-8'))
        self.serial.flush()