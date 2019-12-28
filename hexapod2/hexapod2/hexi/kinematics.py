from vector3d.point import Point

import math

class KinematicLeg:
    def __init__(self, hardwareLeg):
        self.x = self.y = self.z = 0
        self.hardwareLeg = hardwareLeg
        
    def setPosition(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.updateHardwareLeg()

    def setHardwareLeg(hardwareLeg):
        self.hardwareLeg = hardwareLeg

    def updateHardwareLeg(self):
        if(self.hardwareLeg):
            z = -self.z
            self.hardwareLeg.coxa_angle = -math.atan2(self.x, self.y)
        
            G = math.sqrt(self.x*self.x + self.y*self.y)
            L = math.sqrt(z*z + G*G)
        
            a1 = math.acos(z/L)

            a2 = math.acos((self.hardwareLeg.tibia_length*self.hardwareLeg.tibia_length-self.hardwareLeg.femur_length*self.hardwareLeg.femur_length-L*L)/(-2*self.hardwareLeg.femur_length*L))

            self.hardwareLeg.femur_angle = a1+a2-math.pi/2

            self.hardwareLeg.tibia_angle = -math.pi+math.acos((L*L-self.hardwareLeg.tibia_length*self.hardwareLeg.tibia_length-self.hardwareLeg.femur_length*self.hardwareLeg.femur_length)/(-2*self.hardwareLeg.tibia_length*self.hardwareLeg.femur_length))

            #print("For pos %f %f %f setting angles to %f %f %f", self.x, self.y, self.z, self.hardwareLeg.coxa_angle, self.hardwareLeg.femur_angle, self.hardwareLeg.tibia_angle)