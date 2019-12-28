from vector3d.point import Point

import math

class KinematicLeg:
    def __init__(self, hardwareLeg):
        self.x = self.y = self.z = 0

        self.coxa_height = 0.03
        self.femur_length = 0.053
        self.tibia_length = 0.088

        self.hardwareLeg = hardwareLeg
        
    def setPosition(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.updateHardwareLeg()

    def calculatePositions(self, x,y,z):
        z = -z
        coxa_angle = -math.atan2(self.x, self.y)
    
        G = math.sqrt(self.x*self.x + self.y*self.y)
        L = math.sqrt(z*z + G*G)
    
        a1 = math.acos(z/L)

        a2 = math.acos((self.tibia_length*self.tibia_length-self.femur_length*self.femur_length-L*L)/(-2*self.femur_length*L))

        femur_angle = a1+a2-math.pi/2

        tibia_angle = -math.pi+math.acos((L*L-self.tibia_length*self.tibia_length-self.femur_length*self.femur_length)/(-2*self.tibia_length*self.femur_length))

        return coxa_angle, femur_angle, tibia_angle

    def updateHardwareLeg(self):
        if(self.hardwareLeg):
            self.hardwareLeg.coxa_angle, self.hardwareLeg.femur_angle, self.hardwareLeg.tibia_angle = self.calculatePositions(self.x, self.y, self.z)