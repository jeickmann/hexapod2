import msvcrt

def dumpCalibration(hardware):
    print("Calibration Data")
    for i,leg in enumerate(hardware.legs):
        print("Leg {}: {:0.1f}, {:0.1f}, {:0.1f}".format(i, leg.coxa_angle, leg.femur_angle, leg.tibia_angle))

def dumpCalibrationData(hardware):
    result = "["
    for i,leg in enumerate(hardware.legs):
        result = result + "[{:0.1f},{:0.1f},{:0.1f}]".format(leg.coxa_angle, leg.femur_angle, leg.tibia_angle)
    result = result + "]"
    print(result)

def calibrate(hardware):
    print("Calibration")
    currentLeg = 0
    currentJoint = 0
    
    jointNames = ['coxa', 'femur', 'tibia']
    
    Break_KeyCheck = False
    
    while True:
        inc = 0
        base = msvcrt.getch()
        print(base)
        if base == b'\xe0':
            sub = msvcrt.getch()        
            if sub == b'H': #up
                currentLeg = (currentLeg + 1) % 6
                currentJoint = 0
            elif sub == b'M':
                currentJoint = (currentJoint + 1) % 3
            elif sub == b'P': #down
                currentLeg = (currentLeg - 1) % 6
                currentJoint = 0
            elif sub == b'K':
                currentJoint = (currentJoint - 1) % 3
        elif base == b'+':
            inc = 0.1
        elif base == b'-':
            inc = -0.1
        elif base == b'd':
            dumpCalibration(hardware)
            dumpCalibrationData(hardware)
        
        angle = 0
        leg = hardware.legs[currentLeg]
        if(currentJoint == 0):
            leg.coxa_angle = leg.coxa_angle + inc
            angle = leg.coxa_angle
        elif(currentJoint == 1):
            leg.femur_angle = leg.femur_angle + inc
            angle = leg.femur_angle
        else:
            leg.tibia_angle = leg.tibia_angle + inc
            angle = leg.tibia_angle
    
        if(inc != 0):
            hardware.update()
    
        print ("Leg {}, joint {} at {:.2f} radians".format(currentLeg, jointNames[currentJoint], angle))
