# This file is for actually executing the instruction that needs to be run on the rover. 


class CompleteInstruction:
    def __init__(self):
        self.power = False


    def moveWheel(self,speed,side): 
        print("The wheels are moving at speed" + str(speed) + " on the "+ str(side))

    def moveArm(self,angle,servo):
        print("The Arm's servo " + str(servo) + " is at angle " + str(angle))

CompleteInstruction().moveArm(44,32)