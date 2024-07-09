# This file is for actually executing the instruction that needs to be run on the rover. 
TEST = True

import time

if not TEST:
    import PCA9685_v4_armtest

SERVOBASE = 4
MOTORA = 2
MOTORB = 3

class CompleteInstruction:
    def __init__(self):
        self.power = False

        if not TEST:
            self.pwm = PCA9685_v4_armtest.PCA9685(0x40, debug=False)
            self.pwm.setPWMFreq(50)

    def handle_instruction(self, instruction):
        words = instruction.split()
        if not words:
            return ""
        
        command, *arguments = words

        if command == "status":
            return self.get_status()
        elif command =="refresh":
            return "None"
        elif command == "move" and len(arguments) >= 2:
            self.move(arguments[0], arguments[1])
        elif command == "arm" and len(arguments) >= 2:
            self.arm(arguments[0], arguments[1])
        elif command == "power" and len(arguments) >= 1:
            self.powerSet(arguments[0])
        elif command == "stop" and len(arguments) >= 0:
            self.stop()
        else:
            return "Error From command"
        return "None"

    def stop(self):
        print("Stop")
        if not TEST:
            self.pwm.setServoPulse(5, 1500)
            self.pwm.setServoPulse(6, 1500)

    def move(self, side, speed):
        print(f"Move. Speed:{speed}. Side:{side}")
        if not TEST and self.power:
            speed = 2*min(200, int(speed))
            if side == "forward":
                self.pwm.setServoPulse(MOTORA,1500+speed)
                self.pwm.setServoPulse(MOTORB,1500+speed)
            elif side == "back":
                self.pwm.setServoPulse(MOTORA,1500-speed)
                self.pwm.setServoPulse(MOTORB,1500-speed)
            elif side == "left":
                self.pwm.setServoPulse(MOTORA,1500-speed)
                self.pwm.setServoPulse(MOTORB,1500+speed)
            elif side == "right":
                self.pwm.setServoPulse(MOTORA,1500+speed)
                self.pwm.setServoPulse(MOTORB,1500-speed)

    def arm(self, angle, servo):
        print(f"Arm. Servo:{servo}. Angle:{angle}")
        if not TEST and self.power:
            self.pwm.setServoPulse(int(servo) + SERVOBASE,int(angle))

    def powerSet(self, status):
        print(f"Power: {status}")
        if status=="on":
            self.power=True
        else:
            self.power=False


    def get_status(self):
        uptime = time.monotonic()
        return f"""
The rover status will be printed here
The rover uptime is {uptime:.2f} seconds
        """
