# This file is for actually executing the instruction that needs to be run on the rover. 
TEST = True

import time

if not TEST:
    import PCA9685_v4_armtest



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
        print("Rover Stopping")

    def move(self, speed, side):
        print(f"The wheels are moving at speed {speed} on the {side}")
        if not TEST and self.power:
            if side == 'forward':
                self.pwm.setServoPulse(0,speed)
            if side == 'back':
                self.pwm.setServoPulse(0,-speed)

    def arm(self, angle, servo):
        print(f"The Arm's servo {servo} is at angle {angle}")
        if not TEST and self.power:
            self.pwm.setServoPulse(int(servo),int(angle))

    def powerSet(self, status):
        print(f"The rover's power is {status}")
        self.power_status = status

    def get_status(self):
        uptime = time.monotonic()
        return f"""
The rover status will be printed here
The rover uptime is {uptime:.2f} seconds
        """