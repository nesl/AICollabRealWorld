from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note
import re
import numpy as np
import os
import BEV
import math

# Lab room measurement:
#   width: 30 * 30 = 900cm
#   height: 30 * 34 = 1020cm

# GLOBAL VARIABLE
LAB_WIDTH = 900
LAB_LENGTH = 1020
BLOCK_UNIT = 15
LIGHT_COLOR = ["white", "red", "green", "blue"]
# GLOBAL VARIABLE

class iRobot:
    def __init__(self, irobot):
        self.irobot = irobot
        self.radius = 33 #cm
        self.height = 10 #cm

        # 0 for empty space, 1 for current position, 2 for obstacles or walls, 3 for target objects, 4 for other robots, -1 for unknown regions
        self.occupancy_map = np.full((LAB_WIDTH // BLOCK_UNIT, LAB_WIDTH // BLOCK_UNIT), -1)
        self.position = [[0, 0]] #cm
        self.step_size = BLOCK_UNIT
        self.orientation = [0]

        # 0 for default, 1 for red, 2 for green, 3 for blue
        self.lights = [0]
    
    async def set_lights(self, lights):
        if lights == 0:
            await self.irobot.set_lights_on_rgb(0, 0, 0)
        elif lights == 1:
            await self.irobot.set_lights_on_rgb(255, 0, 0)
        elif lights == 2:
            await self.irobot.set_lights_on_rgb(0, 255, 0)
        elif lights == 3:
            await self.irobot.set_lights_on_rgb(0, 0, 255)
        else:
            print("Error: invalid argument \"light\", must be 1, 2, 3, or 4")
            return None
        print(f"iRobot: set lights to {LIGHT_COLOR[lights]}")
        self.lights.append(lights)
        self.position.append(self.position[-1])
        self.orientation.append(self.orientation[-1])
    
    def get_lights(self):
        return self.lights[-1]
    
    async def turn_left(self, angle=90):
        print(f"iRobot: turn {angle} degree left")
        self._change_orientation(-angle) #We define ccw to be negative
        await self.irobot.turn_left(angle)
    
    async def turn_right(self, angle=90):
        print(f"iRobot: turn {angle} degree right")
        self._change_orientation(angle) #We define cw to be positive
        await self.irobot.turn_right(angle)
    
    async def turn_back(self):
        print("iRobot: turn 180 degree back")
        self._change_orientation(180)
        await self.irobot.turn_right(180)

    async def forward_by_distance(self, distance=BLOCK_UNIT):
        print(f"iRobot: forward {distance}cm / {distance / self.step_size} steps")
        self._forward(distance)
        await self.irobot.move(distance)
    
    async def forward_by_steps(self, num_steps=1):
        print(f"iRobot: forward {num_steps * self.step_size}cm / {num_steps} steps")
        self._forward(num_steps * self.step_size)
        await self.irobot.move(num_steps * self.step_size)
    
    async def backward_by_distance(self, distance=BLOCK_UNIT):
        print(f"iRobot: backward {distance}cm / {distance / self.step_size} steps")
        self._forward(-distance)
        await self.irobot.move(-distance)
    
    async def backward_by_steps(self, num_steps=1):
        print(f"iRobot: backward {num_steps * self.step_size}cm / {num_steps} steps")
        self._forward(-num_steps * self.step_size)
        await self.irobot.move(num_steps * self.step_size)
    
    def _change_orientation(self, angle):
        self.orientation.append((self.orientation[-1] + angle) % 360)
        self.position.append(self.position[-1])
        self.lights.append(self.lights[-1])
    
    def _forward(self, distance):
        x_displacement = distance * np.sin(self.orientation[-1] / 180 * np.pi)
        y_displacement = distance * np.cos(self.orientation[-1] / 180 * np.pi)
        self.position.append([self.position[-1][0] + x_displacement, self.position[-1][1] + y_displacement])
        self.orientation.append(self.orientation[-1])
        self.lights.append(self.lights[-1])
    
    def log_status(self):
        print(">>----------------------------------------<<")
        print(f">>orientation: {self.orientation[-1]}")
        print(f">>position: {self.position[-1]}")
        print(">>----------------------------------------<<")

    def get_extrinsic_matrix(self):
        return BEV.get_extrinsic_matrix(0, math.radians(self.orientation[-1]), 0, self.position[-1][0], 0, self.position[-1][1])
    
    def get_extrinsic_matrix_o3d(self):
        return BEV.get_extrinsic_matrix(0, -math.radians(self.orientation[-1]), 0, self.position[-1][0] / 1100, 30, self.position[-1][1] / 1100)
    
    def save(self, dir):
        np.savetxt(os.path.join(dir, f'position.csv'), self.position[1:], delimiter=',')
        np.savetxt(os.path.join(dir, f'orientation.csv'), self.orientation[1:], delimiter=',')
        np.savetxt(os.path.join(dir, f'light.csv'), self.lights[1:], delimiter=',')

    

def parse_integers_from_string(s):
    integers = re.findall(r'-?\d+', s)
    return [int(num) for num in integers]

if __name__ == "__main__":
    robot = Create3(Bluetooth())
    iR = iRobot(robot)
    @event(iR.irobot.when_play)
    async def play(robot):
        while True:
            userinput = input()
            os.system("clear")
            if userinput == "l" or userinput == "left":
                await iR.turn_left()
            elif userinput == "r" or userinput == "right":
                await iR.turn_right()
            elif userinput == "b" or userinput == "back":
                await iR.turn_back()
            elif re.match(r'^l\d+$', userinput) or re.match(r'^left\d+$', userinput):
                angle = iRobot.parse_integers_from_string(userinput)[0]
                await iR.turn_left(angle)
            elif re.match(r'^r\d+$', userinput) or re.match(r'^right\d+$', userinput):
                angle = iRobot.parse_integers_from_string(userinput)[0]
                await iR.turn_right(angle)
            elif re.match(r'^f\d+$', userinput) or re.match(r'^forward\d+$', userinput):
                distance = iRobot.parse_integers_from_string(userinput)[0]
                await iR.forward_by_distance(distance)
            elif re.match(r'^b\d+$', userinput) or re.match(r'^backward\d+$', userinput):
                distance = iRobot.parse_integers_from_string(userinput)[0]
                await iR.backward_by_distance(distance)
            elif userinput == "white":
                await iR.set_lights(0)
            elif userinput == "red":
                await iR.set_lights(1)
            elif userinput == "green":
                await iR.set_lights(2)
            elif userinput == "blue":
                await iR.set_lights(3)
            elif userinput == "stats":
                iR.log_status()
                continue
    iR.irobot.play()
    