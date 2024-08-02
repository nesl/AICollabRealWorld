import robot
from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note
import os
import iRobot_create3
import re


directory_path = "test18"
if os.path.isdir(directory_path):
    print("Folder exist, data collected will replace original data")
else:
    os.mkdir(directory_path)

r = Create3(Bluetooth())
iR = robot.robot(r, 15, directory_path)

@event(r.when_play)
async def play(robot):
    while True:
        userinput = input()
        os.system("clear")
        if userinput == 'l':
            await iR.scan_turn_left()
        elif userinput == 'r':
            await iR.scan_turn_right()
        elif re.match(r'^l\d+$', userinput) or re.match(r'^left\d+$', userinput):
            angle = iRobot_create3.parse_integers_from_string(userinput)[0]
            await iR.scan_turn_left(angle)
        elif re.match(r'^r\d+$', userinput) or re.match(r'^right\d+$', userinput):
            angle = iRobot_create3.parse_integers_from_string(userinput)[0]
            await iR.scan_turn_right(angle)
        elif re.match(r'^f\d+$', userinput) or re.match(r'^forward\d+$', userinput):
            distance = iRobot_create3.parse_integers_from_string(userinput)[0]
            await iR.scan_forward(distance)
        elif userinput == 's':
            await iR.scan_full_rotation(45)
        elif userinput == 'pcd':
            iR.visualize_point_cloud_o3d()

r.play()