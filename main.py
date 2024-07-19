import receive
import mapping
import cv2
import matplotlib.pyplot as plt
import iRobot
import os
import re
import threading
import numpy as np
from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note
import time
from PIL import Image

directory_path = "test7"
os.mkdir(directory_path)

if __name__ == '__main__':
    # Create an instance of Frame_Receiver
    frame_receiver = receive.Frame_Receiver()
    # Start the frame receiving process in a separate thread
    receiving_thread = threading.Thread(target=frame_receiver.start_receiving)
    receiving_thread.start()
    
    robot = Create3(Bluetooth())
    iR = iRobot.iRobot(robot)

    map = mapping.mapping_o3d(15)
    #map = mapping.mapping_o3d(15)

    @event(robot.when_play)
    async def play(robot):
        i = 1
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
            elif userinput == "pcd":
                map.visualize_point_cloud()
                continue

            time.sleep(3)
            color_frame, depth_frame = frame_receiver.get_current_frame()
            depth_intrin_matrix = frame_receiver.depth_intrin_matrix
            #depth_extrin_matrix = iR.get_extrinsic_matrix()
            depth_extrin_matrix = iR.get_extrinsic_matrix_o3d()
            if depth_extrin_matrix is None:
                print('Warning: depth camera extrinsic matrix is None')
            elif color_frame is None:
                print('Warning: color frame is None')
            elif depth_intrin_matrix is None:
                print('Warning: depth intrinsic matrix is None')
            elif depth_frame is None:
                print('Warning: depth frame is None')
            else:
                map.update_point_cloud(color_frame, depth_frame, depth_intrin_matrix, depth_extrin_matrix)
                map.visualize_point_cloud_o3d()
                np.savetxt(os.path.join(directory_path, f'depth_image{i}.csv'), depth_frame, delimiter=',')
                cv2.imwrite(os.path.join(directory_path, f'color_image{i}.png'), color_frame)
                iR.save(directory_path)
                i += 1
                '''
                map.update_point_cloud(depth_frame, depth_intrin_matrix, depth_extrin_matrix)
                map.update_occupancy_grid()

                np.savetxt(os.path.join(directory_path, f'depth_image{i}.csv'), depth_frame, delimiter=',')
                cv2.imwrite(os.path.join(directory_path, f'color_image{i}.png'), color_frame)

                fig, ax = plt.subplots()
                cax = ax.imshow(map.occupancy_grid_2d, cmap='gray', origin='lower')

                # Adding grid lines
                ax.grid(True, which='both', color='k', linestyle='-', linewidth=0.5)
                ax.set_xticks(np.arange(0.5, map.occupancy_grid_2d.shape[1], 1))
                ax.set_yticks(np.arange(0.5, map.occupancy_grid_2d.shape[0], 1))
                ax.set_xticklabels([])
                ax.set_yticklabels([])

                # Setting labels
                ax.set_xlabel('X')
                ax.set_ylabel('Y')

                cv2.imshow("color image", color_frame)
                depth_frame = cv2.applyColorMap(cv2.convertScaleAbs(depth_frame, alpha=0.5), cv2.COLORMAP_JET)
                cv2.imshow("depth image", depth_frame)
                i += 1
                cv2.waitKey(100)
                plt.pause(1)
                print("Processing the latest frames...")
                '''
                
    robot.play()