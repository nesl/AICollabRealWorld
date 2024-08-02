import iRobot_create3
import receive
import mapping
import time
import os
import numpy as np
import cv2
import threading

class robot(iRobot_create3.iRobot_create3, receive.Frame_Receiver, mapping.mapping_o3d):
    def __init__(self, irobot, voxel_size, directory_path):
        iRobot_create3.iRobot_create3.__init__(self, irobot)
        receive.Frame_Receiver.__init__(self)
        mapping.mapping_o3d.__init__(self, voxel_size)
        self.directory_path = directory_path
        self.image_idx = 1
        self.sleep_time = 1

        receiving_thread = threading.Thread(target=self.start_receiving)
        receiving_thread.start()

    def scan(self):
        color_frame, depth_frame = self.get_current_frame()
        depth_intrin_matrix = self.depth_intrin_matrix
        depth_extrin_matrix = self.get_extrinsic_matrix_o3d()
        if depth_extrin_matrix is None:
            print('Error: depth camera extrinsic matrix is None')
            return None
        elif color_frame is None:
            print('Error: color frame is None')
            return None
        elif depth_intrin_matrix is None:
            print('Error: depth intrinsic matrix is None')
            return None
        elif depth_frame is None:
            print('Error: depth frame is None')
            return None
        else:
            self.update_point_cloud(color_frame, depth_frame, depth_intrin_matrix, depth_extrin_matrix)
            np.savetxt(os.path.join(self.directory_path, f'depth_image{self.image_idx}.csv'), depth_frame, delimiter=',')
            cv2.imwrite(os.path.join(self.directory_path, f'color_image{self.image_idx}.png'), color_frame)
            self.save(self.directory_path)
            self.image_idx += 1

    async def scan_full_rotation(self, angle=45):
        if 360 % angle != 0:
            print("Error: 360 must be divisible by angle of rotation")
            return None
        num_rotation = int(360 / angle)
        for i in range(num_rotation):
            await self.turn_left(angle)
            time.sleep(self.sleep_time)
            self.scan()


    async def scan_forward(self, distance, num_steps):
        if distance % num_steps != 0:
            print("Error: distance must be divisible by number of steps")
            return None
        
        step_size = distance / num_steps
        for i in range(num_steps):
            await self.forward_by_distance(step_size)
            time.sleep(self.sleep_time)
            self.scan()
    
    async def scan_turn_left(self, angle=90):
        await self.turn_left(angle)
        time.sleep(self.sleep_time)
        self.scan()
    
    async def scan_turn_right(self, angle=90):
        await self.turn_right(angle)
        time.sleep(self.sleep_time)
        self.scan()

    async def scan_forward(self, distance):
        await self.forward_by_distance(distance)
        time.sleep(self.sleep_time)
        self.scan()

    
        


            
