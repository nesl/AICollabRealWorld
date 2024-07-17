import socket
import pickle
import numpy as np
import cv2
import numpy as np
import cv2
import open3d as o3d

import matplotlib.pyplot as plt
import BEV
import mapping
import threading

def receive_full_data(conn, length):
    data = b''
    while len(data) < length:
        packet = conn.recv(length - len(data))
        if not packet:
            return None
        data += packet
    return data

class Frame_Receiver:
    def __init__(self):
        self.depth_intrin_matrix = None
        self.color_intrin_matrix = None
        self.depth_to_color_extrin_matrix = None
        self.latest_color_frame = None
        self.latest_depth_frame = None

    def update_frames(self, color_frame, depth_frame):
        self.latest_color_frame = color_frame
        self.latest_depth_frame = depth_frame

    def get_current_frame(self):
        return self.latest_color_frame, self.latest_depth_frame

    def start_receiving(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_address = ('0.0.0.0', 65432)
        s.bind(server_address)
        s.listen(1)

        print('Waiting for a connection...')

        while True:
            conn, addr = s.accept()
            print('Connection from', addr)
            first_time = True
            try:
                while True:
                    color_length = int.from_bytes(conn.recv(4), byteorder='big')
                    color_data = receive_full_data(conn, color_length)

                    depth_length = int.from_bytes(conn.recv(4), byteorder='big')
                    depth_data = receive_full_data(conn, depth_length)

                    if first_time:
                        depth_intrin_matrix_data_length = int.from_bytes(conn.recv(4), byteorder='big')
                        depth_intrin_matrix_data = receive_full_data(conn, depth_intrin_matrix_data_length)

                        color_intrin_matrix_data_length = int.from_bytes(conn.recv(4), byteorder='big')
                        color_intrin_matrix_data = receive_full_data(conn, color_intrin_matrix_data_length)

                        depth_to_color_extrin_matrix_data_length = int.from_bytes(conn.recv(4), byteorder='big')
                        depth_to_color_extrin_matrix_data = receive_full_data(conn, depth_to_color_extrin_matrix_data_length)

                        self.depth_intrin_matrix = pickle.loads(depth_intrin_matrix_data)
                        self.color_intrin_matrix = pickle.loads(color_intrin_matrix_data)
                        self.depth_to_color_extrin_matrix = pickle.loads(depth_to_color_extrin_matrix_data)

                        print(self.depth_intrin_matrix)
                        print(self.color_intrin_matrix)
                        print(self.depth_to_color_extrin_matrix)
                        first_time = False

                    # Deserialize the data using pickle
                    if color_data and depth_data:
                        color_image = pickle.loads(color_data)
                        depth_image_cm = pickle.loads(depth_data)

                        self.update_frames(color_image, depth_image_cm)
                    else:
                        print("Connection closed by the sender.")
                        break
            except (ConnectionResetError, ConnectionAbortedError):
                print("Connection lost. Waiting for reconnection...")
            finally:
                conn.close()
                print("Connection closed. Waiting for next connection...")