import threading
import logging
import time

import numpy as np

import algorithm.input_struct as din

buffer = []

def VR_data_in(head_rot, head_pos, left_hand_rot, left_hand_pos, right_hand_rot, right_hand_pos):
    din.head_rot = head_rot
    din.head_pos = head_pos

    din.left_hand_rot = left_hand_rot
    din.left_hand_pos = left_hand_pos

    din.right_hand_rot = right_hand_rot
    din.right_hand_pos = right_hand_pos

def Sensor_data_in(imu1_accin, imu1_gyroin):
    din.imu1_acc = imu1_accin
    din.imu1_gyro = imu1_gyroin
    

def algorithm():
    print(buffer)

def test():
    print("test!")

def intialize():
    x = threading.Thread(target=thread_function, args=(1,))
    x.daemon = True
    x.start()


def thread_function(name):
    while(True):
        logging.info("Thread %s: starting", name)
        print(buffer)
        print(din.head_rot)
        print(din.head_pos)

        print(din.left_hand_rot)
        print(din.left_hand_pos)

        print(din.right_hand_rot)
        print(din.right_hand_pos)

        time.sleep(1)
        logging.info("Thread %s: finishing", name)