import threading
import logging
import time

import numpy as np

import algorithm.input_struct as din
import algorithm.output_struct as dout

buffer = []

def VR_data_in(head_rot, head_pos, left_hand_rot, left_hand_pos, right_hand_rot, right_hand_pos):
    din.head_rot = head_rot
    din.head_pos = head_pos

    din.left_hand_rot = left_hand_rot
    din.left_hand_pos = left_hand_pos

    din.right_hand_rot = right_hand_rot
    din.right_hand_pos = right_hand_pos

def Sensor_data_in(imuNum, imu_accin, imu_gyroin):
    if(imuNum == ["imu1"]):
        din.imu1_acc = imu_accin
        din.imu1_gyro = imu_gyroin
    elif(imuNum == ["imu2"]):
        din.imu2_acc = imu_accin
        din.imu2_gyro = imu_gyroin
    elif(imuNum == ["imu3"]):
        din.imu3_acc = imu_accin
        din.imu3_gyro = imu_gyroin
    elif(imuNum == ["imu4"]):
        din.imu4_acc = imu_accin
        din.imu4_gyro = imu_gyroin
    elif(imuNum == ["imu5"]):
        din.imu5_acc = imu_accin
        din.imu5_gyro = imu_gyroin
    elif(imuNum == ["imu6"]):
        din.imu6_acc = imu_accin
        din.imu6_gyro = imu_gyroin
    elif(imuNum == ["imu7"]):
        din.imu7_acc = imu_accin
        din.imu7_gyro = imu_gyroin
    elif(imuNum == ["imu8"]):
        din.imu8_acc = imu_accin
        din.imu8_gyro = imu_gyroin

def algorithm(deltaTime):
    # detalTime: time in seconds that has passed since this function is called last time 

    # din: tracking data given to the algorithm. 
    # To access fields: din.head_rot, din.imu1_acc... See from input_struct file.

    # dout: joints data computed from tracking data
    # To access fields: din.left_upper_leg, din.waist... See from output_struct file.

    # numpy as np is imported. If others are used, list them in requirements.txt

    


def test():
    print("test!")

def intialize():
    x = threading.Thread(target=thread_function, args=(1,))
    x.daemon = True
    x.start()


def thread_function(name):
    while(True):
        logging.info("Thread %s: starting", name)
        # print(buffer)
        # print(din.head_rot)
        # print(din.head_pos)

        # print(din.left_hand_rot)
        # print(din.left_hand_pos)

        # print(din.right_hand_rot)
        # print(din.right_hand_pos)

        print(din.imu1_acc)
        print(din.imu1_gyro)
        print("\n")

        time.sleep(1)
        logging.info("Thread %s: finishing", name)