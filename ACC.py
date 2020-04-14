import matplotlib.pyplot as plt

import numpy as np

import serial

import time

import re


T = 10.0/0.1; # time interval

t = np.arange(0,10,0.1) # time vector


serdev = '/dev/ttyACM0'

s = serial.Serial(serdev,115200)

ACC_X = []
ACC_Y = []
ACC_Z = []
event = []

for i in range(int(T)):

    line=s.readline() # Read an echo string from K66F terminated with '\n'

    num = list(map(float,line.split()))

    # print (num)

    ACC_X.append(num[0])
    ACC_Y.append(num[1])
    ACC_Z.append(num[2])
    event.append(num[3])


fig, ax = plt.subplots(2, 1)

ax[0].plot(t, ACC_X, color="blue",  label="x")

ax[0].plot(t, ACC_Y, color="red",   label="y")

ax[0].plot(t, ACC_Z, color="green", label="z")

ax[0].set_xlabel('Time')

ax[0].set_ylabel('ACC Vector')

ax[1].stem(t, event) 

ax[1].set_xlabel('Time')

ax[1].set_ylabel('Tilt')

plt.show()

s.close()