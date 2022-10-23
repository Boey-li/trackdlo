#!/usr/bin/env python

import matplotlib.pyplot as plt
import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2, PointField, Image
import sensor_msgs.point_cloud2 as pcl2
import std_msgs.msg

import struct
import time
import cv2
import numpy as np
import math

import time
import pickle as pkl

import message_filters
from sklearn.neighbors import NearestNeighbors
import open3d as o3d
from scipy import ndimage
import pyrealsense2 as rs


rect = (0,0,0,0)
startPoint = False
endPoint = False

# this mask will get updated each iteration
mouse_mask = None

def on_mouse(event, x, y, flags, params):

    global rect, startPoint, endPoint

    # get mouse click
    if event == cv2.EVENT_LBUTTONDOWN:

        if startPoint == True and endPoint == True:
            startPoint = False
            endPoint = False
            rect = (0, 0, 0, 0)

        if startPoint == False:
            rect = (x, y, x, y)
            startPoint = True
        elif endPoint == False:
            rect = (rect[0], rect[1], x, y)
            endPoint = True
    
    # draw rectangle when mouse hovering
    elif event == cv2.EVENT_MOUSEMOVE and startPoint == True and endPoint == False:
        rect = (rect[0], rect[1], x, y)

def callback (rgb, depth, pc):
    global rect, startPoint, endPoint, mouse_mask

    cur_image = ros_numpy.numpify(rgb)

    # initialize mask if none
    if mouse_mask is None:
        mouse_mask = np.ones(cur_image.shape)

    # convert color for opencv display
    cur_image = cv2.cvtColor(cur_image.copy(), cv2.COLOR_BGR2RGB)
    frame = cur_image.copy()

    # filter with mask
    frame = (frame * mouse_mask).astype('uint8')

    # print('start point = ', startPoint)
    # print('end point = ', endPoint)
    # print('rect = ', rect)

    cv2.namedWindow('frame')
    cv2.setMouseCallback('frame', on_mouse)    

    key = cv2.waitKey(10)

    if key == 114: # r
        # reset everyhting
        frame = cur_image.copy()
        startPoint = False
        endPoint = False
        mouse_mask = np.ones(cur_image.shape)
        cv2.imshow('frame',frame)
    else:
        #drawing rectangle
        if startPoint == True and endPoint != True:
            cv2.rectangle(frame, (rect[0], rect[1]), (rect[2], rect[3]), (0, 0, 255), 2)
        
        # if another rectangle is drawn, update mask
        if startPoint == True and endPoint == True:
            mouse_mask[rect[1]:rect[3], rect[0]:rect[2], :] = 0

        cv2.imshow('frame',frame)


if __name__=='__main__':
    rospy.init_node('test', anonymous=True)

    rgb_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
    depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
    pc_sub = message_filters.Subscriber('/camera/depth/color/points', PointCloud2)

    tracking_img_pub = rospy.Publisher ('/tracking_img', Image, queue_size=10)
    mask_img_pub = rospy.Publisher('/mask', Image, queue_size=10)

    ts = message_filters.TimeSynchronizer([rgb_sub, depth_sub, pc_sub], 10)
    ts.registerCallback(callback)

    rospy.spin()