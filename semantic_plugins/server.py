# -*- coding: utf-8 -*-
"""
.. codeauthor:: Daniel Seichter <daniel.seichter@tu-ilmenau.de>
"""
import argparse
import os
from pydoc import describe
import time
import cv2
import numpy as np
import matplotlib.pyplot as plt
import torch
import torch.nn.functional as F
import pyrealsense2 as rs  
from src.args import ArgumentParserRGBDSegmentation
from src.build_model import build_model
from src.prepare_data import prepare_data
import rospy
import sys
sys.path.append("/home/futianqi/Downloads/robo_ws/devel/lib/python2.7/dist-packages")
sys.path.append("/opt/ros/melodic/lib/python2.7/dist-packages")
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from esa.srv import *
# from std_msgs.msg import Int32MultiArray, MultiArrayDimension

count=0

#遍历计算描述符
def calculate_vector(array,depth=None):
    time1=time.time()
    vector = []
    desc = []
    if depth is None:
        for row in range(array.shape[0]):
            for col in range(array.shape[1]):
                ele=array[row][col]
                if len(vector)>0:
                    flag=False
                    for v in vector:
                        if v[0]==ele:
                            v[1]+=row
                            v[2]+=col
                            v[3]+=1
                            flag=True
                            break
                    if flag==False:
                        vector.append([ele,row,col,1])
                else:
                    vector.append([ele,row,col,1])
        for v in vector:
           for v in vector:
            tmp=[v[0],round(v[1]/v[3]),round(v[2]/v[3])]
            desc.append(tmp)
    else:
        for row in range(array.shape[0]):
            for col in range(array.shape[1]):
                ele=array[row][col]
                if len(vector)>0:
                    flag=False
                    for v in vector:
                        if v[0]==ele:
                            v[1]+=row
                            v[2]+=col
                            v[3]+=int(depth[row,col])
                            v[4]+=1
                            flag=True
                            break
                    if flag==False:
                        vector.append([ele,row,col,int(depth[row,col]),1])
                else:
                    vector.append([ele,row,col,int(depth[row,col]),1])
        for v in vector:
            tmp=[v[0],round(v[1]/v[4]),round(v[2]/v[4]),round((v[3]/v[4])/10)]
            desc.append(tmp)
    time2=time.time()
    print("post_processing time = ",time2-time1)
    return desc

def server():
        rospy.init_node("esa_server",anonymous=True)
        machine=rospy.Service("calculate_vector",rgbd,callback)
        print("node init,service ready...")
        rospy.spin()

def callback(msg):
        global count
        img_rgb = bridge.imgmsg_to_cv2(msg.rgb_data, desired_encoding="bgr8")
        img_depth = np.array(msg.depth_data).reshape(msg.height,msg.width)*args.depth_scale
        # h, w, _ = img_rgb.shape
        h,w = msg.height, msg.width
        # cv2.imwrite("rgb_image"+str(count)+".png",img_rgb)
        # cv2.imwrite("depth_image"+str(count)+".png",img_depth)
        # cv2.waitKey(10)
        # print(img_rgb)
        # import time
        # # preprocess sample
        start_time = time.time()
        sample = preprocessor({'image': img_rgb, 'depth': img_depth})

        # add batch axis and copy to device
        image = sample['image'][None].to(device)
        depth = sample['depth'][None].to(device)

        # apply network
        pred = model(image, depth)
        print(h,"and",w)
        pred = F.interpolate(pred, (h, w),
                                mode='bilinear', align_corners=False)
        pred = torch.argmax(pred, dim=1)
        pred = pred.cpu().numpy().squeeze().astype(np.uint8)
        print(type(pred))
        pred_desc=calculate_vector(pred,img_depth)
        print(pred_desc)
        pred_colored = dataset.color_label(pred, with_void=True)
        #print(type(pred_colored))
        
        pred_mask=bridge.cv2_to_imgmsg(pred_colored,"bgr8")
        cv2.imwrite("predmask"+str(count)+".png", pred_colored)
        end_time = time.time()
        print("total_time = ",end_time-start_time)
        res = []

        count+=1
        for v in pred_desc:
            res += v
        return [res, pred_mask]        


if __name__ == '__main__':
    #加载模型
    # arguments
    parser = ArgumentParserRGBDSegmentation(
        description='Efficient RGBD Indoor Sematic Segmentation (Inference)',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.set_common_args()
    parser.add_argument('--ckpt_path', type=str,
                        required=True,
                        help='Path to the checkpoint of the trained model.')
    parser.add_argument('--depth_scale', type=float,
                        default=1.0,
                        help='Additional depth scaling factor to apply.')
    args = parser.parse_args()

    # dataset
    args.pretrained_on_imagenet = False  # we are loading other weights anyway
    dataset, preprocessor = prepare_data(args, with_input_orig=True)
    n_classes = dataset.n_classes_without_void

    # model and checkpoint loading
    model, device = build_model(args, n_classes=n_classes)
    checkpoint = torch.load(args.ckpt_path,
                            map_location=lambda storage, loc: storage)
    model.load_state_dict(checkpoint['state_dict'])
    print('Loaded checkpoint from {}'.format(args.ckpt_path))

    model.eval()
    model.to(device)
    bridge = CvBridge()

    print("prepare model finish!")

    server()

   
