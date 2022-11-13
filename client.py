import argparse
import os
import time
import cv2
import numpy as np
import matplotlib.pyplot as plt
import torch
import torch.nn.functional as F
from glob import glob
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
from esa.srv import rgbd
from std_msgs.msg import Int32MultiArray
from mpl_toolkits.mplot3d import Axes3D  

def _load_img(fp):
    img = cv2.imread(fp, cv2.IMREAD_UNCHANGED)
    if img.ndim == 3:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    return img

if __name__ == '__main__':
    basepath = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            'samples')
    rgb_filepaths = sorted(glob(os.path.join(basepath, '*_rgb.*')))
    depth_filepaths = sorted(glob(os.path.join(basepath, '*_depth.*')))
    # assert args.modality == 'rgbd', "Only RGBD inference supported so far"
    assert len(rgb_filepaths) == len(depth_filepaths)
    filepaths = zip(rgb_filepaths, depth_filepaths)

    rospy.wait_for_service('calculate_vector')
    remote = rospy.ServiceProxy('calculate_vector', rgbd)
    bridge = CvBridge()

    for fp_rgb, fp_depth in filepaths:
        try:
            # 获取样本数据
            img_rgb = _load_img(fp_rgb)
            #竟然是np.ndarray类型返回值
            print(img_rgb)
            img_depth = _load_img(fp_depth).astype("float32")

            depth_height,depth_width=img_depth.shape
            
            rgb=bridge.cv2_to_imgmsg(img_rgb)
            d=img_depth.flatten()
            t0=time.time()
            res = remote.call(rgb,d,depth_width,depth_height)
            t1=time.time()
            print("cost_time",t1-t0)
            desc=np.array(res.desc).reshape(-1,4)
            print("get desc vector:")
            print(desc)
            # 查看一下像素问题
            pred_mask = bridge.imgmsg_to_cv2(res.pred_result, 'passthrough')
            print(pred_mask)
            fig, axs = plt.subplots(1, 3, figsize=(16, 3))
            [ax.set_axis_off() for ax in axs.ravel()]
            axs[0].imshow(img_rgb)
            axs[1].imshow(img_depth, cmap='gray')
            axs[2].imshow(pred_mask,cmap='gray')
            
            plt.suptitle(f"Image: ({os.path.basename(fp_rgb)}, "
                        f"{os.path.basename(fp_depth)})")
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')

            dict={0:'m',1:'y',4:'b',5:'g',6:'r',7:'w',8:'c'}
            for c in [0,1,4,5,6,7,8]:
                xs=np.array([])
                ys=np.array([])
                zs=np.array([])
                for i in range(320):
                    for j in range(180):
                            if pred_mask[j][i] == int(c):
                                if img_depth[j][i] >0.0 and img_depth[j][i] <10000.0:
                                    xs=np.append(xs,i)
                                    zs=np.append(zs,180-1-j)
                                    ys=np.append(ys,img_depth[j][i])
                ax.scatter(xs, ys, zs, c=dict[c], marker='.',alpha=0.5)      

            #todo 可以做散点，体素渲染太慢了
            ax.set_xlabel('X Label')
            ax.set_ylabel('Y Label')
            ax.set_zlabel('Z Label')
            plt.show()

        except rospy.ServiceException as e:
            print ("Service call failed,",e)
    

