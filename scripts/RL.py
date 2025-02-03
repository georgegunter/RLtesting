#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Int16, String, Bool
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import NavSatFix, TimeReference
import traceback
import os
import sys
import time


# For loading the nn:
import onnx
import numpy as np
import onnxruntime as ort

import numpy as np
import pandas as pd
import json

# velocity_topic = "/car/state/vel_x"
# sport_mode_topic = "car/state/sport_mode"
# eco_mode_topic = "car/state/eco_mode"
# gantry_topic = "/vsl/latest_gantry"
# vsl_set_speed_topic = "/vsl/set_speed"
# distance_lines_topic="/acc/set_distance"

lead_x_topic = "/lead_dist"
lead_rv_topic = "/rel_vel"
velocity_topic = "/car/state/vel_x"
cmd_vel_topic = "/cmd_vel"

max_speed = 32 ##32 m/s is 71.6 mph

global lead_x
global lead_rv
global velocity
global cmd_vel

global RL_cmd_accel


velocity = 0
lead_x=252
lead_rv=0
cmd_vel=15.0
RL_cmd_accel=0.0


def velocity_callback(data):
    global velocity
    velocity = data.data

def lead_x_callback(data):
    global lead_x
    lead_x = data.data

def lead_rv_callback(data):
    global lead_rv
    lead_rv = data.data

def cmd_vel_callback(data):
    global cmd_vel
    cmd_vel = data.data

class RL_Accel_Controller():
    def __init__(self,model_path):
        self.model_path = model_path
        self.model = onnx.load_model(model_path)
        onnx.checker.check_model(self.model)
        self.ort_session = ort.InferenceSession(model_path)
        
    def accel_func(self,s,v,dv):
        #  state is [av speed, leader speed, headway]
        data = np.array([[v,v+dv,s]]).astype(np.float32)
        outputs = self.ort_session.run(None, {self.ort_session.get_inputs()[0].name: data})
        return outputs[0][0][0]

    def get_accel(self):
        global lead_x
        global lead_rv
        global velocity

        u = self.accel_func(s=float(lead_x),
            v=float(velocity),
            dv=float(lead_rv))

        return u

class RL_controller:
    def __init__(self):
        rospy.init_node('RLtesting', anonymous=True)
        print('rospy params: ')
        print(rospy.get_param_names())
        # model_path = 'super_resolution_kinda_safe.onnx'
        model_path = rospy.get_param("/RL_testing/rl_filename")
        print(model_path)
        self.RL_accel = RL_Accel_Controller(model_path)


        rospy.Subscriber(lead_x_topic, Float64, lead_x_callback)
        rospy.Subscriber(lead_rv_topic, Float64, lead_rv_callback)
        rospy.Subscriber(velocity_topic,Float64, velocity_callback)

        global RL_cmd_accel_pub
        RL_cmd_accel_pub = rospy.Publisher('/RL_cmd_accel', Float64, queue_size=1000)
 
        self.rate = rospy.Rate(20)

    def loop(self):
        while not rospy.is_shutdown():
            try:
                # Get accel cmd to publish:

                global lead_x
                global lead_rv
                global velocity
                global cmd_vel
                global RL_cmd_accel_pub

                try:
                    RL_cmd_accel = self.RL_accel.get_accel()
                    print(RL_cmd_accel)
                except Exception as e:
                    print(e)
                    traceback.print_exc()
                    print('exception to solver:')
                    RL_cmd_accel = 0.0


                RL_cmd_accel_pub.publish(RL_cmd_accel)


            except Exception as e:
                print(e)
                traceback.print_exc()
                print("Something has gone wrong.")
            self.rate.sleep()

if __name__ == '__main__':
    try:
        head = RL_controller()
        head.loop()
    except Exception as e:
        print(e)
        traceback.print_exc()
        print("An exception occurred")
