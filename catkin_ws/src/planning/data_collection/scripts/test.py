#!/usr/bin/env python

'''
Script that test the data collected
'''
import rospy
#import pandas as pd

from simulacion import spawn_model, set_model_state, delete_model
from juskeshino_tools.JuskeshinoHardware import JuskeshinoHardware

'''
df = pd.read_csv('data.csv')

print(df)


with open('data.csv') as file:
    csv_reader = csv.reader(file, delimiter=',')

    for row in csv_reader:
        username = row['username']
       print('El username es:', username)
data_collected = 
'''

# Data collected 113

def main():
    rospy.init_node("data_test")
    object_name = '004_sugar_box' #'jelly'

    '''
    position_x =  0.375177651643753 #¿con respecto a que posicion guarda? con respecto al base link
    position_y = 0.077661722898483
    position_z = 0.805312156677246
    '''
    position_x = 7.62
    position_y = 5.8
    position_z = 0.8

    orientation_x = 0.040487208590857
    orientation_y = -0.300882987712822
    orientation_z = -0.202359399181383
    orientation_w = 0.931064384029631


    '''
    orientation_x = 0.785995303040303
    orientation_y = 0.278570634691074
    orientation_z = 0.545208709234493
    orientation_w = -0.085774404464576
    '''

    '''
    "object_point_cloud": "",  # Point cloud from a base link reference

    "gripper_position_x": "", 
    "gripper_position_y": "", 
    "gripper_position_z": "", 

    "gripper_orientation_x": "",
    "gripper_orientation_y": "",
    "gripper_orientation_z": "",
    "gripper_orientation_w": "",

    "head_pose_pan": "",
    "head_pose_tilt": "",
    '''
    articular_trajectory = []
   
    '''
    articular_trajectory.append(0.127251954015721)
    articular_trajectory.append(-0.13554683891345)
    articular_trajectory.append(-0.078330564506062)
    articular_trajectory.append(1.99205006976658)
    articular_trajectory.append(-0.170218363727513)
    articular_trajectory.append(-0.816945438551124)
    articular_trajectory.append(0.021250569287713)
    '''
    
    articular_trajectory.append(0.356235176656258)
    articular_trajectory.append(-0.577973942453831)
    articular_trajectory.append(0.111510296859767)
    articular_trajectory.append(1.59703029421676)
    articular_trajectory.append(-0.556109646700829)
    articular_trajectory.append(-1.36560523419521)
    articular_trajectory.append(-0.110030367817781)
    
    print(articular_trajectory)

    # --------------------------------------------------------------------------
    
    delete_model('hsr_pringles_03')
    delete_model('apple_01')
    delete_model('hsr_orange_03')
    

    # --------------------------------------------------------------------------
    spawn_model(object_name)

    
    # --------------------------------------------------------------------------
    twist_linear_x = 0.0
    twist_linear_y = 0.0
    twist_linear_z = 0.0

    twist_angular_x = 0.0
    twist_angular_y = 0.0
    twist_angular_z = 0.0
        

    set_model_state(model_name = object_name, 
                    pose_position_x = position_x, 
                    pose_position_y = position_y, 
                    pose_position_z = position_z,
                    pose_orientation_x = orientation_x,
                    pose_orientation_y = orientation_y,
                    pose_orientation_z = orientation_z,
                    pose_orientation_w = orientation_w,
                    twist_linear_x = twist_linear_x,
                    twist_linear_y = twist_linear_y,
                    twist_linear_z = twist_linear_z, 
                    twist_angular_x = twist_angular_x, 
                    twist_angular_y = twist_angular_y, 
                    twist_angular_z = twist_angular_z, 
                    reference_frame = 'world')

    # --------------------------------------------------------------------------
    
    JuskeshinoHardware.setNodeHandle()
    PREPARE_TOP_GRIP  = [-1.25, 0.3, 0, 2.4, 0, 0.7,0]
    JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_TOP_GRIP, 10)  # prepare 

    APERTURE = 0.9
    JuskeshinoHardware.moveLeftGripper(APERTURE , 2.0)

    JuskeshinoHardware.moveLeftArmWithTrajectory(articular_trajectory,15)

    print("Closing gripper")
    JuskeshinoHardware.moveLeftGripper(-0.0 , 3.0)

    #PREPARE_TOP_GRIP  = [-1.25, 0.3, 0, 2.4, 0, 0.7,0]
    #print("lift object")
    #JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_TOP_GRIP, 10)

if __name__ == '__main__':
    main()