#!/usr/bin/env python


import rospy
import time
import random
import math
import tf
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from gazebo_msgs.srv import DeleteModel, DeleteModelRequest
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose


from data import data_collection
from juskeshino_tools.JuskeshinoHardware import JuskeshinoHardware




"""
def reset_world():
   rospy.wait_for_service("/gazebo/reset_world") # Resets the model's poses
"""
# hsr_pringles_03
# hsr_orange_03
# apple_01




def object_name_decoder_simulation_yolo(object_name):
   #~/Juskeshino/catkin_ws/src/vision/obj_reco/yolo_dataset/json/YOLODataset
  
   # Hay objetos que YOLO no puede reconocer, por ejemplo: 003_cracker_box, 031 spoon
   object_name_decoded = ''
  
   if object_name == '004_sugar_box':    # El nombre del objeto en la simulacion gazebo es 004_sugar_box,
       object_name_decoded = 'jelly'      # pero YOLO lo reconoce como jelly


   if object_name == 'hsr_pringles_03':   
       object_name_decoded = 'mustard'
  
   if object_name == 'hsr_orange_03':   
       object_name_decoded = 'campbells'
  
   if object_name == '005_tomato_soup_can':   
       object_name_decoded = 'campbells'
      
   if object_name == 'apple_01':   
       object_name_decoded = 'apple'
  
   if object_name == '001_chips_can':   
       object_name_decoded = 'pringles'
  
   if object_name == '024_bowl':   
       object_name_decoded = 'red_bowl'
  
   if object_name == '029_plate':   
       object_name_decoded = 'red_dish'
  
   if object_name == '007_tuna_fish_can':   
       object_name_decoded = 'tuna_can'
  
   if object_name == '010_potted_meat_can':   
       object_name_decoded = 'meat_can'


   if object_name == '011_banana':   
       object_name_decoded = 'banana'


   if object_name == '056_tennis_ball':   
       object_name_decoded = 'soft_ball'
  
   if object_name == '033_spatula':   
       object_name_decoded = 'spatula'
  
   if object_name == '014_lemon':   
       object_name_decoded = 'orange'
  
   if object_name == '017_orange':   
       object_name_decoded = 'orange'
  
   return object_name_decoded


def object_random_orientation():
   random_degrees = random.uniform(0, 360)
   roll_radians = math.radians(random_degrees)


   random_degrees = random.uniform(0, 360)
   pitch_radians = math.radians(random_degrees)


   random_degrees = random.uniform(0, 360)
   yaw_radians = math.radians(random_degrees)


   # Return quaternion from Euler angles and axis sequence.
   q = tf.transformations.quaternion_from_euler(roll_radians, pitch_radians, yaw_radians, 'ryxz')
  
  
   random_orientation_x = q[0]
   random_orientation_y = q[1]
   random_orientation_z = q[2]
   random_orientation_w = q[3]
  
  
   print("Random quaternion orientation:", q)
   return q




def object_random_position():
   random_position[0] =random.uniform(0, 30)  # x-axis
   random_position[1] =random.uniform(0, 30)  # y-axis
   return random_position




def spawn_model(object_name):
   rospy.wait_for_service('gazebo/spawn_sdf_model')
  
   initial_pose = Pose()
   '''
   initial_pose.position.x = 7
   initial_pose.position.y = 5.8
   initial_pose.position.z = 1
   '''
   initial_pose.position.x = 7.62
   initial_pose.position.y = 5.8
   initial_pose.position.z = 0.8


   path = 'hardware/justina/gazebo_envs/models/YCB_virtual/' + object_name + '/model.sdf'
   print("PATH SPAWN: ", path)


   #f = open('hardware/justina/gazebo_envs/models/YCB_virtual/017_orange/model.sdf','r')
   f = open(path,'r')
   sdff = f.read()


   try:
       spawn_model = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
       spawn_model(object_name, sdff, "", initial_pose, "world")
       #spawn_model('004_sugar_box', sdff, "", initial_pose, "world")
   except rospy.ServiceException as e:
       rospy.logwarn("Service failed: " + str(e))






def delete_model(model_name):
   rospy.wait_for_service("gazebo/delete_model")
  
   req = DeleteModelRequest()
   req.model_name = model_name


   try:
       deleteModel = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
       resp = deleteModel(req)
   except rospy.ServiceException as e:
       rospy.logwarn("Service failed: " + str(e))




def set_model_state(model_name,
                   pose_position_x,
                   pose_position_y,
                   pose_position_z,
                   pose_orientation_x,
                   pose_orientation_y,
                   pose_orientation_z,
                   pose_orientation_w,
                   twist_linear_x,
                   twist_linear_y,
                   twist_linear_z,
                   twist_angular_x,
                   twist_angular_y,
                   twist_angular_z,
                   reference_frame):
  
   rospy.wait_for_service("/gazebo/set_model_state")


   req = SetModelStateRequest()
   req.model_state.model_name = model_name


   req.model_state.pose.position.x = pose_position_x
   req.model_state.pose.position.y = pose_position_y
   req.model_state.pose.position.z = pose_position_z


   req.model_state.pose.orientation.x = pose_orientation_x
   req.model_state.pose.orientation.y = pose_orientation_y
   req.model_state.pose.orientation.z = pose_orientation_z
   req.model_state.pose.orientation.w = pose_orientation_w


   req.model_state.twist.linear.x = twist_linear_x   # This expresses velocity in free space broken into its linear and angular parts.
   req.model_state.twist.linear.y = twist_linear_y
   req.model_state.twist.linear.z = twist_linear_z


   req.model_state.twist.angular.x = twist_angular_x
   req.model_state.twist.angular.y = twist_angular_y
   req.model_state.twist.angular.z = twist_angular_z


   req.model_state.reference_frame = reference_frame


   try:
       setModelState = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
       resp = setModelState(req)
       if resp.success:
           rospy.loginfo("Set model successful!")


   except rospy.ServiceException as e:
       rospy.logwarn("Service failed: " + str(e))




def main():
   rospy.init_node("simulacion")


   objects = ['004_sugar_box','001_chips_can', '024_bowl','005_tomato_soup_can', 
               '029_plate', '007_tuna_fish_can',
               '010_potted_meat_can', '011_banana', '056_tennis_ball', '033_spatula',
               '014_lemon','017_orange']
  
   delete_model('hsr_pringles_03')
   delete_model('apple_01')
   delete_model('hsr_orange_03')
   #delete_model('004_sugar_box')
   #spawn_model()


   #object_name = 'apple_01'


   JuskeshinoHardware.setNodeHandle()
   PREPARE_TOP_GRIP  = [-1.25, 0.3, 0, 2.4, 0, 0.7,0]
   JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_TOP_GRIP, 10)  # prepare


   for objecto in objects:
       print("nombre del objeto: ", objecto)
       object_name = objecto


       spawn_model(object_name)


          
       #position_x = 10
       position_x = 7.62
       position_y = 5.8
       position_z = 0.8


       '''
       orientation_x = -5.094971986832593e-17
       orientation_y = -5.495237021764431e-16
       orientation_z = 0.769367223624769
       orientation_w = 0.6388067588965498
      
       '''


          
       twist_linear_x = 0.0
       twist_linear_y = 0.0
       twist_linear_z = 0.0


       twist_angular_x = 0.0
       twist_angular_y = 0.0
       twist_angular_z = 0.0
          


       '''
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
       '''
          
       #time.sleep(5) # Sleep for 10 seconds


      
       for i in range(5):
           random_orientation = object_random_orientation()
          
           orientation_x = random_orientation[0]
           orientation_y = random_orientation[1]
           orientation_z = random_orientation[2]
           orientation_w = random_orientation[3]


           print("object name dentro del for: ", object_name)
          
           set_model_state(model_name = object_name,
                           pose_position_x = 7.62,
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
              
           object_name_decoded = object_name_decoder_simulation_yolo(object_name)
          
           end_data_collection = data_collection(object_name_decoded, orientation_x, orientation_y, orientation_z,orientation_w)


           print("termino recoleccion de datos: ", end_data_collection)
           print("repetitions: ", i)


           time.sleep(3) # Sleep for 10 seconds
       delete_model(object_name)
  


if __name__ == '__main__':
   main()
