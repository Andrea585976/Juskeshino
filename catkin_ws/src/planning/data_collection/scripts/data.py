#!/usr/bin/env python


import rospy
import ros_numpy
import numpy as np
import yaml
import csv
import datetime


from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64MultiArray
from vision_msgs.srv import RecognizeObject, RecognizeObjectRequest
from manip_msgs.srv import BestGraspTraj, BestGraspTrajRequest


from juskeshino_tools.JuskeshinoHardware import JuskeshinoHardware




# Data collection
data ={
   "object_name": "",
   "object_category": "",
   "object_state": "",
  
   "object_position_x": "",
   "object_position_y": "",
   "object_position_z": "",


   "object_orientation_x": "",
   "object_orientation_y": "",
   "object_orientation_z": "",
   "object_orientation_w": "",
  
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


   "articular_trajectory_1": "",
   "articular_trajectory_2": "",
   "articular_trajectory_3": "",
   "articular_trajectory_4": "",
   "articular_trajectory_5": "",
   "articular_trajectory_6": "",
   "articular_trajectory_7": "",
}


def append_dict_as_row(file_name, dict_of_elem, field_names):
   # Open file in append mode
   with open(file_name, 'a+', newline='') as write_obj:
       # Create a writer object from csv module
       dict_writer = csv.DictWriter(write_obj, fieldnames=field_names)
       # Add dictionary as wor in the csv
       dict_writer.writerow(dict_of_elem)




def detect_and_recognize_object(name):
   rospy.wait_for_service('/vision/obj_reco/detect_and_recognize_object')
  
   req = RecognizeObjectRequest()
   req.name = name
   req.point_cloud = rospy.wait_for_message("/camera/depth_registered/points", PointCloud2, timeout=1.0)
  
   try:   
       detectAndRecognizeObject = rospy.ServiceProxy('/vision/obj_reco/detect_and_recognize_object', RecognizeObject) # Detect and recognize object from from a base link reference
       #print("hola")
       #print("type req: ",type(req))
       resp = detectAndRecognizeObject(req)
       #print("type: ",type(resp))
       #print("despues hola")
      
       if resp is None:
           print("No se pudo reconocer el objeto")
           return
       else:
           return resp.recog_object, resp.image
      


       """
           resp.recog_object --> type VisionObject.msg
           resp.image --> type Image
       """
   except rospy.ServiceException as e:
       rospy.logwarn("Service failed: " + str(e))
       #detect_and_recognize_object(name)
       return None, None


"""
VisionObject.msg


std_msgs/Header header
string id                                    #name of identifying the object (milk, orange juice, etc) or color (green,blue, etc)
string category                              #object type (drink, snack, etc)
float32 confidence                           #value in [0,1] indicating the probability of a correct identification
                                            #in the case of color objects indiates the grasping priority
sensor_msgs/Image image              #image in rgb of object;
sensor_msgs/Image obj_mask            #binary image of object
sensor_msgs/PointCloud2 point_cloud          #Point cloud (probably, colored) of the object
geometry_msgs/Vector3 size                   #Size in meters: size in x, y and z
geometry_msgs/Pose pose                      #Centroid and orientation
geometry_msgs/Vector3[] bounding_box         #Two vectors indicating the bounding box
geometry_msgs/Vector3[] bounding_polygon     #N vectors
int32 x                      #top left x
int32 y                      #top left y
int32 width                  #top widht
int32 height                     #top height
bool graspable                               #graspable by the gripper
std_msgs/ColorRGBA color_rgba                #Mean object's color
float32 alfa                     #angle of the largest dimension of the object with respect to the surface
string object_state              #inclination of the collinear axis to the longest dimension of the object
"""




"""
RecognizeObject.srv


int32 iterations
sensor_msgs/PointCloud2 point_cloud  #If recognition is made only with RGB image, this is empty
sensor_msgs/Image image              #If recognition is made with PointCloud, this is empty
string name                          #Requested object's name
sensor_msgs/Image obj_mask        #binary image of object
---
vision_msgs/VisionObject recog_object
sensor_msgs/Image image              #Althoug every object has its own point_cloud and image, this field
                                    #is intended to show the recognized object in the original image.
"""


def get_object_pose_with_orientation(object_resp): #vision object
   rospy.wait_for_service('/vision/obj_segmentation/get_obj_pose_with_orientation')
  
   req = RecognizeObjectRequest()
   req.name = object_resp.id
   req.point_cloud = object_resp.point_cloud
   req.obj_mask = object_resp.obj_mask


  
   try:   
       getObjectPoseWithOrientation = rospy.ServiceProxy('/vision/obj_segmentation/get_obj_pose_with_orientation', RecognizeObject)
       resp = getObjectPoseWithOrientation(req)
       return resp.recog_object, resp.image
       """
           resp.recog_object --> type VisionObject.msg
           resp.image --> type Image
       """
   except rospy.ServiceException as e:
       rospy.logwarn("Service failed: " + str(e))






def get_best_grasp_trajectory(name):
   rospy.wait_for_service('/manipulation/get_best_grasp_traj')


   req = BestGraspTrajRequest()
   req.recog_object = name


   try:
       getBestGraspTrayectory = rospy.ServiceProxy("/manipulation/get_best_grasp_traj", BestGraspTraj)
       resp = getBestGraspTrayectory(req)
       return resp
       """
           resp --> type BestGRaspTraj
       """


   except rospy.ServiceException as e:
       rospy.logwarn("Service failed: " + str(e))




"""
BestGRaspTraj.srv
(request)
vision_msgs/VisionObject recog_object                   
---
(response)
trajectory_msgs/JointTrajectory articular_trajectory
bool graspable                               #graspable by the gripper
float64 x_gripper          
float64 y_gripper
float64 z_gripper
float64 roll_gripper
float64 pitch_gripper
float64 yaw_gripper
float64[] q # Send generic trajectory


"""
"""
vision_msgs/VisionObject recog_object                   
---
trajectory_msgs/JointTrajectory articular_trajectory
bool graspable                               #graspable by the gripper
geometry_msgs/PoseStamped pose_stamped                      #Centroid and orientation
float64[] q # Send generic trajectory
"""


def get_head_current_pose():
   current_pose = rospy.wait_for_message("/hardware/head/current_pose", Float64MultiArray, timeout=1.0)
   return current_pose












def data_collection(objectName, orientation_x, orientation_y, orientation_z, orientation_w):
   data_collection_finish = False
   object_name = objectName
   n = 0
   move_head = -0.4


   JuskeshinoHardware.setNodeHandle()


   '''
   PREPARE_TOP_GRIP  = [-1.25, 0.3, 0, 2.4, 0, 0.7,0]
   JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_TOP_GRIP, 10)  # prepare
   '''
  
   #rate = rospy.Rate(10)
   #while not rospy.is_shutdown():
   while(n < 9):  # Repite 9 veces
       print("n:", n)


       # Move head
       print("move_head: ", move_head)
      
       JuskeshinoHardware.moveHead(move_head, -1, 5) # Mueve hacia la izquierda
      


       # Detect and recognize object
       recog_object, image = detect_and_recognize_object(object_name)
       #print(recog_object.category) #no aparece
       if recog_object is None and image is None:
           print("No sis Nonee pudo reconocer el objeto")
           n = n + 1
           move_head = move_head + 0.1
           continue
       else:
           # Save data
           data['object_name'] = recog_object.id
          
           data['object_position_x'] = recog_object.pose.position.x
           data['object_position_y'] = recog_object.pose.position.y
           data['object_position_z'] = recog_object.pose.position.z


           '''
           data['object_orientation_x'] = recog_object.pose.orientation.x
           data['object_orientation_y'] = recog_object.pose.orientation.y
           data['object_orientation_z'] = recog_object.pose.orientation.z
           data['object_orientation_w'] = recog_object.pose.orientation.z
           '''


           recog_object_orientated, image_orientated = get_object_pose_with_orientation(recog_object)
           print("category", recog_object_orientated.category)
           print("state", recog_object_orientated.object_state)


           '''
           data['object_orientation_x'] = recog_object_orientated.pose.orientation.x
           data['object_orientation_y'] = recog_object_orientated.pose.orientation.y
           data['object_orientation_z'] = recog_object_orientated.pose.orientation.z
           data['object_orientation_w'] = recog_object_orientated.pose.orientation.w
           '''


           data['object_orientation_x'] = orientation_x
           data['object_orientation_y'] = orientation_y
           data['object_orientation_z'] = orientation_z
           data['object_orientation_w'] = orientation_w
          
           data['object_category'] = recog_object_orientated.category
           data['object_state'] = recog_object_orientated.object_state
          




           # Get current head pose
           current_head_pose = get_head_current_pose()
           """
           std_msgs/Float64MultiArray Message
               MultiArrayLayout  layout        # specification of data layout
               float64[]         data          # array of data


           """
           data['head_pose_pan'] =  current_head_pose.data[0]
           data['head_pose_tilt'] =  current_head_pose.data[1]




           # Generate best grasp trajectory
           grasp = get_best_grasp_trajectory(recog_object_orientated)
           print("x gripper:", grasp.pose_stamped.pose.position.x)
           print("y gripper:", grasp.pose_stamped.pose.position.y)
           print("w gripper:", grasp.pose_stamped.pose.orientation.w)
           #print("articular trajectory:", grasp.articular_trajectory.points[-1].positions)


           if not grasp.articular_trajectory.points: #la lista en vacía
               print("No se encontro pose")
               n = n + 1
               move_head = move_head + 0.1
               continue
           else:
               # Save data
               data['gripper_position_x'] = grasp.pose_stamped.pose.position.x
               data['gripper_position_y'] = grasp.pose_stamped.pose.position.y
               data['gripper_position_z'] = grasp.pose_stamped.pose.position.z
              
               data['gripper_orientation_x'] = grasp.pose_stamped.pose.orientation.x
               data['gripper_orientation_y'] = grasp.pose_stamped.pose.orientation.y
               data['gripper_orientation_z'] = grasp.pose_stamped.pose.orientation.z
               data['gripper_orientation_w'] = grasp.pose_stamped.pose.orientation.w


               data['articular_trajectory_1'] = grasp.articular_trajectory.points[-1].positions[0]
               data['articular_trajectory_2'] = grasp.articular_trajectory.points[-1].positions[1]
               data['articular_trajectory_3'] = grasp.articular_trajectory.points[-1].positions[2]
               data['articular_trajectory_4'] = grasp.articular_trajectory.points[-1].positions[3]
               data['articular_trajectory_5'] = grasp.articular_trajectory.points[-1].positions[4]
               data['articular_trajectory_6'] = grasp.articular_trajectory.points[-1].positions[5]
               data['articular_trajectory_7'] = grasp.articular_trajectory.points[-1].positions[6]




               # ----------------------------------- Save Point Cloud ---------------------------------
               objectPointCloud = recog_object.point_cloud
               numpy_arr_objectPointCloud = ros_numpy.point_cloud2.pointcloud2_to_array(objectPointCloud)
              
               hora_actual = datetime.datetime.now()
               print(hora_actual)
              
               filename = 'planning/data_collection/scripts/object_point_cloud/' + object_name + '_' + str(hora_actual) + '.npz'
               #filename = object_name + '_' + str(hora_actual) + '.npz'
               print(filename)


               # Save the arrays to a npz file
               np.savez(filename, numpy_arr_objectPointCloud)


               # Load the arrays from the file
               loaded_arrays = np.load(filename)
               print("Point cloud from npz file:", loaded_arrays['arr_0'])


               # Save local-path file
               #data['object_point_cloud']="/" + filename
               data['object_point_cloud'] = filename


               # -----------------------------------------------------------------------------------


               print(data)


               # Store data in a yaml file
               """
               with open('data.yaml', 'w') as file:
                   yaml.dump(data, file)
               """


               # Store data in a csv file
               '''
               #Para poner header. Hacerlo la primera vez.
               with open('planning/data_collection/scripts/data.csv', 'w', newline='') as f_output:
                   csv_output = csv.DictWriter(f_output, fieldnames=data.keys())
                   csv_output.writeheader()
               '''

               """
               with open('data.csv', 'w', newline='') as f_output:
                   csv_output = csv.DictWriter(f_output, fieldnames=data.keys())
                   csv_output.writeheader()
                   csv_output.writerow(data)
               """
               append_dict_as_row('planning/data_collection/scripts/data.csv', data, data.keys())
               # append_dict_as_row('data.csv', data, data.keys())
                   #    rate.sleep()
               move_head = move_head + 0.1
               n = n + 1
   data_collection_finish = True
   return data_collection_finish


'''   
if __name__ == '__main__':
   #main()
   rospy.init_node("data")
   flag = data_collection('apple')
   print("Flag data collection finish: ", flag)
'''
