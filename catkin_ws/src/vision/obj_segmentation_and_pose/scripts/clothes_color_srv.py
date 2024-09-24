#!/usr/bin/env python3

import rospy
import ros_numpy
import cv2
import numpy as np
import tf
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped, Point
from vision_msgs.srv import *



"""
    Nodo para detectar el color de las prendas de una persona, recibe un mensaje tipo
    HumanPoseEstimatorResult y regresa el color que esta mayoritariamente presente en
    la prenda.
"""


def point_actual2point_target(pointxyz, f_actual, f_target):
    global pose_in, listener
    # Empaqueta msg, convierte orientacion de frame 'realsense_link' a frame 'base_link'
    ps_msg = PointStamped()
    ps_msg.header.frame_id = f_actual   # frame de origen
    ps_msg.header.stamp = rospy.Time(0)  # la ultima transformacion
    ps_msg.point = pointxyz
    new_ps = listener.transformPoint(f_target, ps_msg)
    new_pointxyz = new_ps.point
    return new_pointxyz



def transformPointCloud2(pc):
    #sensor_frame = 'realsense_link'
    sensor_frame = 'head_rgbd_sensor_link'
    base_link = 'base_link'
    # tranformar la nube de puntos al sistema  "base_link'
    x_arr = pc['x']
    y_arr = pc['y']
    z_arr = pc['z']

    i,j = 0,0
    p = Point()
    for i in range(480):
        for j in range(640):
            if np.isnan(x_arr[i,j]) or np.isnan(y_arr[i,j]) or np.isnan(z_arr[i,j]): 'print("punto nan")'
            else: 
                p.x, p.y, p.z = x_arr[i,j], y_arr[i,j] , z_arr[i,j]
                new_frame_p = point_actual2point_target(p, sensor_frame, base_link)
                x_arr[i,j], y_arr[i,j] , z_arr[i,j] = new_frame_p.x, new_frame_p.y, new_frame_p.z

    new_pc = cv2.merge((np.asarray(x_arr),np.asarray(y_arr),np.asarray(z_arr)))

    pc['x'] = x_arr
    pc['y'] = y_arr
    pc['z'] = z_arr

    return new_pc  



def clothes_color(hsv):
    h = hsv[0]
    s = hsv[1]
    v = hsv[2]

    """
    color = ['red','orange','yellow','green','blue','purple']
    h // 180
    s // 255
    v // 255
    """
    
    # de negro a blanco
    
    if (s <= 5):
        if (s < 11):
            if (v >= 0 ) and (v < 27) : color = 'black'
            if (h >= 27 ) and (v < 92) : color = 'gray'
            else: color = 'white'
        return color

    else:
    
        if (h >= 0 ) and (h < 10) : color = 'red'
            
        if (h >= 10) and (h < 25): color = 'orange'
            
        if (h >= 25) and (h < 34): color = 'yellow'
                        
        if (h >= 34 ) and (h < 70): color = 'green'
            
        if (h >= 70) and (h < 125): color = 'blue'
            
        if (h >= 125) and (h < 160): color = 'purple'

        if (h >= 160 ) and (h < 180) : color = 'red'
                
    return color
        


def color_histogram(img_bgr, mask):
    # Obtener histograma de color de la imagen para cada canal
    img_hsv = cv2.cvtColor(img_bgr,cv2.COLOR_BGR2HSV)
    Hh = cv2.calcHist([img_hsv],[0], mask ,[180],[0,180])
    Sh = cv2.calcHist([img_hsv],[1], mask ,[256],[0,256])
    Vh = cv2.calcHist([img_hsv],[2], mask ,[256],[0,256])

    #print("Hh, Sh, Vh", Hh, Sh, Vh)
    h = Hh.flatten().tolist() 
    s = Sh.flatten().tolist() 
    v = Vh.flatten().tolist()
    #print("h,s,v mayor valor", h,s,v)
    maxh = max(h)  #lista
    maxs = max(s)  #lista
    maxv = max(v)  #list
    
    H = h.index(maxh)   #lista
    S = s.index(maxs)   #lista
    V = v.index(maxv)   #lista
    #print("indice de mayor valor", H,S,V)

    return [H,S,V]



def callback(req):
    global listener , pe_srv, pubHumanPoseEnable


    #msg.coordinates_array.coordinates_array[0])#coordinates_array[0].keypoints_array[2].keypoint_coordinates.position)
    # Extrayendo valores rgb de la nube de puntos
    rgb_arr = arr_pc['rgb'].copy()
    rgb_arr.dtype = np.uint32
    # unir los canales
    r,g,b = ((rgb_arr >> 16) & 255), ((rgb_arr >> 8) & 255), (rgb_arr & 255)
    img_bgr = cv2.merge((np.asarray(b,dtype='uint8'),np.asarray(g,dtype='uint8'),np.asarray(r,dtype='uint8')))    
    # Extrayendo valores XYZ de la nube de puntos
    copy_pc_array = arr_pc.copy()
    # nube con respecto a base link **********************************************************************
    new_pc = transformPointCloud2(copy_pc_array) 
    #*************************************************************************************
    # Asigna limites de distancia en x,yz para segmentar el torso y pierna***************
    # Hombros
    keypoint1 = Point()
    keypoint1 = guy.keypoints_array[2].keypoint_coordinates.position
    keypoint2 = Point()
    keypoint2 = guy.keypoints_array[5].keypoint_coordinates.position
    # codo
    keypoint3 = Point()
    keypoint3 = guy.keypoints_array[3].keypoint_coordinates.position  

    # rodillas
    
    keypoint4 = Point() #l_knee
    keypoint4 = guy.keypoints_array[12].keypoint_coordinates.position  

    point_ra     = point_actual2point_target(keypoint1, sensor_frame ,base_link)
    point_la     = point_actual2point_target(keypoint2, sensor_frame ,base_link)
    point_r_elb  = point_actual2point_target(keypoint3, sensor_frame ,base_link)
    #point_l_knee = point_actual2point_target(keypoint4, sensor_frame ,base_link)

    # TORSO
    # obteniendo rangos a partir del servicio pose_estimator
    # Hombro derecho - hombro izquierdo..............................yr-yl
    # profundidad, medida experimental     
    x1_min = point_ra.x - 0.2
    x1_max = point_ra.x + 0.2
    # de hombro derecho a izquierdo
    y1_min = point_la.y
    y1_max = point_ra.y
    # altura de hombro a codo
    z1_min = point_r_elb.z  
    z1_max = point_ra.z

    # PANTS 
    
    x2_min = point_l_knee.x 
    x2_max = point_l_knee.x + 0.09
    # derecha = max,   izquierdo = min
    y2_min = point_l_knee.y - 0.04
    y2_max = point_l_knee.y + 0.04
    # altura de rodilla + 0,1 m
    z2_min = point_l_knee.z  
    z2_max = point_l_knee.z + 0.2
    

    min_valor_torso = np.array([x1_min, y1_min, z1_min]) 
    max_valor_torso = np.array([x1_max, y1_max, z1_max])

    
    min_valor_leg_l = np.array([x2_min, y2_min, z2_min]) 
    max_valor_leg_l = np.array([x2_max, y2_max, z2_max])
    

    # rango de la matriz de coordenadas
    torso_mask = cv2.inRange(new_pc , min_valor_torso, max_valor_torso)

    #leg_l_mask = cv2.inRange(new_pc , min_valor_leg_l , max_valor_leg_l)

    img_shirt = cv2.bitwise_and(img_bgr,img_bgr, mask=torso_mask)
    #img_pants = cv2.bitwise_and(img_bgr,img_bgr, mask=leg_l_mask)
    #cv2.imshow("shirt", img_shirt)
    #cv2.waitKey(0) 
    #cv2.imshow("leg ", img_pants )
    #cv2.waitKey(0)
    print("*************************************")
    hsv_shirt = color_histogram(img_bgr, torso_mask)
    #hsv_pants = color_histogram(img_bgr, leg_l_mask)
    print("hist: ", hsv_shirt)

    color_shirt = clothes_color(hsv_shirt)
    #color_pants = clothes_color(hsv_pants)

    print("color shirt", color_shirt)
    #print("color pants", color_pants)

    #color_clothes = [hsv_shirt, hsv_pants]
    """
    resp = FindPersonResponse()
    #resp.person.shirt = color_shirt
    #resp.person.pants = 'color_pants'
    
    
    return resp
    

def main():
    global pe_srv, listener, pubHumanPoseEnable
    print("Node that detects the color of a human's clothing.......(๑╹ω╹๑ )")
    rospy.init_node("clothes_color_srv")

    pubHumanPoseEnable = rospy.Publisher("/vision/human_pose/enable" , bool, queue_size=1)
    #rospy.Service("/vision/obj_segmentation_and_pose/clothes_color", FindPerson, callback)     
    #listener = tf.TransformListener()
    loop = rospy.Rate(10)
    while not rospy.is_shutdown():

        loop.sleep()

if __name__ == '__main__':
    main()