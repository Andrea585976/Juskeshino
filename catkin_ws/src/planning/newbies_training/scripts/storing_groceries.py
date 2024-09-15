#!/usr/bin/env python3
import rospy
import yaml
import numpy
import smach
import time
import smach_ros
from juskeshino_tools.JuskeshinoNavigation import JuskeshinoNavigation
from juskeshino_tools.JuskeshinoHardware import JuskeshinoHardware
from juskeshino_tools.JuskeshinoSimpleTasks import JuskeshinoSimpleTasks
from juskeshino_tools.JuskeshinoVision import JuskeshinoVision
from juskeshino_tools.JuskeshinoHRI import JuskeshinoHRI
from juskeshino_tools.JuskeshinoManipulation import JuskeshinoManipulation
from juskeshino_tools.JuskeshinoKnowledge import JuskeshinoKnowledge
from sensor_msgs.msg import LaserScan
#TO DO :
# microphone
# table height

DESK = "table"
CABINET="scan_cabinet"
#"scan_cabinet"
#"table"
TOP_SHELF=[1.28, 0.04, 0.0, 2.1 , 0.0, -1.4, 0.0]
MIDDLE_SHELF=[1.28, 0.0, 0.0, 1.55 , 0.0, -1.2, 0.0]
LOW_SHELF=[0.31, 0.1, -0.1, 0.35, 0.0, -0.9, 0.0]
PREPARE_GRIP  = [-0.69, 0.2, 0, 1.55, 0, 1.16, 0]
HOME=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
HOLD_OBJ = [0.38, 0.19, -0.01, 1.57, 0 , 0.25, 0.0 ]
GET_CLOSE_TO_TABLE = 0.48
#0.48
GET_CLOSE_TO_CABINET=0.5
#0.5

TABLE_TORSO_HEIGHT = 0.15

def matching_objects(obj):
    obj_yaml =rospy.get_param("~categories")
    try:
        f = open(obj_yaml,'r')
        data = yaml.safe_load(f)
    except:
        data = {}
        print("KnownLocations.->Cannot load locations from file " + obj_yaml)
    for category, items in data.items():
        if obj.lower() in [item.lower() for item in items]:
            rospy.loginfo(f"--->>> Detected {obj} matches an item in the '{category}' category.")
            return category  
    return False

class WaitForTheDoor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succed','failed', 'kill'])
        self.tries = 0

    def execute(self,userdata):
        self.tries += 1
        
        # CONTINUE TRYING OR KILL
        if self.tries<30:
            rospy.logwarn('\n--> STATE 1 <: Wait for the door to be opened')
            rospy.sleep(3) # seconds
        else:
            JuskeshinoHRI.say("I have waited too long for the door to open. I'm shutting down")
            return "kill"
        
        # READ AND DECIDE
        msg = rospy.wait_for_message('/hardware/scan', LaserScan)
        door_threshold = numpy.mean(msg.ranges[(int)(len(msg.ranges)*0.5 - 10):(int)(len(msg.ranges)*0.5 + 10)]) 
        
        if door_threshold < 1.0:
            print(f"Door is closed. Threshold: {door_threshold}")
            JuskeshinoHRI.say("The door is closed")
            return 'failed'
        
        JuskeshinoHRI.say("The door is open")
        rospy.sleep(2.5)
        JuskeshinoNavigation.moveDist(2, timeout=10)

        return 'succed'

class NavigateToTable(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succed', 'kill'])
        self.tries = 0

    def execute(self,userdata):
        self.tries += 1

        if self.tries<20:
            rospy.logwarn('\n--> STATE 2 <: Reaching the table')
            JuskeshinoHRI.say(" I am moving to the table")
            JuskeshinoHardware.moveHead(0,-1, 5) # HEAD TILTED DOWN, NO PAN
            JuskeshinoNavigation.getClose(DESK, 120) # REACH DESK
            JuskeshinoNavigation.moveDist(GET_CLOSE_TO_TABLE, timeout=5) # GET CLOSE TO THE TABLE
            JuskeshinoHRI.say("I have reached the table. I will start to analyze the table.")
            return 'succed'
        
        else:
            JuskeshinoHRI.say("I did too many attempts to reach the table. I'm shutting down")
            return 'kill'

class RecognizeObjects(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['succed', 'failed','tries'],
                            output_keys=['object_output','object'])
        self.tries = 0

        # The input and output data of a state is called 'userdata'
    def execute(self,userdata):
        self.tries += 1
        
        # CONTINUE ATTEMPTS OR KILL
        if self.tries < 20:
            rospy.logwarn('\n--> STATE 4 <: Recognizing objects')
            JuskeshinoHRI.say("I will start to recognize the objects in the table.")
            JuskeshinoHardware.moveHead(0,-0.8, 5)
            rospy.sleep(4)
            recog_objects, img = JuskeshinoVision.detectAndRecognizeObjects()

            if recog_objects is not None:        
                for obj in recog_objects:
                    obj=recog_objects [0]
                    print(f"Object of interest is {obj.id}")
                    x,y,z = obj.pose.position.x, obj.pose.position.y, obj.pose.position.z
                    x,y,z = JuskeshinoSimpleTasks.transformPoint(x,y,z, "shoulders_left_link", "base_link")
                    print(f"The centroid of {obj.id} is x:{x} y:{y} z:{z}")
                    obj_oriented = JuskeshinoVision.getObjectOrientation(obj)
                    userdata.object_output=[x,y,z]
                    userdata.object=obj_oriented
                    print('Object found: ', obj_oriented.id, obj_oriented.pose, obj_oriented.object_state, obj_oriented.size, obj_oriented.graspable)
                    return 'succed'
            
            JuskeshinoHRI.say('I could not detect objects, I will try again')
            return 'tries'    
        
        print('After a lot of tries, I could not detect objects')
        return 'failed'
    
class Classification(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['succed', 'failed'],
                            input_keys=['object_output','object'])
        self.tries = 0

    # The input and output data of a state is called 'userdata'
    def execute(self,userdata):
        self.tries += 1
        if self.tries < 20:    
            rospy.logwarn('\n--> STATE 5 <: Matching object with its classification')
            obj=userdata.object
            category=matching_objects(obj.id)
            if category:
                prompt = "I found" + obj.id + " which is part of the category " + category
                JuskeshinoHRI.say(prompt)
                return'succed'
        return 'failed'
    
class AlignWithObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['succed', 'failed'],
                            input_keys=['object_output','object'])
        self.tries = 0

        # The input and output data of a state is called 'userdata'
    def execute(self,userdata):
        self.tries += 1
        if self.tries < 10:
            rospy.logwarn('\n--> STATE 6 <: I am going to align with the object')
            obj=userdata.object
            JuskeshinoHRI.say("I am ready to pick the ",obj.id)
            JuskeshinoSimpleTasks.handling_location_la(obj.pose.position)
            return'succed'
        return 'failed'

class GiveMeDObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succed', 'failed']
                             ,input_keys=['object'])
        
        # The input and output data of a state is called 'userdata'
    def execute(self,userdata):
        rospy.logwarn('\n--> STATE <: GIVE ME THE OBJECT')
        obj=userdata.object
        JuskeshinoHRI.say((f"Please, help me. Put the {obj.id} in my hand and wait for the gripper closes "))
        JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_GRIP, 10)
        JuskeshinoHRI.say("I am ready to pick the ",obj.id )
        rospy.sleep(2)
        success=JuskeshinoManipulation.dynamic_grasp_left_arm()
        rospy.sleep(0.5)
        if success:
            JuskeshinoHRI.say("Thank you")
            return 'succed'
        return 'failed'

class GraspObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['succed', 'failed', 'help'],
                            input_keys=['object_output','object'])
        self.tries = 0

        # The input and output data of a state is called 'userdata'
    def execute(self,userdata):
        self.tries += 1
        if self.tries <6:
            rospy.logwarn('\n--> STATE 7 <: Picking up the target object, attempt: ' + str(self.tries))
            # GET OBJECT LOCATION AND CALCULATE TRAJECTORY
            obj=userdata.object
            x,y,z=userdata.object_output
            #x,y,z = JuskeshinoSimpleTasks.transformPoint(x,y,z, "shoulders_left_link", "base_link")
            JuskeshinoHRI.say("I am ready to pick the "+ obj.id)
            JuskeshinoHardware.moveTorso(TABLE_TORSO_HEIGHT , timeout = 5.0)
            rospy.sleep(1)
            JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_GRIP, 10)
            JuskeshinoHardware.moveLeftGripper(0.7, 100.0)

            manip_method = rospy.get_param("~manip_method")
            if manip_method == "best":
                # EXECUTE TRAJECTORY
                [response, success] = JuskeshinoManipulation.GripLa(obj)
            else:
                response = JuskeshinoManipulation.laIk([x,y,z, 0,-1.5,0])
                if response is None or response is False:
                    success = False
                else:
                    success = True
            if success:
                JuskeshinoHRI.say("Object found correctly")
                JuskeshinoHardware.moveLeftArmWithTrajectory(response.articular_trajectory,10)
                success=JuskeshinoManipulation.dynamic_grasp_left_arm()
                rospy.sleep(5)
                JuskeshinoHardware.moveLeftArmWithTrajectory(HOLD_OBJ, 10)
                JuskeshinoHRI.say("Verifying...")
                JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_GRIP, 10)
                rospy.sleep(0.5)
                if not success:
                    JuskeshinoHRI.say("I couldn't grasp the " + obj.id )
                    return 'help'
                else:
                    JuskeshinoHRI.say("I took correctly the " + obj.id )
                    return 'succed'
                
            else:
                print("No possible poses found")
                return 'help'
            
        return 'failed'

#0,-1.5,0
class FailedGrasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['succed', 'failed', 'help'],
                            input_keys=['object_output','object'])
        self.tries = 0

        # The input and output data of a state is called 'userdata'
    def execute(self,userdata):
        self.tries += 1
        if self.tries<6:
            obj=userdata.object
            JuskeshinoHRI.say("Object found")
            [response, success] = JuskeshinoManipulation.GripLa(obj)
            JuskeshinoHardware.moveLeftGripper(0.9, 100.0)
            # JuskeshinoHardware.moveTorso(0.14 , 5.0)
            JuskeshinoHardware.moveLeftArmWithTrajectory(response.q,10)
            print("Closing gripper")
            success=JuskeshinoManipulation.dynamic_grasp_left_arm()
            print (success)
            JuskeshinoHardware.moveLeftArmWithTrajectory(HOLD_OBJ, 9)
            JuskeshinoHRI.say("Verifying...")
            JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_GRIP, 10)
            rospy.sleep(0.02) 
            # JuskeshinoHardware.moveLeftGripper(0.16 , 3.0) 
            # JuskeshinoHardware.moveLeftArmWithTrajectory(PREPARE_GRIP, 10)
            # rospy.sleep(2)
            # recog_objects, img = JuskeshinoVision.detectAndRecognizeObjects()
            # for obj in recog_objects:
            #     if obj and objc:
            #         JuskeshinoHRI.say("Please, help me. Put the obj in my hand until the gripper closes "+ obj.id)
            #         return 'failed'
            if not success:
                JuskeshinoHRI.say(f"Please, help me. Put the {obj.id} in my hand until the gripper closes ")
                
                return 'help'
            else:
                JuskeshinoHRI.say("I took correctly the "+ obj.id )
                return 'succed'
        return 'failed'
    
class TransportObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['succed', 'failed'])
        self.tries = 0

        # The input and output data of a state is called 'userdata'
    def execute(self,userdata):
        self.tries += 1
        if self.tries<20:
            rospy.logwarn('\n--> STATE 8 <: Transporting object to cabinet')
            JuskeshinoHRI.say(" I am moving to the cabinet")
            JuskeshinoNavigation.moveDist(-GET_CLOSE_TO_TABLE, 10)
            rospy.sleep(2)
            JuskeshinoNavigation.getClose(CABINET, 20)
            return 'succed'
        
class ScanCabinet(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succed', 'failed', 'tries'],
                    input_keys=['object_output','object'],
                    output_keys=['object_shelf','height'])
        self.tries = 0

    def execute(self, userdata):
        # request = segmentation_server.request_class()
        self.tries += 1
        height_d=None
        if self.tries < 4:
            rospy.logwarn('\n--> STATE 9 <: Scanning cabinet')
            JuskeshinoHRI.say("Scanning cabinet")

            # Detect
            tar_obj=userdata.object
            picked_obj_category = matching_objects(tar_obj.id)

            JuskeshinoHardware.moveHead(0,-0.3, 5)
            rospy.sleep(4)
            recog_objects, img = JuskeshinoVision.detectAndRecognizeObjects()     
            if recog_objects is not None:
                for obj in recog_objects:
                    obj_category = matching_objects(obj.id)
                    if obj_category == picked_obj_category:
                        JuskeshinoHRI.say("I found a match")
                        #JuskeshinoSimpleTasks.handling_location_la(obj.pose.position)
                        userdata.object_shelf=obj
                        if obj.pose.position.z>1.3:
                            JuskeshinoHRI.say("The object is part of the second shelf")
                            print("The object is part of the second shelf")
                            height_d='second'
                            print (height_d)
                            userdata.height=height_d
                            return 'succed'
                        elif 1.3 > obj.pose.position.z > 0.9:
                            JuskeshinoHRI.say("The object is part of the third shelf") 
                            print("The object is part of the third shelf")
                            height_d='third'
                            userdata.height=height_d
                            return 'succed'
                        elif 0.9 > obj.pose.position.z > 0.6:
                            JuskeshinoHRI.say("The object is part of the fourth shelf")
                            print("The object is part of the fourth shelf") 
                            height_d='fourth'
                            userdata.height=height_d
                            return 'succed'
            JuskeshinoHRI.say(f"I couldn't found a match for the {tar_obj.id}")
            return 'tries'
        else:
            JuskeshinoHRI.say(f"I am going to place the {tar_obj.id}")
            height_d='second'
            userdata.height=height_d
            return 'succed'
                # for obj in recog_objects:
                #     print("",obj.id)
                #     print(obj.pose.position)
                #     category1=matching_objects(obj.id)
                #     category2=matching_objects(tar_obj.id)
                #     print("",category1)
                #     print(category2)
                    
                #     for recog_obj in recog_objects:  # Iterate over all objects to find matching categories
                #         recog_category = matching_objects(recog_obj.id)
                #         if recog_category and category2 == recog_category:
                #             category1 = category2  # Set category1 to match category2
                #             obj = recog_obj  # Set obj to the matching object
                #             #centroid maths
                #             JuskeshinoSimpleTasks.handling_location_la(obj.pose.position)
                #             userdata.object_shelf=obj
                #             #Aquí en este estado hay un error por ahí, es donde se atora y solamente itera sobre el mismo objecto n lugar de continuar
                #             if obj.pose.position.z>1.3:
                #                 JuskeshinoHRI.say("The object is part of the top shelf")
                #                 print("The object is part of the top shelf")
                #                 height_d='top'
                #                 print (height_d)
                #                 userdata.height=height_d
                #                 return 'succed'
                #                 #variable shared with the next state giving q for left arm to leave the object 'object_shelf'
                #             elif 1.3 > obj.pose.position.z > 0.8:
                #                 JuskeshinoHRI.say("The object is part of the middle shelf") 
                #                 print("The object is part of the middle shelf")
                #                 height_d='middle'
                #                 userdata.height=height_d
                #                 return 'succed'
                #             elif 0.8 > obj.pose.position.z > 0.2:
                #                 JuskeshinoHRI.say("The object is part of the low shelf")
                #                 print("The object is part of the low shelf") 
                #                 height_d='low'
                #                 userdata.height=height_d
                #                 return 'succed'
                            
                #         else:
                #             #Check for the position of each object and start from that 
                #             print('Cannot match objects, I will try again...')
                #             #JuskeshinoNavigation.moveDist(0.1, 10)
                #             return 'failed'  
            
        print('I could not detect objects')
        return 'failed'
    #check the centroid of shelf obj and leave the object on the side
    #Align with shelf state

class LeaveObject(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succed', 'failed', 'tries'],
                    input_keys=['object_shelf','height'])
        self.tries = 0

    def execute(self, userdata):
        self.tries += 1
        if self.tries < 8:
            JuskeshinoHardware.moveHead(0,-0.35, 5)
            shelf=userdata.height

            if shelf == 'second':
                JuskeshinoHardware.moveTorso(0.28 , 10.0)
                JuskeshinoHardware.moveLeftArmWithTrajectory(TOP_SHELF, 10)
            if shelf =='third':
                JuskeshinoHardware.moveTorso(0.02 , 10.0)
                JuskeshinoHardware.moveLeftArmWithTrajectory(MIDDLE_SHELF, 10)
            if shelf=='fourth':
                JuskeshinoHardware.moveTorso(0.28 , 10.0)
                JuskeshinoHardware.moveLeftArmWithTrajectory(LOW_SHELF, 10)
            if shelf=='fifth':
                JuskeshinoHardware.moveTorso(0.02 , 10.0)
                JuskeshinoHardware.moveLeftArmWithTrajectory(LOW_SHELF, 10)
            rospy.sleep(1)
            JuskeshinoNavigation.moveDist(GET_CLOSE_TO_CABINET, timeout=5)
            rospy.sleep(2)
            # obj=userdata.object_shelf
            # JuskeshinoHRI.say(f"Please, help me to place the {obj.id} in the {shelf} part of the shelf. Put your hand below the object and waitn until I open the gripper.")
            # rospy.sleep(12)
            print("Opening gripper")
            JuskeshinoHardware.moveLeftGripper(0.8, 5.0)
            JuskeshinoNavigation.moveDist(-0.5, timeout=5)
            JuskeshinoHardware.moveTorso(0.04 , 5.0)
            rospy.sleep(1)
            JuskeshinoHardware.moveLeftArmWithTrajectory(HOME, 8)
            JuskeshinoHardware.moveLeftGripper(0.0, 5.0)
            return 'succed'
        return 'failed'   
            #     JuskeshinoHardware.moveLeftGripper(0.9, 100.0)
            
            # rospy.logwarn('\n--> STATE 10 <: Leaving object')
            
            # # JuskeshinoNavigation.getClose('SHELF', 10)
            # JuskeshinoHRI.say("Leaving object")
            # [response, success] = JuskeshinoManipulation.GripLa(obj_shelf)
            # if success :
            #     print("Object position: ",obj_shelf.pose.position)
            #     #JuskeshinoHardware.moveTorso(0.14 , 5.0)
            #     JuskeshinoHardware.moveLeftArmWithTrajectory(response.articular_trajectory,10)
            #     print("Opening gripper")
            #     JuskeshinoHardware.moveLeftGripper(0.9, 100.0)
            #     return 'succed'
            # return 'failed'
                       

def main():
    rospy.init_node("storing_groceries")
    rate = rospy.Rate(0.2)
    JuskeshinoNavigation.setNodeHandle()
    JuskeshinoHardware.setNodeHandle()
    JuskeshinoVision.setNodeHandle()
    JuskeshinoSimpleTasks.setNodeHandle()
    JuskeshinoManipulation.setNodeHandle()
    JuskeshinoHRI.setNodeHandle()
    JuskeshinoKnowledge.setNodeHandle()
         
    #-------------------CREATING SMACH state machine------------------------------

    rospy.loginfo('Starting state machine...')

    #   The outcome in the initialization of the state machine 'END' represents the 
    #    possible end point or result of the state machine's execution
    sm = smach.StateMachine(outcomes=['END'])

    # For viewing state machine (ROS)
    sis = smach_ros.IntrospectionServer('SMACH_VIEW_SERVER', sm, '/SM_STORING')
    sis.start()

    with sm:
        # Add states to the container
        smach.StateMachine.add('WAIT_FOR_THE_DOOR', WaitForTheDoor(), 
        transitions={'failed':'WAIT_FOR_THE_DOOR', 
                     'succed':'NAVIGATE_TO_TABLE', 
                     'kill': 'END'})

        smach.StateMachine.add('NAVIGATE_TO_TABLE', NavigateToTable(), 
        transitions={'kill':'END', 
                     'succed':'RECOGNIZE_OBJ'})

        smach.StateMachine.add('RECOGNIZE_OBJ',RecognizeObjects(), 
        transitions={'failed':'END',
                     'succed':'CLASSIFICATION', 
                     'tries':'RECOGNIZE_OBJ'})
        
        smach.StateMachine.add('CLASSIFICATION',Classification(), 
        transitions={'failed':'RECOGNIZE_OBJ',
                     'succed':'ALIGNE_WITH_OBJ'})

        smach.StateMachine.add('ALIGNE_WITH_OBJ',AlignWithObject(), 
        transitions={'failed':'TRANSPORT_OBJECT',
                     'succed':'GRASP_OBJ'})

        smach.StateMachine.add('GIVE_ME_D_OBJ',GiveMeDObject(), 
        transitions={'failed':'GIVE_ME_D_OBJ',
                     'succed':'TRANSPORT_OBJECT'})
        
        smach.StateMachine.add('GRASP_OBJ',GraspObject(), 
        transitions={'failed':'FAILED_GRASP',
                     'help': 'GIVE_ME_D_OBJ',
                     'succed':'TRANSPORT_OBJECT'})
        
        smach.StateMachine.add('FAILED_GRASP',FailedGrasp(), 
        transitions={'failed':'GIVE_ME_D_OBJ',
                     'succed':'TRANSPORT_OBJECT',
                     'help': 'GIVE_ME_D_OBJ'})
        
        smach.StateMachine.add('TRANSPORT_OBJECT',TransportObject(), 
        transitions={'failed':'NAVIGATE_TO_TABLE',
                     'succed':'SCAN_CABINET'})
  
        smach.StateMachine.add('SCAN_CABINET',ScanCabinet(), 
        transitions={'failed':'SCAN_CABINET',
                     'succed':'LEAVE_OBJ',
                     'tries':'SCAN_CABINET'})

        smach.StateMachine.add('LEAVE_OBJ',LeaveObject(), 
        transitions={'failed':'LEAVE_OBJ',
                     'succed':'NAVIGATE_TO_TABLE',
                     'tries':'LEAVE_OBJ'})
                     
    # Execute SMACH plan
    outcome = sm.execute()
    
if __name__ == "__main__":
    main()