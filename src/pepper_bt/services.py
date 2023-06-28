#!/usr/bin/env python
import qi
import argparse
import sys
import time

from naoqi import ALProxy

import rospy
# from naoqi_bridge_msgs.msg import PeoplePerceptionPeopleList, PeoplePerceptionPeopleDetected

import subprocess


class Pepper():

    def __init__(self, ip_address, port=9559):
        parser = argparse.ArgumentParser()
        parser.add_argument("--ip", type=str, default=ip_address,
                            help="Robot IP address. On robot or Local Naoqi: use %s." % ip_address)
        parser.add_argument("--port", type=int, default=port,
                            help="Naoqi port number")
        args = parser.parse_args()       
        self.session = qi.Session()
        self.connection_url = "tcp://" + args.ip + ":" + str(args.port)
        
        try:
            self.session.connect("tcp://" + args.ip + ":" + str(args.port))

            self.tabletService = self.session.service("ALTabletService")
            self.motion_service = self.session.service("ALMotion")
            self.face_characteristic = self.session.service("ALFaceCharacteristics")
            self.speaking_movement = self.session.service("ALSpeakingMovement")
        except RuntimeError:
            print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
                "Please check your script arguments. Run with -h option for help.")
            sys.exit(1)      


    def tablet_show_image(self,img_url):
        try:
            self.tabletService.enableWifi()
            #tabletService.playVideo("http://clips.vorwaerts-gmbh.de/big_buck_bunny.mp4")
            self.tabletService.showImage(img_url)
            #tabletService.showImage(img_url)
            #time.sleep(5)
            #tabletService.hideImage()
        except Exception as e:
            print("Error is %s" % e)

    
    def start_recognizing_people(self):
        try:
            app = qi.Application(["HumanGreeter", "--qi-url=" + self.connection_url])
        except RuntimeError:
            print ("HumanGreeter connection has error to connect")
            sys.exit(1)
         
        return HumanGreeter(app)
        
        
    def move_head_down(self):
        """Look down"""
        self.motion_service.setAngles("HeadPitch", 0.46, 0.2)
    
    def move_head_up(self):
        """Look up"""
        self.motion_service.setAngles("HeadPitch", -0.4, 0.2)

    def move_head_default(self):
        """Put head into default position in 'StandInit' pose"""
        self.motion_service.setAngles("HeadPitch", 0.0, 0.2)

    @staticmethod
    def share_localhost(folder):
        """
        Shares a location on localhost via HTTPS to Pepper be
        able to reach it by subscribing to IP address of this
        computer.

        :Example:

        >>> pepper.share_localhost("/Users/pepper/Desktop/web/")

        :param folder: Root folder to share
        :type folder: string
        """
        # TODO: Add some elegant method to kill a port if previously opened
        subprocess.Popen(["cd", folder])
        try:
            subprocess.Popen(["python", "-m", "SimpleHTTPServer"])
        except Exception as error:
            subprocess.Popen(["python", "-m", "SimpleHTTPServer"])
        print("[INFO]: HTTPS server successfully started")




class HumanGreeter(object):

    def __init__(self, app):
        
        super(HumanGreeter, self).__init__()
        app.start()
        session = app.session

        self.memory = session.service("ALMemory")
        self.gaze_analysis = session.service("ALGazeAnalysis")
        self.people_perception = session.service("ALPeoplePerception")
        self.posture_service   = session.service("ALRobotPosture")
        self.motion            = session.service("ALMotion")
        self.awareness_service = session.service("ALBasicAwareness")

        # Get the services ALTextToSpeech and ALFaceDetection.
        self.tts = session.service("ALTextToSpeech")
        self.face_detection = session.service("ALFaceDetection")
        self.face_detection.subscribe("HumanGreeter")
        # # Enable or disable tracking.
        # self.face_detection.enableTracking(False)
        self.subscribers = []
        self.got_face = False
        #self.people_perception.resetPopulation()
        #print("time: ",self.people_perception.getTimeBeforeVisiblePersonDisappears())
        #print("time: ",self.people_perception.getTimeBeforePersonDisappears())
        self.callback = None
        self.stop_run = False
     
    def set_callback(self, callback):
        self.callback = callback

    def trigger_callback(self, value):
        if self.callback:
            self.callback(value)  


    def on_human_tracked(self, value):
        self.trigger_callback("Trigggggggggggggggggger !!!!!!!!!!")
        if value == []:  # empty value when the face disappears
            print("empty")
            self.got_face = False
        elif not self.got_face:  # only speak the first time a face appears
            self.got_face = True
            print("I saw a face!")
            self.tts.say("Hello, you!")
            # First Field = TimeStamp.
            timeStamp = value[0]
            print("TimeStamp is: " + str(timeStamp))

            # Second Field = array of face_Info's.
            faceInfoArray = value[1]
            for j in range( len(faceInfoArray)-1 ):
                faceInfo = faceInfoArray[j]

                # First Field = Shape info.
                faceShapeInfo = faceInfo[0]

                # Second Field = Extra info (empty for now).
                faceExtraInfo = faceInfo[1]

                # print("Face Infos :  alpha %.3f - beta %.3f \n" % (faceShapeInfo[1], faceShapeInfo[2]))
                # print("Face Infos :  width %.3f - height %.3f \n" % (faceShapeInfo[3], faceShapeInfo[4]))
                # print("Face Extra Infos :" + str(faceExtraInfo))

    def on_person_looks_at_robot(self, data):
        print("on_person_looks_at_robot")
        print(data)
        # event_name = data[0]
        # person_id = data[1]
        # subscriber_identifier = data[2]
       
        # print("event_name: ", event_name)
        # print("person_id: ", person_id)
        # print("subscriber_identifier: ", subscriber_identifier)

    def on_person_starts_looking_at_robot(self,data):
        print("on_person_starts_looking_at_robot")
        print(data)

    def on_person_stop_looking_at_robot(self,data):
        print("on_person_stop_looking_at_robot")
        print(data)

 
    def move_forward_event(self, value):
        print("Got event Move Forward : ",str(value))

        try:
            self._moveForward(value)
            self.motion_service.stopMove()
        except KeyboardInterrupt:
            print( "KeyBoard Interrupt initiated")
            self.motion_service.stopMove()
            exit()
        except Exception as errorMsg:
            print(str(errorMsg))
            print("This example is not allowed on this robot.")
            exit()

    
    def _moveForward(self, amnt):
        #TARGET VELOCITY
        X = 0.5  # forward
        Y = 0.0
        Theta = 0.0

        try:
            self.motion_service.moveToward(X, Y, Theta)
        except KeyboardInterrupt:
            print("KeyBoard Interrupt initiated")
            self.motion_service.stopMove()
            exit()
        except Exception as errorMsg:
            print(str(errorMsg))
            print("This example is not allowed on this robot.")
            exit()

        print("=====================================================================")
        print( "Forward Movement Started")
        time.sleep(float(amnt))
        print("Forward Movement Complete")
        print("=====================================================================")

        
    def on_people_list(self,data):
        print("people list")
        print(data)


    def on_people_detected(self, data):
        print("on people detected")
        print(data)

    def on_non_visible_people_list(self,data):
        print("on_non_visible_people_list")
        print(data)

    def on_just_left(self,data):
        print("on_just_left")
        print(data)


    def run(self):
        # start
        # print("Waiting for the robot to be in wake up position")
        # self.motion.wakeUp()
        # self.posture_service.goToPosture("StandInit", 0.5)

        print("Starting HumanGreeter")
        self.create_callbacks()
        # self.set_awareness(True)

        try:
            while not self.stop_run:
                time.sleep(.5)
        except KeyboardInterrupt:
            print("Interrupted by user, stopping HumanGreeter")
            #self.face_detection.unsubscribe("HumanGreeter")
            #stop
            sys.exit(0)

    def stop(self):
         self.face_detection.unsubscribe("HumanGreeter")
         self.stop_run = True
        #  for subscriber in  self.subscribers:
        #      subscriber.signal.disconnectAll()


    def create_callbacks(self):
        self.connect_callback("FaceDetected",self.on_human_tracked)
        # self.connect_callback("GazeAnalysis/PeopleLookingAtRobot",self.on_person_looks_at_robot)
        # self.connect_callback("GazeAnalysis/PersonStartsLookingAtRobot",self.on_person_starts_looking_at_robot)
        # self.connect_callback("GazeAnalysis/PersonStopsLookingAtRobot",self.on_person_stop_looking_at_robot)
        # self.connect_callback("PeoplePerception/PeopleList",self.on_people_list)
        # self.connect_callback("PeoplePerception/PeopleDetected",self.on_people_detected)
        # self.connect_callback("PeoplePerception/NonVisiblePeopleList",self.on_non_visible_people_list)
        # # self.connect_callback("PeoplePerception/JustLeft",self.on_just_left)
        # self.connect_callback("move_forward_event",self.move_forward_event)

    def set_awareness(self, state):
        """
        Turn on or off the basic awareness of the robot,
        e.g. looking for humans, self movements etc.

        :param state: If True set on, if False set off
        :type state: bool
        """
        if state:
            self.awareness_service.resumeAwareness()
            print("[INFO]: Awareness is turned on")
        else:
            self.awareness_service.pauseAwareness()
            print("[INFO]: Awareness is paused")


        
    def connect_callback(self, event_name, callback_function):
        try:
            subscriber = self.memory.subscriber(event_name)
            subscriber.signal.connect(callback_function)
            self.subscribers.append(subscriber)
        except Exception as e:
            print("Error is %s" % e)



      