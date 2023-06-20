#!/usr/bin/env python
import qi
import argparse
import sys
import time

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
            self.memory_service = self.session.service("ALMemory")
            self.gaze_analysis = self.session.service("ALGazeAnalysis")
            self.motion_service = self.session.service("ALMotion")
            self.people_perception = self.session.service("ALPeoplePerception")
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

    def gaze_detection(self):
        #self.connect_callback("GazeAnalysis/PersonStartsLookingAtRobot",self.on_gaze_data)

        try:
            print("tollerance: ", self.gaze_analysis.getTolerance())
            self.memory_service.subscriber("GazeAnalysis/PeopleLookingAtRobot").signal.connect(self.on_person_looks_at_robot)
            self.memory_service.subscriber("GazeAnalysis/PersonStartsLookingAtRobot").signal.connect(self.on_person_starts_looking_at_robot)
            self.memory_service.subscriber("GazeAnalysis/PersonStopsLookingAtRobot").signal.connect(self.on_person_stop_looking_at_robot)
        except Exception as e:
            print("Error is %s" % e)

        

        # Keep the script running
        while True:
            pass

    def people_(self):
        rospy.init_node("pepper_people_detection")
        self.people_perception.setFastModeEnabled(True)
        subscriber = self.memory_service.subscriber("PeoplePerception/PeopleDetected")
        subscriber.signal.connect(self.people_detected)
        rospy.spin()
    #     try:
    #         while True:
    #     # Keep the program running to receive person detection events
    #             pass
    #     except KeyboardInterrupt:
    # # Unsubscribe from person detection events and disconnect
    #         self.people_perception.unsubscribe("PersonDetection")
            
    
    def people_percept(self):
        # Enable person detection
        self.people_perception.setFastModeEnabled(True)
        print("hi")
        try:

            subscribers = []
            subscriber1 = self.memory_service.subscriber("PeoplePerception/PeopleList")
            subscriber1.signal.connect(self.on_people_recognized)
            subscribers.append(subscriber1)


            subscriber2 = self.memory_service.subscriber("PeoplePerception/PeopleDetected")
            subscriber2.signal.connect(self.people_detected)
            subscribers.append(subscriber2)
            # self.memory_service.subscriber("PeoplePerception/PeopleList", PeoplePerceptionPeopleList).signal.connect(self.on_people_recognized)
            # self.memory_service.subscriber("PeoplePerception/PeopleDetected", PeoplePerceptionPeopleDetected).signal.connect(self.people_detected)
        except Exception as e:
            print("Error is %s" % e)

        try:
            while True:
                # Keep the program running to receive person detection events
                pass
        except KeyboardInterrupt:
                subscriber2.signal.disconnect(self.people_detected)

    def people_detected(self, data):
        print("people detected!!!")

    

    def move_toward(self):
        self.memory_service.subscriber("move_forward_event").signal.connect(self.move_forward_event)
        while True:
            pass


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

        return
    
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

        return


    def get_face_properties(self):
        face_id = self.memory_service.getData("PeoplePerception/PeopleList")
        #self.speaking_movement.setEnabled(True)
                # loop on, wait for events until manual interruption
        print("face ids: ", face_id)
        # try:
        #     while True:
        #         time.sleep(1)
        # except KeyboardInterrupt:
           # print "Interrupted by user, shutting down"

           # print "Starting Clean Up process"
           # self.cleanUp()

          #  print "Waiting for the robot to be in rest position"
          #  self.motion.rest()

            # sys.exit(0)
    
    def recognize_people(self):
        try:
            app = qi.Application(["HumanGreeter", "--qi-url=" + self.connection_url])
        except RuntimeError:
            print ("HumanGreeter connection has error to connect")
            sys.exit(1)
         
        human_greeter = HumanGreeter(app)
        human_greeter.run()
        

    def on_people_recognized(self, data):
       print("poeple")
        

    def connect_callback(self, event_name, callback_function):
        print("Callback Connection")
        #self.memory_service.subscriber(event_name).signal.connect(callback_function)
        self.memory_service.subscriber("GazeAnalysis/PersonStartsLookingAtRobot").signal.connect(self.on_person_looks_at_robot)

    def on_person_looks_at_robot(self, data):
        print("here")
        event_name = data[0]
        person_id = data[1]
        subscriber_identifier = data[2]
       
        print("event_name: ", event_name)
        print("person_id: ", person_id)
        print("subscriber_identifier: ", subscriber_identifier)

    def on_person_starts_looking_at_robot(self,data):
        print("start")

    def on_person_stop_looking_at_robot(self,data):
        print("stop")
    
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
        
        self.subscriber = self.memory.subscriber("FaceDetected")
        self.subscriber.signal.connect(self.on_human_tracked)
        # Get the services ALTextToSpeech and ALFaceDetection.
        # self.tts = session.service("ALTextToSpeech")
        self.face_detection = session.service("ALFaceDetection")
        self.face_detection.subscribe("HumanGreeter")
        self.got_face = False


    def on_human_tracked(self, value):
        """
        Callback for event FaceDetected.
        """
        print("faced detected!!!!")


    def run(self):
        print("Starting HumanGreeter")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("Interrupted by user, stopping HumanGreeter")
            self.face_detection.unsubscribe("HumanGreeter")
            #stop
            sys.exit(0)