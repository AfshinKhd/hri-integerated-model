#!/usr/bin/env python
import qi
import argparse
import sys
import time
import os
from naoqi import ALProxy
import paramiko
from scp import SCPClient
import pepper_bt.constant as const
import subprocess
import pepper_bt.configs as cfg
import logging
logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)
# from naoqi_bridge_msgs.msg import PeoplePerceptionPeopleList, PeoplePerceptionPeopleDetected


class patched_SSHClient(paramiko.SSHClient):
    def _auth(self, username, password, *args):
        if not password:
            try:
                self._transport.auth_none(username)
                return
            except paramiko.BadAuthenticationType:
                pass
        paramiko.SSHClient._auth(self, username, password, *args)


class Pepper():

    def __init__(self, ip_address, port=9559):
        parser = argparse.ArgumentParser()
        parser.add_argument("--ip", type=str, default=ip_address,
                            help="Robot IP address. On robot or Local Naoqi: use %s." % ip_address)
        parser.add_argument("--port", type=int, default=port,
                            help="Naoqi port number")
        args = parser.parse_args()       
        self.connection_url = "tcp://" + args.ip + ":" + str(args.port)
        self.ip = ip_address

        self.beep_volume = 70 #(0~100)
        
        try:
            self.app = qi.Application(["TabletModule", "--qi-url=" + self.connection_url])
            self.session = qi.Session()
            self.session.connect("tcp://" + args.ip + ":" + str(args.port))
            self.led_service = self.session.service("ALLeds")
            self.memory_service = self.session.service("ALMemory")
            self.tablet_service = self.session.service("ALTabletService")
            self.motion_service = self.session.service("ALMotion")
            self.face_characteristic = self.session.service("ALFaceCharacteristics")
            self.speaking_movement = self.session.service("ALSpeakingMovement")
            self.animation_service = self.session.service("ALAnimationPlayer")
            self.audio_player = self.session.service("ALAudioPlayer")
            self.audio_recorder = self.session.service("ALAudioRecorder")
            self.speech_service = self.session.service("ALSpeechRecognition")
            self.tts = self.session.service("ALTextToSpeech")
            self.posture_service = self.session.service("ALRobotPosture")

            ssh = patched_SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh.load_system_host_keys()
            #ssh.connect(hostname=ip_address, username="nao", password=" ")
            #self.scp = SCPClient(ssh.get_transport())

        except RuntimeError:
            print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
                "Please check your script arguments. Run with -h option for help.")
            sys.exit(1)      
      
    def get_user_speech(self):
        self.audio_player.playSine(1000,self.beep_volume,1,0.3)
        time.sleep(.5)

        
    def listen(self):
        if True:
            self.audio_player.playSine(1000,self.beep_volume,1,0.3)
            time.sleep(.5)
            self.blink_eyes([255, 255, 0])

        base_duration=3
        self.record_time = time.time()+base_duration
        self.audio_recorder.stopMicrophonesRecording()
        print('Speech Detected : Start Recording')
        channels = [0,0,1,0] #left,right,front,rear
        fileidx = "recog"
        #self.audio_recorder.startMicrophonesRecording("/home/nao/record/"+fileidx+".wav", "wav", 48000, channels)
        self.audio_recorder.startMicrophonesRecording("/home/nao/speech.wav", "wav", 48000, channels)
        #self.audio_recorder.startMicrophonesRecording("/home/nao/speech.wav", "wav", 48000, (0, 0, 1, 0))
        #
        #
        while time.time() < self.record_time :
            print(time.time())
            # if self.audio_terminate :
            #     self.audio_recorder.stopMicrophonesRecording()
            #     print('kill!!!')
            #     return None
            time.sleep(0.1)
        
        self.audio_recorder.stopMicrophonesRecording()
        self.audio_recorder.recording_ended = True

        if not os.path.exists('./audio_record'):
                os.mkdir('./audio_record', 0755)

        cmd = 'sshpass -p 1847! scp nao@'+str(self.ip)+':/home/nao/speech.wav ./audio_record'
        os.system(cmd)
        #self.download_file("speech.wav")
   
   
        print("Stop Recording")
        self.blink_eyes([0, 0, 0])


    
    def blink_eyes(self, rgb):
        """
        Blink eyes with defined color

        :param rgb: Color in RGB space
        :type rgb: integer

        :Example:

        >>> pepper.blink_eyes([255, 0, 0])

        """
        self.led_service.fadeRGB('AllLeds', rgb[0], rgb[1], rgb[2], 1.0)


    def share_localhost(folder):
        """
        Shares a location on localhost via HTTPS to Pepper be
        able to reach it by subscribing to IP address of this
        computer.

        :Example:


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


    def tablet_show_web(self, url = cfg.WEB_URL):
            try:
                # Ensure that the tablet wifi is enable
                self.tablet_service.enableWifi()
                # Display a web page on the tablet
                #self.tabletService.showWebview("http://www.facebook.com")
                #time.sleep(3)
                self.tablet_service.showWebview(url)
                return True
            except Exception as e:
                print("tablet_show_web function, error:",e)
                return False

    def tablet_hide_web(self):
         self.tablet_service.hideWebview()
    
    def download_file(self, file_name):
        """
        Download a file from robot to ./tmp folder in root.

        ..warning:: Folder ./tmp has to exist!
        :param file_name: File name with extension (or path)
        :type file_name: string
        """
        self.scp.get(file_name, local_path="/audio_record/")
        print("[INFO]: File " + file_name + " downloaded")
        self.scp.close()
    
    def play_sound(self, sound):
        """
        Play a `mp3` or `wav` sound stored on Pepper

        .. note:: This is working only for songs stored in robot.

        :param sound: Absolute path to the sound
        :type sound: string
        """
        print("[INFO]: Playing " + sound)
        self.audio_service.playFile(sound)

    def stop_sound(self):
        """Stop sound"""
        print("[INFO]: Stop playing the sound")
        self.audio_service.stopAll()
    
    def say(self, text):
        """
        Text to speech (robot internal engine)

        :param text: Text to speech
        :type text: string
        """
        self.tts.say(text)
        print("[INFO]: Robot says: " + text)

    def set_speech_speed(self, speed = 80):
        print("speed of speack is :",self.tts.getParameter("speed"))
        self.tts.setParameter("speed", speed)
        

    def reset_speach_speed(self):
        print("speed of speack is :",self.tts.getParameter("speed"))
        self.tts.resetSpeed()

    def speech_gesture(self):
        names = list()
        times = list()
        keys = list()
        names.append("LElbowRoll")
        times.append([1.01, 1.3, 1.6])
        keys.append([-1.37289, -1.12923, -0.369652])
        self.motion_service.angleInterpolation(names, keys, times, True)

        #self.motion_service.angleInterpolationWithSpeed(["RShoulderPitch", "RWristYaw", "RHand"], [0.8, 2.5, 1.0], 1.0)

    def present_gesture(self):
        #self.hand("left",False)
        self.start_animation("Explain_11")
    
    def hand(self, hand, close):
        """
        Close or open hand

        :param hand: Which hand
            - left
            - right
        :type hand: string
        :param close: True if close, false if open
        :type close: boolean
        """
        hand_id = None
        if hand == "left":
            hand_id = "LHand"
        elif hand == "right":
            hand_id = "RHand"

        if hand_id:
            if close:
                self.motion_service.setAngles(hand_id, 0.0, 0.2)
                print("[INFO]: Hand " + hand + "is closed")
            else:
                self.motion_service.setAngles(hand_id, 1.0, 0.2)
                print("[INFO]: Hand " + hand + "is opened")
        else:
            print("[INFO]: Cannot move a hand")

    

    def start_animation(self, animation):
        """
        Starts a animation which is stored on robot

        .. seealso:: Take a look a the animation names in the robot \
        http://doc.aldebaran.com/2-5/naoqi/motion/alanimationplayer.html#alanimationplayer

        :param animation: Animation name
        :type animation: string
        :return: True when animation has finished
        :rtype: bool
        """
        try:
            animation_finished = self.animation_service.run("animations/Stand/Gestures/" + animation, _async=True)
            animation_finished.value()
            return True
        except Exception as error:
            print(error)
            return False

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
    
    def tablet_touch_handling(self):
        try:
           self.app.start()
           return self.app
        except RuntimeError:
           print ("tablet connection has error to connect")
           sys.exit(1)
        

    def _touch_dwon_feedback(self, app):
        try:
            session = app.session
            tabletService = session.service("ALTabletService")
            painting = []
            # Don't forget to disconnect the signal at the end
            signalID = 0

            # function called when the signal onTouchDown is triggered
            def callback(x, y):
                print("coordinate are x: ", x, " y: ", y)
                if x < 690 and x > 80:
                    painting.append(const.judgment_of_cambyses_painting['name'])
                elif x > 690 and x < 1680:
                    painting.append(const.scream_painting['name'])
                else:
                    print("Dosn't exist painting on tablet")
                tabletService.onTouchDown.disconnect(signalID)
                #app.stop()
               
    
             # attach the callback function to onJSEvent signal
            signalID = tabletService.onTouchDown.connect(callback)
            #app.run()
            try:
                while len(painting) == 0:    
                    time.sleep(.5)
            except KeyboardInterrupt:
                print("Interrupted by user, stopping HumanGreeter")
                return None
            return painting[0]

        except Exception as  e:
            print("Error was: ", e)

    def stand(self):
        """Get robot into default standing position known as `StandInit` or `Stand`"""
        self.posture_service.goToPosture("Stand", 0.5)
        print("[INFO]: Robot is in default position")
        
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



      