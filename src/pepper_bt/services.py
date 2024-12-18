#!/usr/bin/env python
import qi
import naoqi
import argparse
import sys
import time
import os
from naoqi import ALProxy, ALModule, ALBroker
from pepper_bt import topics

import paramiko
from scp import SCPClient

import pepper_bt.constant as const
import subprocess
import pepper_bt.configs as cfg
import functools



import logging
logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)
# from naoqi_bridge_msgs.msg import PeoplePerceptionPeopleList, PeoplePerceptionPeopleDetected


class patched_SSHClient(paramiko.SSHClient):
    def _auth(self, username, password, *args):
        if not password:
            try:
                print("here")
                self._transport.auth_none(username)
                return
            except paramiko.BadAuthenticationType:
                pass
        paramiko.SSHClient._auth(self, username, password, *args)


class Pepper():

    def __init__(self, ip_address, port=9559):
        # This code get ip and port from command
        # parser = argparse.ArgumentParser()
        # # Todo: change it
        # parser.add_argument("--ip", type=str, default=ip_address,
        #                     help="Robot IP address. On robot or Local Naoqi: use %s." % ip_address)
        # parser.add_argument("--port", type=int, default=port,
        #                     help="Naoqi port number")
        # args = parser.parse_args()       
        self.connection_url = "tcp://" + ip_address + ":" + str(port)
        self.ip = ip_address
        self.port = port

        self.beep_volume = 70 #(0~100)
        self.tablet_signalID = 0

        # try:
        # self.ssh = patched_SSHClient()
        # self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        # self.ssh.load_system_host_keys()
        # self.ssh.connect(hostname="192.168.229.141",port=8080)
        #self.scp = SCPClient(self.ssh.get_transport())
        # except paramiko.AuthenticationException:
        #     print("Authentication failed. Check your credentials.")
        # except paramiko.SSHException as e:
        #     print("SSH error: ", e)
        # except Exception as e:
        #     print("An error occurred: ",e)
        
        try:
            #self.app = qi.Application(["TabletModule", "--qi-url=" + self.connection_url])
            self.session = qi.Session()
            self.session.connect(self.connection_url)
            self.led_service = self.session.service("ALLeds")
            self.memory_service = self.session.service("ALMemory")
            self.tablet_service = self.session.service("ALTabletService")
            self.motion_service = self.session.service("ALMotion")
            self.face_characteristic = self.session.service("ALFaceCharacteristics")
            self.speaking_movement = self.session.service("ALSpeakingMovement")
            self.animation_service = self.session.service("ALAnimationPlayer")
            self.audio_player = self.session.service("ALAudioPlayer")
            self.audio_recorder = self.session.service("ALAudioRecorder")
            self.audio_device = self.session.service("ALAudioDevice")
            self.speech_service = self.session.service("ALSpeechRecognition")
            self.speech_service.setLanguage("English")
        
            self.tts = self.session.service("ALTextToSpeech")
            self.posture_service = self.session.service("ALRobotPosture")

            # ssh = patched_SSHClient()
            # ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            # ssh.load_system_host_keys()
            #ssh.connect(hostname=ip_address, username="nao", password=" ")
            #self.scp = SCPClient(ssh.get_transport())

        except RuntimeError:
            print("Can't connect to Naoqi")
            # print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
            #     "Please check your script arguments. Run with -h option for help.")
            sys.exit(1)      
      
    def get_user_speech(self):
        self.audio_player.playSine(1000,self.beep_volume,1,0.3)
        time.sleep(.5)

    def close_app(self):
        qi._stopApplication()

        
    def listen_to_user(self):

        vocabulary = ["yes", "no"]
        # vocabulary = ["one", "yes", "artist", "two","who was the artist", "when the artist was born"]

        self.speech_service.pause(True)

        try:
            self.speech_service.setVocabulary(vocabulary, False)
        except RuntimeError as error:
            print(error)
            self.speech_service.removeAllContext()
            self.speech_service.setVocabulary(vocabulary, True)
            self.speech_service.subscribe("Test_ASR")
        try:

            self.audio_player.playSine(1000,self.beep_volume, 1, 0.3)
            self.blink_eyes([255, 255, 0])

            print("[INFO]: Robot is listening to you...")
            self.speech_service.pause(False)
            time.sleep(4)
            words = self.memory_service.getData("WordRecognized")
            print(words)
            return words[0]
        except:
            return "no"
        

        # Start the speech recognition engine with user Test_ASR
        # self.speech_service.subscribe("Test_ASR")
        # print ('Speech recognition engine started')
        # time.sleep(20)
        # self.speech_service.unsubscribe("Test_ASR")
        # Or, if you want to enable word spotting:
        #asr.setVocabulary(vocabulary, True)

        # self.speech_service.setAudioExpression(False)
        # self.speech_service.setVisualExpression(False)
        # self.audio_recorder.stopMicrophonesRecording()
        # print("[INFO]: Speech recognition is in progress. Say something.")
        # while True:
        #     #print(self.memory_service.getData("ALSpeechRecognition/Status"))
        #     if self.memory_service.getData("ALSpeechRecognition/Status") == "SpeechDetected":
        #         print("[INFO]: Robot is listening to you")
        #         self.blink_eyes([255, 255, 0])
        #         self.audio_recorder.startMicrophonesRecording("/home/nao/speech.wav", "wav", 48000, (0, 0, 1, 0))

        #         self.blink_eyes([255, 255, 0])
        #         break


   
   
        print("Stop Recording")
        self.blink_eyes([0, 0, 0])
    
    def listen(self):

        base_duration=3
        self.record_time = time.time() + base_duration

        self.audio_player.playSine(1000,self.beep_volume,1,0.3)

        self.speech_service.setAudioExpression(False)
        self.speech_service.setVisualExpression(False)

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
            print(self.memory_service.getData("ALSpeechRecognition/Status"))
            # if self.audio_terminate :
            #     self.audio_recorder.stopMicrophonesRecording()
            #     print('kill!!!')
            #     return None
            time.sleep(0.1)
        
        self.audio_recorder.stopMicrophonesRecording()


        with open("/home/nao/speech.wav", 'rb') as audio_file:
            content = audio_file.read()
            print(content)
			# audio_sample = speech_client.sample(
			# 	content=content,
			# 	source_uri=None,
			# 	encoding='LINEAR16',
			# 	sample_rate=16000)
			# try:
			# 	alternatives = speech_client.speech_api.sync_recognize(audio_sample,language_code='en-IN')
			# 	return (str(alternatives[0].transcript))
			# except ValueError:
    		# 		return ""
    

        # if not os.path.exists('./tmp'):
        #         os.mkdir('./tmp', 0755)

        # cmd = 'sshpass -p 1847! scp nao@'+str(self.ip)+':/home/nao/speech.wav ./audio_record'
        #cmd = 'scp nao@'+str(self.ip)+":9559"+':/home/nao/speech.wav ./audio_record'
        # cmd = 'scp nao@'+str(self.ip)+':/home/nao/speech.wav ./audio_record'
        # os.system(cmd)
        #self.download_file("/home/nao/speech.wav")
        #self.download_file("speech.wav")


    def download_file(self, file_name):
        """
        Download a file from robot to ./tmp folder in root.

        ..warning:: Folder ./tmp has to exist!
        :param file_name: File name with extension (or path)
        :type file_name: string
        """
        try:
            # sftp = self.ssh.open_sftp()
            # sftp.put(file_name, "/tmp/")
            # sftp.close()
            # # Close the SSH connection
            # self.ssh.close()

            self.scp.get(file_name, local_path="/tmp/")
            print("[INFO]: File " + file_name + " downloaded")
            self.scp.close()
        except paramiko.AuthenticationException:
            print("Authentication failed. Check your credentials.")
        except paramiko.SSHException as e:
            print("SSH error: ", e)
        except Exception as e:
            print("An error occurred: ",e)

    
    def blink_eyes(self, rgb):
        """
        Blink eyes with defined color

        :param rgb: Color in RGB space
        :type rgb: integer
        """

        self.led_service.fadeRGB('AllLeds', rgb[0], rgb[1], rgb[2], 1.0)



    def fade_ears(self):
        # Example showing how to fade the ears group to mid-intensity
        name = 'EarLeds'
        intensity = 0.5
        duration = 1.0
        self.led_service.fade(name, intensity, duration)

    def led_rasta(self):
   
        # Example showing a one second rasta animation
        duration = 1.0
        self.led_service.rasta(duration)

    def tablet_show_web(self, url = cfg.WEB_URL):
            try:
                # Ensure that the tablet wifi is enable
                self.tablet_service.enableWifi()
                self.tablet_service.showWebview(url)
                return True
            
            except Exception as e:
                print("Tablet show web has error: ",e)
                return False
            

    def tablet_show_rate(self):
        self.tablet_show_web(cfg.RATE_URL)
       

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
        self.tts.setParameter("speed", speed)
        print("speed of speack is :",self.tts.getParameter("speed"))
        
    def reset_speach_speed(self):
        self.tts.resetSpeed()
        print("speed of speack is :",self.tts.getParameter("speed"))

    def speech_gesture(self):
        names = list()
        times = list()
        keys = list()
        names.append("LElbowRoll")
        times.append([1.01, 1.3, 1.6, 1.9, 4.5])
        keys.append([-1.37289, -1.12923, -0.369652 ,-0.379652, -0.379653])
        self.motion_service.angleInterpolation(names, keys, times, True)

        #self.motion_service.angleInterpolationWithSpeed(["RShoulderPitch", "RWristYaw", "RHand"], [0.8, 2.5, 1.0], 1.0)

    def present_gesture(self, _async=False):
        #self.hand("left",False)
        if _async:
            self.start_animation("Explain_11")
        else:
            topics.Play_Animation().publish()

    def speech_recognition(self):
        print(self.__class__.__name__)
        
        # Subscribe to the microphone data
        #microphone_id = self.audio_device.getMicrophoneNames()[0]
        #print("mic_id ", microphone_id)
        # self.audio_device.subscribe(self.__class__.__name__)

        # base_duration = 3
        # self.record_time = time.time() + base_duration
    
        # try:
        #     while time.time() < self.record_time :
        #         print(time.time())
        #         # Get the audio buffer
        #         audio_data = self.audio_device.getData(microphone_id)
        #         # Process the audio_data as needed (e.g., save to a file)
        # except KeyboardInterrupt:
        #     pass
        # finally:
        #     # Unsubscribe and close the connection
        #     self.audio_device.unsubscribe(microphone_id)
        try:
            app = qi.Application(["Sound Processing Module", "--qi-url=" + self.connection_url])
        except RuntimeError:
            print ("HumanGreeter connection has error to connect")
            sys.exit(1)

        MySoundProcessingModule = SoundProcessingModule(app)
        # app.session.listen("0.0.0.0:9558")
        app.session.registerService("SoundProcessingModule", MySoundProcessingModule)
        MySoundProcessingModule.startProcessing()

        # global SpeechRecognition
        # SpeechRecognition = SpeechRecognitionModule("SpeechRecognition", app, self.ip, self.port)
        # SpeechRecognition.run()
        #print("buffer type", type(audio_data))
        # Set the audio configuration
        # sample_rate = 48000
        # channels = [0, 0, 1, 0]  # Mono front microphones
        # is_left = False
        # is_right = True
        # is_front = False
        # is_rear = False
        # _channels = 0 # ALL_Channels: 0,  AL::LEFTCHANNEL: 1, AL::RIGHTCHANNEL: 2 AL::FRONTCHANNEL: 3  or AL::REARCHANNEL: 4.
        # nDeinterleave = 0
        # self.audio_device.setClientPreferences(
        #     "MyAudioModule", sample_rate, _channels, nDeinterleave
        # )
        # print("fffffffffffffffffff")
        # # Subscribe to the audio data stream
        # buffer_size = 960  # Number of samples per buffer
        # self.audio_device.subscribe("MyAudioModule")
        # print("gggggggggggggggggggggggggggg")


        
    def cb(self, data):
        print("heee")
    
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
        """
        tablet specifications: Resolution: 1280*800
        """
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
    
    # def tablet_touch_handling(self):
    #     """
    #     If run and stop the application is needed, this function can be used
    #     """
    #     try:
    #        self.app.start()
    #        return self.app
    #     except RuntimeError:
    #        print ("tablet connection has error to connect")
    #        sys.exit(1)
        

    def _touch_down_feedback(self, **coordination_limit):
        if coordination_limit is None:
            lower_x = 0  
            upper_x = 1700 
            lower_y = 0 
            upper_y = 1024
        else:
            lower_x = coordination_limit['lower_x']
            upper_x = coordination_limit['upper_x'] 
            lower_y = coordination_limit['lower_y']
            upper_y = coordination_limit['upper_y']
        try:
            #session = app.session
            #tabletService = session.service("ALTabletService")
            coordinate = []
            # Don't forget to disconnect the signal at the end
            self.tablet_signalID = 0
            # function called when the signal onTouchDown is triggered
            def callback(x, y):
                print("coordinate are x: ", x, " y: ", y)
                if lower_x < x and upper_x > x and lower_y < y and upper_y > y:
                    coordinate.append({'x':x, 'y':y})
                    self.tablet_service.onTouchDown.disconnect(self.tablet_signalID)
                else:
                    print("wrong click")
                
                #app.stop()
               
             # attach the callback function to onJSEvent signal
            self.tablet_signalID = self.tablet_service.onTouchDown.connect(callback)
            #app.run()
            try:
                while len(coordinate) == 0:    
                    time.sleep(.5)
            except KeyboardInterrupt:
                print("Interrupted by user, stopping HumanGreeter")
                return None
            except RuntimeError as e:
                print("Touch down error : ", e)
                return None
            return coordinate[0]

        except Exception as  e:
            print("Error was: ", e)

    def shotdown_tablet_touch(self):
        self.tablet_service.onTouchDown.disconnect(self.tablet_signalID)


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
        #subprocess.Popen(["cd.."])
        try:
            #subprocess.Popen(["python", "-m", "SimpleHTTPServer"])
            subprocess.Popen(["python -m "])
        except Exception as error:
            subprocess.Popen(["python", "-m", "SimpleHTTPServer"])
        print("[INFO]: HTTPS server successfully started")




class HumanGreeter(object):

    def __init__(self, app):
        
        super(HumanGreeter, self).__init__()
        self.app = app
        self.app.start()
        session = self.app.session

        self.memory = session.service("ALMemory")
        self.gaze_analysis = session.service("ALGazeAnalysis")
        self.people_perception = session.service("ALPeoplePerception")
        self.posture_service   = session.service("ALRobotPosture")
        self.motion            = session.service("ALMotion")
        self.awareness_service = session.service("ALBasicAwareness")

        # Get the services ALTextToSpeech and ALFaceDetection.
        self.tts = session.service("ALTextToSpeech")
        self.face_detection = session.service("ALFaceDetection")
        #self.face_detection.subscribe("HumanGreeter")
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
             
         if value == []:  # empty value when the face disappears
            print("Face disappears")
            self.got_face = False
         elif not self.got_face:  # only speak the first time a face appears
            self.got_face = True
            print("I saw a face!")
            self.trigger_callback({"on":"human_tracked", "value":value})

        

    def on_touched(self, value):
        # Disconnect to the event when talking,
        # to avoid repetitions
        id = self.subscribers[1]['id']
        self.subscribers[1]['subscriber'].signal.disconnect(id)
        self.subscribers[1]['is_connected'] = False
        
        
        """
        [['LArm', True, [1L]], ['LHand/Touch/Back', True, [1L]]]
        [['RArm', True, [1L]], ['RHand/Touch/Back', True, [1L]]]
        [['Head', True, [1L]], ['Head/Touch/Rear', True, [1L]]]
        """

        for p in value:
            if p[1]:
                if p[0] == 'Head':
                    self.trigger_callback("FORCE_STOP")

        # Reconnect again to the event
        self.subscribers[1]['id'] = self.subscribers[1]['subscriber'].signal.connect(self.on_touched)
        self.subscribers[1]['is_connected'] = True


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
         #self.face_detection.unsubscribe("HumanGreeter")

    #    self.face_detection.unsubscribe("HumanGreeter")
       # Todo: don't forget for unsubscribe tablet module
    #    qi._stopApplication()
       self.stop_run = True
    #    for subscriber in  self.subscribers:
    #         if subscriber['is_connected']:
    #             id = subscriber['id']
    #             subscriber['subscriber'].signal.disconnect(id)
       

    def create_callbacks(self):
        self.connect_callback("FaceDetected",self.on_human_tracked)
        self.connect_callback("TouchChanged",self.on_touched)


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
            id = subscriber.signal.connect(callback_function)
            self.subscribers.append({"name":event_name, "subscriber":subscriber, "id":id, "is_connected":True})
        except Exception as e:
            print("Error is %s" % e)



class SoundProcessingModule(object):

    def __init__(self, app):
        try:
            super(SoundProcessingModule, self).__init__()
            app.start()
            session = app.session

            self.module_name = "SoundProcessingModule"
            # session.registerService(self.module_name, self)

            self.memory_service = session.service("ALMemory")
            self.memory_service.declareEvent("SoundProcessingModule")

            # Get the service ALAudioDevice.
            self.audio_service = session.service("ALAudioDevice")
            self.audio_player = session.service("ALAudioPlayer")
            self.isProcessingDone = False
            self.nbOfFramesToProcess = 20
            self.framesCount=0
            self.micFront = []


            print(self.audio_service.enableEnergyComputation())
            print(self.audio_service.getFrontMicEnergy())
            # self.audio_service.subscribe(self.module_name)

            print("ping", self.audio_service.ping() )
            # print("isRunning", self.audio_service.isRunning(0) )
            print("isRunning", self.audio_service.getMethodHelp("setClientPreferences") )
            self.audio_service.wait(1)
            self.audio_service.setClientPreferences(self.module_name, 48000, 0, 0)
            print("here")
            # services = session.services()
            # if "ALMemory" in services:
            #     print("ALAudioDevice service is available.")
            # else:
            #     print("ALAudioDevice service is not available.")




            # self.myBroker = ALBroker("pythonBroker", "0.0.0.0", 0, ip, port)
            # ALModule.__init__(self, strModuleName )
            
            # app.start()
            # session = app.session
            # self.BIND_PYTHON(self.__class__.__name__, "callback" )
            # self.strNaoIp = ip

            # self.memory = session.service("ALMemory")
            # declare event to ALMemory so other modules can subscribe
            # self.memory = ALProxy("ALMemory")
            # self.memory.declareEvent("SpeechRecognition")

            # is these 2 line necessary? what do they do?
            # just copied them from the examples...
            #self.BIND_PYTHON( self.getName(), "callback" )



        except Exception as err:
            print( "ERR: SpeechRecognitionModule: loading error: %s" % str(err) )


    def startProcessing(self):
        """
        Start processing
        """
        # ask for the front microphone signal sampled at 16kHz
        # if you want the 4 channels call setClientPreferences(self.module_name, 48000, 0, 0)

        # Initialize the audio device
        # self.audio_service.enableAudioInput(True)
        # self.audio_service.enableAudioOutput(True)
        
        print("First Step")
        
        self.audio_service.setClientPreferences(self.module_name, 48000, 0, 0)
        #self.audio_service.setClientPreferences(self.module_name, 16000, 3, 0)
        print("Second Step")
        self.audio_service.subscribe(self.module_name)

        while self.isProcessingDone == False:
            time.sleep(1)

        self.audio_service.unsubscribe(self.module_name)


    def process(self, nbOfChannels, nbrOfSamplesByChannel, buffer, timeStamp):
        print("buuuuuff")


    def processRemote(self, nbOfChannels, nbOfSamplesByChannel, timeStamp, inputBuffer):
        print("procccccccccccccc")


    def run(self):
        try:

            # audio = naoqi.ALProxy( "ALAudioDevice")
            # nNbrChannelFlag = 0 # ALL_Channels: 0,  AL::LEFTCHANNEL: 1, AL::RIGHTCHANNEL: 2 AL::FRONTCHANNEL: 3  or AL::REARCHANNEL: 4.
            # nDeinterleave = 0
            # SAMPLE_RATE = 48000
            #audio.setClientPreferences("SpeechRecognition",  SAMPLE_RATE, nNbrChannelFlag, nDeinterleave) # setting same as default generate a bug !?!
            #audio.subscribe("SpeechRecognition")
       
            while True:
                time.sleep(1)

        except KeyboardInterrupt:
            pass