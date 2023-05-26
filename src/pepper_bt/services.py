#!/usr/bin/env python
import qi
import argparse
import sys
import time
PEPPER_IP="10.0.0.3"

class TabletShowImage():

    def __init__(self):
        parser = argparse.ArgumentParser()
        parser.add_argument("--ip", type=str, default=PEPPER_IP,
                            help="Robot IP address. On robot or Local Naoqi: use %s." % PEPPER_IP)
        parser.add_argument("--port", type=int, default=9559,
                            help="Naoqi port number")
        args = parser.parse_args()
        self.session = qi.Session()
        try:
            self.session.connect("tcp://" + args.ip + ":" + str(args.port))
        except RuntimeError:
            print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
                "Please check your script arguments. Run with -h option for help.")
            sys.exit(1)
      


    def show_image(self,img_url):
        try:
            tabletService = self.session.service("ALTabletService")
            tabletService.enableWifi()
            #tabletService.playVideo("http://clips.vorwaerts-gmbh.de/big_buck_bunny.mp4")
            tabletService.showImage(img_url)
            #time.sleep(5)
            #tabletService.hideImage()
        except Exception as e:
            print("Error is %s" % e)