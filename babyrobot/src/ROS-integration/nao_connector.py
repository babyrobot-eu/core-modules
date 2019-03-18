#!/usr/bin/env python

import socket  # Networking support
import signal  # Signal support (server shutdown on signal receive)
import time    # Current time
import json
from naoqi import ALProxy
import socket
import sys
import StringIO
import rospy

''' Launch and stop a behavior on NAO ROBOT, if possible. '''
def launchAndStopBehavior(managerProxy, behaviorName):

    # Check that the behavior exists.
    if (managerProxy.isBehaviorInstalled(behaviorName)):

        # Check that it is not already running.
        if (not managerProxy.isBehaviorRunning(behaviorName)):
          # Launch behavior. This is a blocking call, use post if you do not
          # want to wait for the behavior to finish.
          managerProxy.runBehavior(behaviorName)
          #time.sleep(5)
        else:
          print "Behavior is already running."

    else:
        print "Behavior not found."
        return


''' Print behaviors inside the robot '''
def getBehaviors(managerProxy):
  names = managerProxy.getInstalledBehaviors()
  print "Behaviors on the robot:"
  print names

  names = managerProxy.getRunningBehaviors()
  print "Running behaviors:"
  print names

class Connector:

    ''' connects to the nao robot '''
    def connect_to_nao(self, ipAddress, port):
        self.nao_ip = ipAddress
        self.nao_port = port 
        self.tts = ALProxy("ALTextToSpeech", self.nao_ip, self.nao_port)
        self.manager = ALProxy("ALBehaviorManager", self.nao_ip, self.nao_port)
        self.motion = ALProxy("ALMotion", self.nao_ip, self.nao_port)
        self.aup = ALProxy("ALAudioPlayer", self.nao_ip, self.nao_port)
        rospy.init_node('nao_connector', anonymous=True)


    def __init__(self, broker_ip_address, broker_port, nao_ip_address, nao_port):
        """ Constructor """
        self.host = broker_ip_address   # <-- works on all avaivable network interfaces
        self.port = broker_port
        signal.signal(signal.SIGINT, self.graceful_shutdown)
        self.connect_to_nao(nao_ip_address, nao_port)
        getBehaviors(self.manager)
        self.connect_to_broker()

    def readlines(self, sock, recv_buffer=4096, delim='\n'):
        buffer = ''
        data = True
        while data:
            data = sock.recv(recv_buffer)
            buffer += data

            while buffer.find(delim) != -1:
                line, buffer = buffer.split('\n', 1)
                yield line
        return

    # ''' connect to iristk broker and wait for nao events '''
    def connect_to_broker(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Connect the socket to the port where the server is listening
        server_address = (self.host, self.port)
        self.sock.connect(server_address)

         # CONNECT [ticket] [myname] \n 
        message = 'CONNECT furhat nao \n'
        self.sock.sendall(message)

        # print "Aaa"
        
        l = self.sock.recv(8192)
        # print l

        message = 'SUBSCRIBE ** \n'
        self.sock.sendall(message)

        json_format_1 = """{ \"class\": \"iristk.system.Event\", \"event_name\": \"%s\", \"text\": \"%s\" }\n"""
        json_format_2 = """{ \"class\": \"iristk.system.Event\", \"event_name\": \"%s\"}\n"""
        event_format = "EVENT %s %s\n"

        next_line_read = False

        rospy.loginfo("Nao Connector is connected and ready")

        # read lines continuously from the socket and wait for events aimed to NAO

        for line in self.readlines(self.sock):
            # if read the next line as JSON
            if next_line_read:
                # read the line as a JSON
                j = json.loads(line)
                # print "BEHAVIOR: " + j["name"]
                # try:
                    # run the behavior on nao
                if "name" in j:
                    # print "PLAYING WAV", j["name"]
                    fullfile = str("/home/nao/audioAthena/%s.wav"%j["name"])
                    self.aup.playFile(fullfile)
                    rospy.loginfo("Nao playing behavior: {}".format(str(j["name"])))
                else:
                    # print "PLAYING BEHAVIOR", j["behavior"]
                    self.manager.runBehavior(str(j["behavior"]))                
                    rospy.loginfo("Nao playing behavior: {}".format(str(j["behavior"])))

                # Exception Exception, e:
                #     print e

                # send back the event that nao has finished his behavior
                js = json_format_2 % ("athena.nao.behavior.done" )
                self.sock.sendall(event_format % ("athena.nao.behavior.done", len(js)))
                self.sock.sendall(js)        

                # print "Received event " + j["event_name"] + " with behavior field: " + j["name"] + " at time " + j["event_time"] + "\n"
                next_line_read = False

            if line.startswith("EVENT"):
                data_arr = line.split()
                event_name = data_arr[1]

                # if the event is aimed to nao then we need to read the next line as a json which includes the behavior we need to run
                if event_name == "athena.nao.behavior":
                    next_line_read = True


    def graceful_shutdown(self, sig, dummy):
         """ This function shuts down the server. It's triggered
         by SIGINT signal """
         self.sock.close() #shut down the server
         sys.exit(1)

###########################################################
# shut down on ctrl+c

def main():
    s = Connector(broker_ip_address='192.168.0.105', broker_port=1932, nao_ip_address='192.168.0.121', nao_port=9559)  # construct server object


if __name__ == '__main__':
    main()


