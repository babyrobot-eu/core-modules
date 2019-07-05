#!/usr/bin/env python
# -*- coding: utf-8 -*-
from Tkinter import *
from tkNotebook import Notebook
import socket
import sys
sys.path.append("..")
from config import cfg

class WizardOfOzRemote(object):


    def __init__(self):
        #Default settings
        self.host = cfg['broker']
        self.port = 1932

        self.connect_to_broker()

        #Create canvas
        self.root = Tk()
        self.root.title("Multi3 Admin")

        #Create notebook
        self.noteBook = Notebook(self.root, width=1800, height=700, activefg='blue')
        self.noteBook.grid()

        #Create main tab in notebook
        self.sumsTab = self.noteBook.add_tab(text = "Sums Tab")
        self.emorecTab = self.noteBook.add_tab(text = "Emorec Tab")

        # self.drawSettingsTab()
        self.sumsTab.focus()
        
        self.buttons = []
        self.droplists = []
        self.addSumsTab()
        self.addEmorecTab()

    def add_button(self, tab, text, event, row, column, event_text=None, options=None, x=None, y=None, z=None, big=None, behavior=None):
        if not big:
            button = Button(tab, text=text, anchor=W, bg='grey', command= lambda: self.send_event(event, event_text, options,x,y,z,behavior))
        else:
            button = Button(tab, text=text, bg='grey', height=2, command= lambda: self.send_event(event, event_text, options,x,y,z,behavior))
        button.grid(row=row, column=column, sticky='EW')
        self.buttons.append(button)
    
    def add_droplist(self, tab, max_choice, event, row, column, event_text=None, options=None, x=None, y=None, z=None, big=None, behavior=None):
        var = StringVar(self.root)
        var.set(1)
        choices = range(1, max_choice + 1)
        droplist = OptionMenu(tab, var, *choices, command= lambda dropvalue: self.send_event(event, dropvalue, options,x,y,z,behavior))
        droplist.grid(row=row, column=column, sticky='EW')
        self.droplists.append(droplist)
    
    def ok(self, x):
        print('Value is ', x)

    def connect_to_broker(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Connect the socket to the port where the server is listening
        server_address = (self.host, self.port)
        self.sock.connect(server_address)

        message = 'CONNECT furhat admin \n'
        self.sock.sendall(message)

        l = self.sock.recv(8192)

        # message = 'SUBSCRIBE ** \n'
        # self.sock.sendall(message)


    def send_event(self, event, text=None, options=None, x=None, y=None, z=None, behavior=None):
        json_format_1 = """{ \"class\": \"iristk.system.Event\", \"event_name\": \"%s\", \"text\": \"%s\" }\n"""
        json_format_3 = """{ \"class\": \"iristk.system.Event\", \"event_name\": \"%s\", \"options\": \"%s\" }\n"""
        json_format_4 = """{ \"class\": \"iristk.system.Event\", \"event_name\": \"%s\", \"x\": %f,  \"y\": %f,  \"z\": %f }\n"""
        json_format_2 = """{ \"class\": \"iristk.system.Event\", \"event_name\": \"%s\"}\n"""
        json_format_5 = """{ \"class\": \"iristk.system.Event\", \"event_name\": \"%s\", \"behavior\": \"%s\" }\n"""
        event_format = "EVENT %s %s\n"
        if options is not None:
            js = json_format_3 % (event, options)
            self.sock.sendall(event_format % (event, len(js)))
            self.sock.sendall(js)
            print "Sending event %s with text %s" % (event,  options) 
            return       
        if x is not None:
            js = json_format_4 % (event, x,y,z)
            self.sock.sendall(event_format % (event, len(js)))
            self.sock.sendall(js)
            print "Sending event %s with text %s" % (event,  options) 
            return       

        if behavior is not None:
            js = json_format_5 % (event, behavior)
            self.sock.sendall(event_format % (event, len(js)))
            self.sock.sendall(js)
            print "Sending event %s with behavior %s" % (event,  behavior) 
            return       


        if text == None:
            js = json_format_2 % (event)
            self.sock.sendall(event_format % (event, len(js)))
            self.sock.sendall(js)
            print "Sending event %s" % event
        else:
            js = json_format_1 % (event, text)
            self.sock.sendall(event_format % (event, len(js)))
            self.sock.sendall(js)
            print "Sending event %s with text %s" % (event,  text)

    def addSumsTab(self):
        col = 1
        Label(self.sumsTab, text="Set child gender").grid(row=0, column=col)
        self.add_button(self.sumsTab, row=1, column=col, text="Male", event="athena.games.sums.male")
        self.add_button(self.sumsTab, row=2, column=col, text="Female", event="athena.games.sums.female")
        Label(self.sumsTab, text="Select child").grid(row=4, column=col)
        self.add_droplist(self.sumsTab, row=5, column=col, max_choice=3, event="athena.games.sums.iristk.childid")
        Label(self.sumsTab, text="Select session").grid(row=6, column=col)
        self.add_droplist(self.sumsTab, row=7, column=col, max_choice=20, event="athena.games.sums.iristk.sessionid")

        col += 1
        Label(self.sumsTab, text="Master Switch").grid(row=0, column=col)
        self.add_button(self.sumsTab, row=2, column=col, text="Stop", event="athena.games.sums.stop")
        self.add_button(self.sumsTab, row=3, column=col, text="Reset cardholder", event="athena.games.sums.resetcardholder")
        self.add_button(self.sumsTab, row=8, column=col, text="END SESSION", event="athena.games.sums.endsession")


        col += 1
        Label(self.sumsTab, text="Switch States Manually").grid(row=0, column=col)
        self.add_button(self.sumsTab, row=1, column=col, text="Idle state", event="athena.games.sums.idle")
        self.add_button(self.sumsTab, row=2, column=col, text="Introduction state", event="athena.games.sums.start")
        self.add_button(self.sumsTab, row=3, column=col, text="Child sum state", event="athena.games.sums.childsum")
        self.add_button(self.sumsTab, row=4, column=col, text="Robot wrong sum state", event="athena.games.sums.gotorobotwrongsum")
        self.add_button(self.sumsTab, row=5, column=col, text="Robot test state", event="athena.games.sums.gotorobottest")
        self.add_button(self.sumsTab, row=6, column=col, text="Replay state", event="athena.games.sums.gotoreplay")

        col += 1
        Label(self.sumsTab, text="Introduction State").grid(row=0, column=col)
        self.add_button(self.sumsTab, row=1, column=col, text="Child said Yes", event="athena.games.sums.start.respond",
            event_text="yes")
        self.add_button(self.sumsTab, row=2, column=col, text="Child said No", event="athena.games.sums.start.respond",
            event_text="no")
        
        col += 1
        Label(self.sumsTab, text="Child sum state").grid(row=0, column=col)
        self.add_button(self.sumsTab, row=1, column=col, text="Reask 1", event="athena.games.sums.reask1")
        self.add_button(self.sumsTab, row=2, column=col, text="Reask 2", event="athena.games.sums.reask2")
        self.add_button(self.sumsTab, row=3, column=col, text="Reask 3", event="athena.games.sums.reask3")

        col += 1
        Label(self.sumsTab, text="Robot wrong sum state").grid(row=0, column=col)
        self.add_button(self.sumsTab, row=1, column=col, text="Reask 1", event="athena.games.sums.robotwrong.reask1")
        self.add_button(self.sumsTab, row=2, column=col, text="Reask 2", event="athena.games.sums.robotwrong.reask2")
        self.add_button(self.sumsTab, row=3, column=col, text="Reask 3", event="athena.games.sums.robotwrong.reask3")

        col += 1
        Label(self.sumsTab, text="Replay state").grid(row=0, column=col)
        self.add_button(self.sumsTab, row=1, column=col, text="Yes", event="athena.games.sums.replay.yes")
        self.add_button(self.sumsTab, row=2, column=col, text="No", event="athena.games.sums.replay.no")

        col += 1
        Label(self.sumsTab, text="Zeno controls").grid(row=0, column=col)
        self.add_button(self.sumsTab, row=1, column=col, text="happy", event="athena.admin.zenohappy")
        self.add_button(self.sumsTab, row=2, column=col, text="sad", event="athena.admin.zenosad")
        self.add_button(self.sumsTab, row=3, column=col, text="neutral", event="athena.admin.zenoneutral")

        col += 1
        Label(self.sumsTab, text="Proposed Action").grid(row=0, column=col)
        self.add_button(self.sumsTab, row=1, column=col, text="0", event="athena.games.sums.proposedaction", event_text='0')
        self.add_button(self.sumsTab, row=2, column=col, text="1", event="athena.games.sums.proposedaction", event_text='1')
        self.add_button(self.sumsTab, row=3, column=col, text="2", event="athena.games.sums.proposedaction", event_text='2')
        self.add_button(self.sumsTab, row=4, column=col, text="3", event="athena.games.sums.proposedaction", event_text='3')
        self.add_button(self.sumsTab, row=5, column=col, text="4", event="athena.games.sums.proposedaction", event_text='4')

        col += 1
        Label(self.sumsTab, text="Zeno Show Numbers").grid(row=0, column=col)
        self.add_button(self.sumsTab, row=1, column=col, text="Show 0", event="athena.admin.zeno_card_select", event_text='0')
        self.add_button(self.sumsTab, row=2, column=col, text="Show 1", event="athena.admin.zeno_card_select", event_text='1')
        self.add_button(self.sumsTab, row=3, column=col, text="Show 2", event="athena.admin.zeno_card_select", event_text='2')
        self.add_button(self.sumsTab, row=4, column=col, text="Show 3", event="athena.admin.zeno_card_select", event_text='3')
        self.add_button(self.sumsTab, row=5, column=col, text="Show 4", event="athena.admin.zeno_card_select", event_text='4')

        col += 1
        Label(self.sumsTab, text="Zeno Reset").grid(row=0, column=col)
        self.add_button(self.sumsTab, row=1, column=col, text="Reset", event="athena.admin.zenoreset")



    def addEmorecTab(self):
        col = 1
        Label(self.emorecTab, text="Switch states manually").grid(row=0, column=col)
        self.add_button(self.emorecTab, row=1, column=col, text="Emorec_1 - Happy", event="athena.games.emorec.gotoemorec1")
        self.add_button(self.emorecTab, row=2, column=col, text="Emorec_2 - Happy card", event="athena.games.emorec.gotoemorec2")
        self.add_button(self.emorecTab, row=3, column=col, text="Emorec_3 - Happy Zeno", event="athena.games.emorec.gotoemorec3")
        self.add_button(self.emorecTab, row=4, column=col, text="Emorec_4 - Sad", event="athena.games.emorec.gotoemorec4")
        self.add_button(self.emorecTab, row=5, column=col, text="Emorec_5 - Sad card", event="athena.games.emorec.gotoemorec5")
        self.add_button(self.emorecTab, row=6, column=col, text="Emorec_6 - Sad Zeno", event="athena.games.emorec.gotoemorec6")

        col += 1
        Label(self.emorecTab, text="Card controls").grid(row=0, column=col)
        self.add_button(self.emorecTab, row=1, column=col, text="Clear cards", event="athena.games.emorec.clearcards")
        self.add_button(self.emorecTab, row=2, column=col, text="Show happiness", event="athena.games.emorec.showhappiness")
        self.add_button(self.emorecTab, row=3, column=col, text="Show sadness", event="athena.games.emorec.showsadness")

        col += 1
        Label(self.emorecTab, text="Child behavior").grid(row=0, column=col)
        self.add_button(self.emorecTab, row=1, column=col, text="Correct", event="athena.games.emorec.correct", event_text="yes")
        self.add_button(self.emorecTab, row=2, column=col, text="Wrong", event="athena.games.emorec.correct", event_text="no")

    ## RUN PROGRAM ##
    def run(self):
        #Starts and runs the GUI
        self.root.mainloop()

## AUTOSTART ##
if __name__ == "__main__":
    remote = WizardOfOzRemote()
    remote.run()
