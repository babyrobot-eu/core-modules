#!/usr/bin/env python
# -*- coding: utf-8 -*-
from Tkinter import *
from tkNotebook import Notebook
import socket
import sys
import rospy


class WizardOfOzRemote(object):


    def __init__(self):
        #Default settings
        self.host = "192.168.0.105"
        self.port = 1932        

        self.connect_to_broker()

        #Create canvas
        self.root = Tk()
        self.root.title("Multi3 Admin")

        #Create notebook
        self.noteBook = Notebook(self.root, width=1800, height=700, activefg='blue')
        self.noteBook.grid()

        #Create main tab in notebook
        self.mainTab = self.noteBook.add_tab(text = "Main Tab")
        self.guessTab = self.noteBook.add_tab(text = "GtO Tab")

        rospy.init_node("Wizard of Oz", anonymous=True)
        rospy.loginfo("Wizard of Oz is connected and ready")

        # self.drawSettingsTab()
        self.mainTab.focus()
        
        self.buttons = []
        # self.root.grid_columnconfigure(4, minsize=100)  # Here
        self.addObjectTab()
        # self.buttons = []
        i=1
        Label(self.mainTab, text="Gestures").grid(row=0, column=i)

        self.add_button(self.mainTab,row=1, column=i, text="greetings", event="athena.gesture.recognized", event_text="greetings")
        self.add_button(self.mainTab,row=2, column=i, text="come_closer", event="athena.gesture.recognized", event_text="come_closer")
        self.add_button(self.mainTab,row=3, column=i, text="point", event="athena.gesture.recognized", event_text="point")
        self.add_button(self.mainTab,row=4, column=i, text="sit_down", event="athena.gesture.recognized", event_text="sit_down")
        self.add_button(self.mainTab,row=5, column=i, text="stop", event="athena.gesture.recognized", event_text="stop")
        self.add_button(self.mainTab,row=6, column=i, text="circle", event="athena.gesture.recognized", event_text="circle")
        self.add_button(self.mainTab,row=7, column=i, text="none", event="athena.gesture.recognized", event_text="none")

        i+=1
        Label(self.mainTab, text="ASR Global").grid(row=0, column=i)

        self.add_button(self.mainTab,row=1, column=i, text="nai", event="athena.asr.recognized", event_text="nai")
        self.add_button(self.mainTab,row=2, column=i, text="oxi", event="athena.asr.recognized", event_text="oxi")
        self.add_button(self.mainTab,row=3, column=i, text="none", event="athena.asr.recognized", event_text="none")


        Label(self.mainTab, text="Listeners").grid(row=0, column=5)
        self.add_button(self.mainTab,row=1, column=5, text="Listen gesture", event="athena.gesture.listen")
        self.add_button(self.mainTab,row=2, column=5, text="Listen pantomime", event="athena.pantomime.listen")
        self.add_button(self.mainTab,row=3, column=5, text="Listen kinect_gesture", event="athena.kinect_gesture.listen")
        self.add_button(self.mainTab,row=4, column=5, text="Listen asr nai oxi", event="athena.asr.listen", options="nai,oxi")
        self.add_button(self.mainTab,row=5, column=5, text="Listen asr zwa", event="athena.asr.listen", options="mhrykastiko,papia,kyknos,kota,peristeri,galopoyla,pagoni,cari,batraxos,gata,skylos,koyneli,skioyros,skantzoxoiros,goyroyni,probato,katsika,agelada,gaidaros,alogo")
        self.add_button(self.mainTab,row=6, column=5, text="Listen asr pantomime", event="athena.asr.listen", options="xoreywXoro,paizwKithara,karfwma,bafwToixo,tzamia,odhghsh,skabwTrypa,skoupizwPatwma,kolympi,gymanzomai,siderwma,gyrnawSelides")
        self.add_button(self.mainTab,row=7, column=5, text="Listen emotion", event="athena.emotion.listen")



        Label(self.mainTab, text="Part1").grid(row=0, column=3)
        self.add_button(self.mainTab,row=1, column=3, text="xoreywXoro", event="athena.asr.recognized", event_text="xoreywXoro")
        self.add_button(self.mainTab,row=2, column=3, text="paizwKithara", event="athena.asr.recognized", event_text="paizwKithara")
        self.add_button(self.mainTab,row=3, column=3, text="karfwma", event="athena.asr.recognized", event_text="karfwma")
        self.add_button(self.mainTab,row=4, column=3, text="odhghsh", event="athena.asr.recognized", event_text="odhghsh")
        self.add_button(self.mainTab,row=5, column=3, text="bafwToixo", event="athena.asr.recognized", event_text="bafwToixo")
        self.add_button(self.mainTab,row=6, column=3, text="kolympi", event="athena.asr.recognized", event_text="kolympi")

        Label(self.mainTab, text="Part2").grid(row=0, column=4)
        self.add_button(self.mainTab,row=1, column=4, text="gymanzomai", event="athena.asr.recognized", event_text="gymanzomai")
        self.add_button(self.mainTab,row=2, column=4, text="gyrnawSelides", event="athena.asr.recognized", event_text="gyrnawSelides")
        self.add_button(self.mainTab,row=3, column=4, text="tzamia", event="athena.asr.recognized", event_text="tzamia")
        self.add_button(self.mainTab,row=4, column=4, text="siderwma", event="athena.asr.recognized", event_text="siderwma")
        self.add_button(self.mainTab,row=5, column=4, text="skoupizwPatwma", event="athena.asr.recognized", event_text="skoupizwPatwma")
        self.add_button(self.mainTab,row=6, column=4, text="skabwTrypa", event="athena.asr.recognized", event_text="skabwTrypa")


        Label(self.mainTab, text="Part 1 NAO recognizes").grid(row=8, column=3)
        Label(self.mainTab, text="Part 2 NAO recognizes").grid(row=8, column=4)
        self.add_button(self.mainTab,row=9, column=3, text="xoreywXoro", event="athena.pantomime.recognized", event_text="xoros1")
        self.add_button(self.mainTab,row=10, column=3, text="paizwKithara", event="athena.pantomime.recognized", event_text="kithara1") 
        self.add_button(self.mainTab,row=11, column=3, text="karfwma", event="athena.pantomime.recognized", event_text="karfono1")
        self.add_button(self.mainTab,row=12, column=3, text="odhghsh", event="athena.pantomime.recognized", event_text="leoforeio1")
        self.add_button(self.mainTab,row=13, column=3, text="bafwToixo", event="athena.pantomime.recognized", event_text="vafo1")
        self.add_button(self.mainTab,row=14, column=3, text="kolympi", event="athena.pantomime.recognized", event_text="kolympi1")
        
        self.add_button(self.mainTab,row=9, column=4, text="gymanzomai", event="athena.pantomime.recognized", event_text="gym1")
        self.add_button(self.mainTab,row=10, column=4, text="gyrnawSelides", event="athena.pantomime.recognized", event_text="vivlio1")
        self.add_button(self.mainTab,row=11, column=4, text="tzamia", event="athena.pantomime.recognized", event_text="tzamia1")
        self.add_button(self.mainTab,row=12, column=4, text="siderwma", event="athena.pantomime.recognized", event_text="siderono1")
        self.add_button(self.mainTab,row=13, column=4, text="skoupizwPatwma", event="athena.pantomime.recognized", event_text="skoupizeis1")
        self.add_button(self.mainTab,row=14, column=4, text="skabwTrypa", event="athena.pantomime.recognized", event_text="trypa1")
        
        
        






        Label(self.mainTab, text="Annotation").grid(row=10, column=1)

        Label(self.mainTab, text="ASR").grid(row=11, column=1)
        self.add_button(self.mainTab,row=12, column=1, text=u"ΣΩΣΤΟ", event="athena.annotation.asr.correct")
        self.add_button(self.mainTab,row=13, column=1, text=u"ΛΑΘΟΣ", event="athena.annotation.asr.correct")

        Label(self.mainTab, text="GESTURE").grid(row=15, column=1)
        self.add_button(self.mainTab,row=16, column=1, text=u"ΣΩΣΤΟ", event="athena.annotation.gesture.correct")
        self.add_button(self.mainTab,row=17, column=1, text=u"ΛΑΘΟΣ", event="athena.annotation.gesture.correct")






        Label(self.mainTab, text="Admin").grid(row=0, column=7)
        self.add_button(self.mainTab,row=1, column=7, text="Start", event="athena.admin.start")
        self.add_button(self.mainTab,row=2, column=7, text="Pause", event="athena.admin.pause")
        self.add_button(self.mainTab,row=3, column=7, text="Start Gesture", event="athena.admin.start_gesture")
        self.add_button(self.mainTab,row=4, column=7, text="Start Emorec", event="athena.admin.start_emorec")
        self.add_button(self.mainTab,row=5, column=7, text="Start Pantomime", event="athena.admin.start_pantomime")
        self.add_button(self.mainTab,row=6, column=7, text="Start RPS", event="athena.admin.start_rpc")
        self.add_button(self.mainTab,row=7, column=7, text="Start Farm", event="athena.admin.start_farm")

        self.add_button(self.mainTab,row=8, column=7, text="Start Engagement", event="athena.admin.start_engagement")

        self.add_button(self.mainTab,row=9, column=7, text="Male", event="athena.admin.gender_change", event_text="male")
        self.add_button(self.mainTab,row=10, column=7, text="Female", event="athena.admin.gender_change", event_text="female")



        self.add_button(self.mainTab,row=15, column=7, text="Test Mode", event="athena.admin.test")
        self.add_button(self.mainTab,row=16, column=7, text="Test Gesture", event="athena.test.gesture")
        self.add_button(self.mainTab,row=17, column=7, text="Test Pantomime", event="athena.test.pantomime")
        self.add_button(self.mainTab,row=18, column=7, text="Test Emotion", event="athena.test.emotion")



        Label(self.mainTab, text="RPS").grid(row=0, column=9)
        self.add_button(self.mainTab,row=1, column=9, text="First Wins", event="athena.games.rpc.round_winner", event_text="first")
        self.add_button(self.mainTab,row=2, column=9, text="Second Wins", event="athena.games.rpc.round_winner", event_text="second")
        self.add_button(self.mainTab,row=3, column=9, text="Tie", event="athena.games.rpc.round_winner", event_text="tie")

        Label(self.mainTab, text="Nao").grid(row=0, column=11)
        self.add_button(self.mainTab,row=1, column=11, text="Behavior Done", event="athena.nao.behavior.done")

        Label(self.mainTab, text="Games").grid(row=0, column=13)
        self.add_button(self.mainTab,row=1, column=13, text="Start Touch Emorec", event="athena.games.emorec.start")
        self.add_button(self.mainTab,row=2, column=13, text="Stop Touch Emorec", event="athena.games.emorec.stop")
        self.add_button(self.mainTab,row=3, column=13, text="Start Touch Pantomime", event="athena.games.pantomime.start")
        self.add_button(self.mainTab,row=4, column=13, text="Continue Touch Pantomime", event="athena.games.pantomime.continue")
        self.add_button(self.mainTab,row=5, column=13, text="Stop Touch Pantomime", event="athena.games.pantomime.stop")
        self.add_button(self.mainTab,row=6, column=13, text="Start Farm", event="athena.games.farm.start")
        self.add_button(self.mainTab,row=7, column=13, text="Stop Farm", event="athena.games.farm.stop")

        Label(self.mainTab, text="Farm Animals").grid(row=0, column=15)
        self.add_button(self.mainTab,row=1, column=15, text="Αγελάδα", event="athena.asr.recognized", event_text="agelada")
        self.add_button(self.mainTab,row=2, column=15, text="Άλογο", event="athena.asr.recognized", event_text="alogo")
        self.add_button(self.mainTab,row=3, column=15, text="Βάτραχος", event="athena.asr.recognized", event_text="batraxos")
        self.add_button(self.mainTab,row=4, column=15, text="Γάιδαρος", event="athena.asr.recognized", event_text="gaidaros")
        self.add_button(self.mainTab,row=5, column=15, text="Γαλοπούλα", event="athena.asr.recognized", event_text="galopoyla")
        self.add_button(self.mainTab,row=6, column=15, text="Γάτα", event="athena.asr.recognized", event_text="gata")
        self.add_button(self.mainTab,row=7, column=15, text="Γουρούνι", event="athena.asr.recognized", event_text="goyroyni")
        self.add_button(self.mainTab,row=8, column=15, text="Κατσίκα", event="athena.asr.recognized", event_text="katsika")
        self.add_button(self.mainTab,row=9, column=15, text="Κότα", event="athena.asr.recognized", event_text="kota")
        self.add_button(self.mainTab,row=10, column=15, text="Κουνέλι", event="athena.asr.recognized", event_text="koyneli")
        self.add_button(self.mainTab,row=11, column=15, text="Κύκνος", event="athena.asr.recognized", event_text="kyknos")
        self.add_button(self.mainTab,row=12, column=15, text="Πάπια", event="athena.asr.recognized", event_text="papia")
        self.add_button(self.mainTab,row=13, column=15, text="Παγώνι", event="athena.asr.recognized", event_text="pagoni")
        self.add_button(self.mainTab,row=14, column=15, text="Πρόβατο", event="athena.asr.recognized", event_text="probato")
        self.add_button(self.mainTab,row=15, column=15, text="Περιστέρι", event="athena.asr.recognized", event_text="peristeri")
        self.add_button(self.mainTab,row=16, column=15, text="Σκαντζόχοιρος", event="athena.asr.recognized", event_text="skantzoxoiros")
        self.add_button(self.mainTab,row=17, column=15, text="Σκίουρος", event="athena.asr.recognized", event_text="skioyros")
        self.add_button(self.mainTab,row=18, column=15, text="Σκύλος", event="athena.asr.recognized", event_text="skylos")
        self.add_button(self.mainTab,row=19, column=15, text="Ψάρι", event="athena.asr.recognized", event_text="cari")

        Label(self.mainTab, text="Farm properties 1").grid(row=0, column=17)
        self.add_button(self.mainTab,row=1, column=17, text="Μέγάλο", event="athena.asr.recognized", event_text="megalo")
        self.add_button(self.mainTab,row=2, column=17, text="Μεσαίο", event="athena.asr.recognized", event_text="mesaio")
        self.add_button(self.mainTab,row=3, column=17, text="Μικρό", event="athena.asr.recognized", event_text="mikro")

        self.add_button(self.mainTab,row=4, column=17, text="Πτηνά", event="athena.asr.recognized", event_text="pthna")
        self.add_button(self.mainTab,row=5, column=17, text="Αμφίβια", event="athena.asr.recognized", event_text="amfibia")
        self.add_button(self.mainTab,row=6, column=17, text="Θηλαστικά", event="athena.asr.recognized", event_text="uhlastika")
        self.add_button(self.mainTab,row=7, column=17, text="Ψάρια", event="athena.asr.recognized", event_text="caria")

        self.add_button(self.mainTab,row=8, column=17, text="Κίτρινο", event="athena.asr.recognized", event_text="kitrino")
        self.add_button(self.mainTab,row=9, column=17, text="Άσπρο", event="athena.asr.recognized", event_text="aspro")
        self.add_button(self.mainTab,row=10, column=17, text="Μαύρο", event="athena.asr.recognized", event_text="mayro")
        self.add_button(self.mainTab,row=11, column=17, text="Καφέ", event="athena.asr.recognized", event_text="kafe")
        self.add_button(self.mainTab,row=12, column=17, text="Γκρι", event="athena.asr.recognized", event_text="gkri")
        self.add_button(self.mainTab,row=13, column=17, text="Ροζ", event="athena.asr.recognized", event_text="roz")
        self.add_button(self.mainTab,row=14, column=17, text="Πράσινο", event="athena.asr.recognized", event_text="prasino")

        self.add_button(self.mainTab,row=15, column=17, text="Πόδια4", event="athena.asr.recognized", event_text="podia4")
        self.add_button(self.mainTab,row=16, column=17, text="Ποδια2", event="athena.asr.recognized", event_text="podia2")
        self.add_button(self.mainTab,row=17, column=17, text="Πόδια0", event="athena.asr.recognized", event_text="podia0")


        Label(self.mainTab, text="Farm properties 2").grid(row=0, column=19)

        self.add_button(self.mainTab,row=1, column=19, text="Χριστούγεννα", event="athena.asr.recognized", event_text="xmas")
        self.add_button(self.mainTab,row=2, column=19, text="entypOura", event="athena.asr.recognized", event_text="entypOura")
        self.add_button(self.mainTab,row=3, column=19, text="Γυάλα", event="athena.asr.recognized", event_text="gyala")
        self.add_button(self.mainTab,row=4, column=19, text="Πριγκίπισσα", event="athena.asr.recognized", event_text="prigkipissa")
        self.add_button(self.mainTab,row=5, column=19, text="Σπίτι", event="athena.asr.recognized", event_text="spiti")
        self.add_button(self.mainTab,row=6, column=19, text="Φίλος Ανθρώπου", event="athena.asr.recognized", event_text="filosAnur")

        self.add_button(self.mainTab,row=7, column=19, text="Καρότα", event="athena.asr.recognized", event_text="karota")
        self.add_button(self.mainTab,row=8, column=19, text="Φουντωτή Ουρά", event="athena.asr.recognized", event_text="fountOyra")
        self.add_button(self.mainTab,row=9, column=19, text="Αγκάθια", event="athena.asr.recognized", event_text="agkauia")
        self.add_button(self.mainTab,row=10, column=19, text="Τρώει τα πάντα", event="athena.asr.recognized", event_text="trweiPanta")
        self.add_button(self.mainTab,row=11, column=19, text="Δίνει μαλλί", event="athena.asr.recognized", event_text="malli")
        self.add_button(self.mainTab,row=12, column=19, text="Βράχια", event="athena.asr.recognized", event_text="braxia")
        self.add_button(self.mainTab,row=13, column=19, text="Μηρυκαστικό", event="athena.asr.recognized", event_text="mhrykastiko")
        self.add_button(self.mainTab,row=14, column=19, text="Αν Πετάει", event="athena.asr.recognized", event_text="anPetaei")
        self.add_button(self.mainTab,row=15, column=19, text="Καλπασμός", event="athena.asr.recognized", event_text="kalpasmo")
        self.add_button(self.mainTab,row=16, column=17, text="Αυγά", event="athena.asr.recognized", event_text="ayga")
        self.add_button(self.mainTab,row=17, column=17, text="Λαιμός", event="athena.asr.recognized", event_text="laimo")
        self.add_button(self.mainTab,row=18, column=17, text="Ειρήνη", event="athena.asr.recognized", event_text="eirhnh")

        Label(self.mainTab, text="Farm events").grid(row=0, column=21)

        self.add_button(self.mainTab,row=1, column=21, text="correct_placement", event="athena.games.farm.correct_placement")
        self.add_button(self.mainTab,row=2, column=21, text="incorrect_placement", event="athena.games.farm.incorrect_placement")

        Label(self.mainTab, text="Robot behaviors").grid(row=0, column=23)
        Label(self.mainTab, text="Nao").grid(row=1, column=23)
        self.add_button(self.mainTab,row=2, column=23, text="Behavior done",  event="athena.nao.behavior.done")
        self.add_button(self.mainTab,row=3, column=23, text="Stand up",  event="athena.nao.behavior", behavior="standup")

        Label(self.mainTab, text="Zeno").grid(row=5, column=23)
        self.add_button(self.mainTab,row=6, column=23, text="Zeno behavior done",  event="athena.zeno.behavior.done")


        Label(self.mainTab, text="Pantomime Kill").grid(row=7, column=23)
        self.add_button(self.mainTab,row=8, column=23, text="xoros",  event="athena.games.pantomime.xoros")
        self.add_button(self.mainTab,row=9, column=23, text="kolibi",  event="athena.games.pantomime.kolibi")
        self.add_button(self.mainTab,row=10, column=23, text="vivlio",  event="athena.games.pantomime.vivlio")
        self.add_button(self.mainTab,row=11, column=23, text="trww",  event="athena.games.pantomime.trww")
        self.add_button(self.mainTab,row=12, column=23, text="xtena",  event="athena.games.pantomime.xtena")
        self.add_button(self.mainTab,row=13, column=23, text="gimnastiki",  event="athena.games.pantomime.gimnastiki")


        Label(self.mainTab, text="Launch").grid(row=15, column=20)
        self.add_button(self.mainTab,row=16, column=20, text="speech pantomime start", event_text="/home/kinect/indigo_ws/src/online_multimodal/launch/babyaudio_iros.launch",  event="athena.launch.launch")
        self.add_button(self.mainTab,row=17, column=20, text="speech pantomime stop", event_text="/home/kinect/indigo_ws/src/online_multimodal/launch/babyaudio_iros.launch",  event="athena.launch.stop")
        self.add_button(self.mainTab,row=18, column=20, text="speech farm start", event_text="/home/kinect/indigo_ws/src/online_multimodal/launch/babyaudio_interspeech.launch",  event="athena.launch.launch")
        self.add_button(self.mainTab,row=19, column=20, text="speech farm stop", event_text="/home/kinect/indigo_ws/src/online_multimodal/launch/babyaudio_interspeech.launch",  event="athena.launch.stop")
        # self.add_button(self.mainTab,row=18, column=23, text="/home/kinect/indigo_ws/src/online_multimodal/launch",  event="athena.launch.launch")


    def add_button(self, tab, text, event, row, column, event_text=None, options=None, x=None, y=None, z=None, big=None, behavior=None):
        if not big:
            button = Button(tab, text=text, anchor=W, bg='grey', command= lambda: self.send_event(event, event_text, options,x,y,z,behavior))
        else:
            button = Button(tab, text=text, bg='grey', height=2, command= lambda: self.send_event(event, event_text, options,x,y,z,behavior))
        button.grid(row=row, column=column, sticky='EW')
        self.buttons.append(button)

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
            # print "Sending event %s with text %s" % (event,  options) 
            rospy.loginfo("Sending event %s with text %s" % (event,  options))
            return       
        if x is not None:
            js = json_format_4 % (event, x,y,z)
            self.sock.sendall(event_format % (event, len(js)))
            self.sock.sendall(js)
            # print "Sending event %s with text %s" % (event,  options) 
            rospy.loginfo("Sending event %s with text %s" % (event,  options))
            return       

        if behavior is not None:
            js = json_format_5 % (event, behavior)
            self.sock.sendall(event_format % (event, len(js)))
            self.sock.sendall(js)
            rospy.loginfo("Sending event %s with behavior %s" % (event,  behavior))

            # print "Sending event %s with behavior %s" % (event,  behavior) 
            return       


        if text == None:
            js = json_format_2 % (event)
            self.sock.sendall(event_format % (event, len(js)))
            self.sock.sendall(js)
            # print "Sending event %s" % event
            rospy.loginfo("Sending event %s" % event)

        else:
            js = json_format_1 % (event, text)
            self.sock.sendall(event_format % (event, len(js)))
            self.sock.sendall(js)
            rospy.loginfo("Sending event %s with text %s" % (event,  text))

    def addObjectTab(self):

        # Guess The Object Tab
        Label(self.guessTab, text="Start Game").grid(row=0, column=1)
        self.add_button(self.guessTab, row=1, column=1, text="Start", event="athena.admin.start_object")
        self.add_button(self.guessTab, row=2, column=1, text="start_game", event="iccs.main.start", event_text="start_game")

        Label(self.guessTab, text="Global").grid(row=3, column=1)
        self.add_button(self.guessTab, row=4, column=1, text="Yes", event="iccs.asr.recognised", event_text="yes")
        self.add_button(self.guessTab, row=5, column=1, text="No", event="iccs.asr.recognised", event_text="no")
        self.add_button(self.guessTab, row=9, column=1, text="Continue", event="iccs.got.continue", event_text="continue")


        Label(self.guessTab, text="Ask for Game").grid(row=0, column=3)
        self.add_button(self.guessTab, row=1, column=3, text="Ask Male", event="iccs.got.askmale", event_text="ask_male")
        self.add_button(self.guessTab, row=2, column=3, text="Ask Female", event="iccs.got.askfemale", event_text="ask_female")

        Label(self.guessTab, text="Objects").grid(row=0, column=4)
        self.add_button(self.guessTab, row=1, column=4, text="Ball", event="iccs.got.object", event_text="μπάλα")
        self.add_button(self.guessTab, row=2, column=4, text="Book", event="iccs.got.object", event_text="βιβλίο")
        self.add_button(self.guessTab, row=3, column=4, text="Box", event="iccs.got.object", event_text="κουτί")
        self.add_button(self.guessTab, row=4, column=4, text="Spoon", event="iccs.got.object", event_text="κουτάλι")
        self.add_button(self.guessTab, row=5, column=4, text="Pencil", event="iccs.got.object", event_text="μολύβι")


        Label(self.guessTab, text="            ").grid(row=0, column=5)
        Label(self.guessTab, text="Playing").grid(row=0, column=6)
        self.add_button(self.guessTab, row=1, column=6, text="Correct", event="iccs.got.playing", event_text="correct")
        self.add_button(self.guessTab, row=2, column=6, text="Wrong", event="iccs.got.playing", event_text="wrong")
        self.add_button(self.guessTab, row=3, column=6, text="Wrong-Ask for answer", event="iccs.got.playing", event_text="wrong_ask_for_answer")
        self.add_button(self.guessTab, row=4, column=6, text="Give spatial", event="iccs.got.playing", event_text="give_spatial")
        self.add_button(self.guessTab, row=5, column=6, text="Pointed at object", event="iccs.got.playing", event_text="pointed_object")
        self.add_button(self.guessTab, row=6, column=6, text="Semantic similarity", event="iccs.got.playing", event_text="semantic_similarity")
        self.add_button(self.guessTab, row=7, column=6, text="Long pause", event="iccs.got.playing", event_text="long_pause")

        Label(self.guessTab, text="            ").grid(row=0, column=7)
        Label(self.guessTab, text="Other").grid(row=0, column=8)
        self.add_button(self.guessTab, row=1, column=8, text="Repeat", event="iccs.got.playing", event_text="repeat")
        self.add_button(self.guessTab, row=4, column=8, text="Emergency Exit", event="iccs.got.playing", event_text="emergency_exit")

        Label(self.guessTab, text="            ").grid(row=0, column=9)
        Label(self.guessTab, text="Spatial").grid(row=0, column=10)
        self.add_button(self.guessTab, row=1, column=10, text="Left", event="iccs.got.playing", event_text="spatial_left")
        self.add_button(self.guessTab, row=2, column=10, text="Right", event="iccs.got.playing", event_text="spatial_right")
        self.add_button(self.guessTab, row=3, column=10, text="Behind", event="iccs.got.playing", event_text="spatial_behind")
        self.add_button(self.guessTab, row=4, column=10, text="On table", event="iccs.got.playing", event_text="spatial_table")

    ## RUN PROGRAM ##
    def run(self):
        #Starts and runs the GUI
        self.root.mainloop()

## AUTOSTART ##
if __name__ == "__main__":
    remote = WizardOfOzRemote()
    remote.run()
