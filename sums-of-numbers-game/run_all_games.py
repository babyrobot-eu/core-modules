import os
import sys
from sums_game import SumsGame
from emorec_game import EmorecGame
import pygame
from pygame.locals import *
import socket
import threading
import json
from events import *
from config import cfg
import ast

class GameRunner(object):
    """docstring for GameRunner"""

    def __init__(self, wizard_mode):
        super(GameRunner, self).__init__()
        self.wizard_mode = wizard_mode
        pygame.init()
        self.infoObject = pygame.display.Info()
        if cfg['screen_size']:
            screen_size = cfg['screen_size']
        else:
            screen_size = (self.infoObject.current_w, self.infoObject.current_h)
        self.screen = pygame.display.set_mode(screen_size, pygame.RESIZABLE)
        bg = pygame.image.load(cfg['cardback'])
        self.bg = pygame.transform.scale(bg, screen_size)
        self.event_id = 1

        if not wizard_mode:
            self.t1 = threading.Thread(target=self.connect_to_broker)
            self.t1.daemon = True
            self.t1.start()
            self.connected = True
            self.run_games()
        else:
            self.connected = False
            self.run_games()

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

    # ------------- THREAD 1 ------------- #
    def connect_to_broker(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        ipaddress = cfg['broker']
        port = 1932
        # Connect the socket to the port where the server is listening
        server_address = (ipaddress, port)
        self.sock.connect(server_address)
        message = 'CONNECT furhat games \n'
        self.sock.sendall(message)

        l = self.sock.recv(8192)
        print l

        message = 'SUBSCRIBE athena.games.** \n'
        self.sock.sendall(message)
        jsonflag = False
        jsonevent = ''
        for line in self.readlines(self.sock):
            print('line: ', line)
            if line.startswith("EVENT") and not jsonflag:
                data_arr = line.split()
                event_name = data_arr[1]
                # ----------- MAIN EVENTS ---------- #
                if event_name == "athena.games.sums.masterstart":
                    pygame.event.post(pygame.event.Event(START_SUMS, {}))
                elif event_name == "athena.games.sums.stop":
                    pygame.event.post(pygame.event.Event(STOP_SUMS, {}))

                # ------------ GAME EVENTS ---------- #
                elif event_name == 'athena.games.sums.enablecards':
                    pygame.event.post(pygame.event.Event(GAME_EVENT, name='athena.games.sums.enablecards'))
                elif event_name == 'athena.games.sums.disablecards':
                    pygame.event.post(pygame.event.Event(GAME_EVENT, name='athena.games.sums.disablecards'))
                elif event_name == 'athena.games.sums.robotwrongsum.select':
                    jsonflag = True
                    jsonevent = event_name
                elif event_name == 'athena.games.sums.robotcorrectsum.select':
                    pygame.event.post(pygame.event.Event(GAME_EVENT, name='athena.games.sums.robotcorrectsum.select'))
                elif event_name == 'athena.games.sums.robotsum.make':
                    pygame.event.post(pygame.event.Event(GAME_EVENT, name='athena.games.sums.robotsum.make'))
                elif event_name == 'athena.games.sums.resetcardholder':
                    pygame.event.post(pygame.event.Event(GAME_EVENT, name='athena.games.sums.resetcardholder'))
                # Turn events
                elif event_name == 'athena.games.sums.endturn':
                    pygame.event.post(pygame.event.Event(GAME_EVENT, name='athena.games.sums.endturn'))
                elif event_name == 'athena.games.sums.reloadequations':
                    pygame.event.post(pygame.event.Event(GAME_EVENT, name='athena.games.sums.reloadequations'))
                # Sums events
                elif event_name == 'athena.games.sums.playwithsum1':
                    pygame.event.post(pygame.event.Event(GAME_EVENT, name='athena.games.sums.playwithsum1'))
                elif event_name == 'athena.games.sums.playwithsum2':
                    pygame.event.post(pygame.event.Event(GAME_EVENT, name='athena.games.sums.playwithsum2'))
                
                # Initialization event
                elif event_name == 'athena.games.sums.initid':
                    jsonflag = True
                    jsonevent = event_name
                # 
                elif event_name == 'athena.games.sums.selectequations':
                    pygame.event.post(pygame.event.Event(GAME_EVENT, name='athena.games.sums.selectequations'))

                # ---------------- EMOREC ---------------- #
                elif event_name == 'athena.games.emorec.clearcards':
                    pygame.event.post(pygame.event.Event(GAME_EVENT, name='athena.games.emorec.clearcards'))
                elif event_name == 'athena.games.emorec.showhappiness':
                    pygame.event.post(pygame.event.Event(GAME_EVENT, name='athena.games.emorec.showhappiness'))
                elif event_name == 'athena.games.emorec.showsadness':
                    pygame.event.post(pygame.event.Event(GAME_EVENT, name='athena.games.emorec.showsadness'))
            
            elif jsonflag:
                if jsonevent == 'athena.games.sums.robotwrongsum.select':
                    jsonflag = False
                    dct = ast.literal_eval(line)
                    text = dct['text']
                    number = int(text)
                    pygame.event.post(pygame.event.Event(GAME_EVENT, name='athena.games.sums.robotwrongsum.select', number=number))
                elif jsonevent == 'athena.games.sums.initid':
                    jsonflag = False
                    dct = ast.literal_eval(line)
                    text = dct['text']
                    child_id = int(text[0])
                    pygame.event.post(pygame.event.Event(GAME_EVENT, name='athena.games.sums.initid', child_id=child_id))

    def send_event(self, event_name, sender, text=None):
        if not self.connected:
            return
        data = {}
        data["class"] = "iristk.system.Event"
        data["event_name"] = event_name
        if text:
            data["text"] = text
        data["event_sender"] = sender
        data["event_id"] = "gamerunner" + str(self.event_id)

        json_data = json.dumps(data)

        message = "EVENT " + event_name + " " + str(len(json_data)) + " \n"
        # print message
        # print json_data
        self.sock.sendall(message)
        self.sock.sendall(json_data)

        self.event_id += 1

    def screensaver(self):
        self.screen.fill(cfg['background_color'])
        self.screen.blit(self.bg, (0, 0))
        pygame.display.flip()

    def clear_screen(self):
        self.screen.fill(cfg['background_color'])
        pygame.display.flip()

    # ------------- THREAD 2 --------------- #
    def run_games(self):
        self.screensaver()
        clock = pygame.time.Clock()

        sumsgame = SumsGame(cfg=cfg, screen=self.screen, gamerunner=self, wizard_mode=self.wizard_mode)
        emorecgame = EmorecGame(cfg=cfg, screen=self.screen, gamerunner=self)
        game_state = 'idle'
        while True:
            clock.tick(30)
            for event in pygame.event.get():
                if self.wizard_mode:
                    if event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE and game_state == 'idle':
                        sumsgame.read_new_sums_from_pickle()
                        sumsgame.init_game()
                        game_state = 'running'
                        continue
                    if event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE and game_state == 'running':
                        sumsgame.end_game()
                        self.clear_screen()
                        game_state = 'idle'
                        continue
                    if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                        exit(0)
                    if event.type == QUIT:
                        return
                else:
                    if event.type == START_SUMS and game_state == 'idle':
                        game_state = 'running'
                    if event.type == STOP_SUMS and game_state == 'running':
                        self.clear_screen()
                        sumsgame.end_game()
                        game_state = 'idle'
                    if event.type == GAME_EVENT and game_state == 'running':
                        sumsgame.process_game_event(event)
                    if event.type == QUIT:
                        return
                if game_state == 'running' and event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                    sumsgame.process_mouse_event()
                
                # ------------- EMOREC -------------- #
                if event.type == GAME_EVENT:
                    emorecgame.process_game_event(event)

if __name__ == '__main__':
    if sys.argv[-1] == 'woz':
        game_runner = GameRunner(wizard_mode=True)
    else:
        game_runner = GameRunner(wizard_mode=False)
