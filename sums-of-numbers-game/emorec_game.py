#!/usr/bin/env python
# -*- coding: utf-8 -*-
import pygame
from pygame.locals import *
import os
import time

from config import cfg
from events import GAME_EVENT
from card import Card

class EmorecGame(object):
    def __init__(self, cfg, screen, gamerunner):
        self.cfg = cfg
        self.screen = screen
        self.gamerunner = gamerunner
        # Screen size
        self.w, self.h = pygame.display.get_surface().get_size()
        self.cw = int(0.2 * self.w)
        self.ch = int(0.5 * self.h)
        self.card_posx = int((self.w - self.cw) / 2)
        self.card_posy = int((self.h - self.ch) / 2)
    
    def process_game_event(self, event):
        if event.name == 'athena.games.emorec.clearcards':
            self.clear_cards()
        elif event.name == 'athena.games.emorec.showhappiness':
            self.show_happiness_card()
        elif event.name == 'athena.games.emorec.showsadness':
            self.show_sadness_card()

    def show_happiness_card(self):
        pygame.display.set_caption('Emorec Game')
        cardback = self.cfg['cardback']

        card = Card(0, self.cfg['happy_card'], cardback,
                    (self.card_posx, self.card_posy), (self.cw, self.ch), self.gamerunner)
        card.draw(self.screen)

    def show_sadness_card(self):
        pygame.display.set_caption('Emorec Game')
        cardback = self.cfg['cardback']

        card = Card(1, self.cfg['sad_card'], cardback,
                    (self.card_posx, self.card_posy), (self.cw, self.ch), self.gamerunner)
        card.draw(self.screen)

    def clear_cards(self):
        pygame.display.set_caption('Emorec Game')
        self.screen.fill(cfg['background_color'])
        pygame.display.flip()
        
