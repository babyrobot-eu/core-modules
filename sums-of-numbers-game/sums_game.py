#!/usr/bin/env python
# -*- coding: utf-8 -*-
import pygame
from pygame.locals import *
from random import shuffle
import os
import time
import pickle

from config import cfg
from events import GAME_EVENT

from card import Card
import card_utils
from sums_game_utils import *

STOP_SUMS = USEREVENT + 2
global previous_time

class SumsGame(object):
    def __init__(self, cfg, screen, gamerunner, wizard_mode=False):
        self.cfg = cfg
        self.screen = screen
        self.gamerunner = gamerunner
        self.wizard_mode = wizard_mode
        self.child_id = 1
    
    def init_game(self):
        global previous_time

        print('Target card: {}, in cardholder: {}'.format(str(self.current_sum[0]), str(self.current_sum[1])))

        # --------------------- Screen related parameters ------------------ #
        self.screen.fill(self.cfg['background_color'])
        pygame.display.set_caption('Sums Game')
        previous_time = -200
        cardback = self.cfg['cardback']

        # Screen size
        w, h = pygame.display.get_surface().get_size()
        print('Screen size: ', w, h)

        # ---------- Card parameters ---------#
        number_of_top_cards = 5
        # Card width
        cw = int(0.12 * w)
        self.cw = cw
        # Card height
        ch = int(0.3 * h)
        self.ch = ch
        # Space between cards
        s = int((0.9 * w - number_of_top_cards * cw) / (number_of_top_cards + 1))
        cards = {}

        # ------------------ TOP CARDS POSITIONS ------------------------ #
        card_posx_list = [0 for i in range(number_of_top_cards)]
        card_posx_list[0] = int(0.05 * w + s)
        for i in range(1, number_of_top_cards):
            card_posx_list[i] = card_posx_list[i - 1] + cw + s
        # shuffle(card_posx_list)

        card_posy = 0.1 * h
        card = Card(0, "sums_game_data/zero.png", cardback,
                    (card_posx_list[0], card_posy), (cw, ch), self.gamerunner)
        card.draw(self.screen)
        cards[0] = card

        card = Card(1, "sums_game_data/one.png", cardback,
                    (card_posx_list[1], card_posy), (cw, ch), self.gamerunner)
        card.draw(self.screen)
        cards[1] = card

        card = Card(2, "sums_game_data/two.png", cardback,
                    (card_posx_list[2], card_posy), (cw, ch), self.gamerunner)
        card.draw(self.screen)
        cards[2] = card

        card = Card(3, "sums_game_data/three.png", cardback,
                    (card_posx_list[3], card_posy), (cw, ch), self.gamerunner)
        card.draw(self.screen)
        cards[3] = card

        card = Card(4, "sums_game_data/four.png", cardback,
                    (card_posx_list[4], card_posy), (cw, ch), self.gamerunner)
        card.draw(self.screen)
        cards[4] = card

        # SECOND LINE
        second_line_h = h - 0.1 * h - ch

        # SYMBOLS SIZES
        cw2 = int(0.5 * cw)
        ch2 = int(0.5 * ch)
        # Distance from the sides = k
        k = int(0.5 * (w - 4 * s - 3 * cw - 2 * cw2))

        # DRAW SYMBOLS
        card2_posx = k + cw + s
        card2_posy = second_line_h + int(0.5 * (cw - cw2))
        card2 = Card("plus", "sums_game_data/plus.png", cardback,
                    (card2_posx, card2_posy), (cw2, ch2), self.gamerunner)
        card2.can_open = False
        card2.draw(self.screen)
        cards['plus'] = card2

        card2_posx = card2_posx + cw2 + 2 * s + cw
        card2 = Card("equal", "sums_game_data/equal.png", cardback,
                    (card2_posx, card2_posy), (cw2, ch2), self.gamerunner)
        card2.can_open = False
        card2.draw(self.screen)
        cards['equal'] = card2

        # DRAW CARDHOLDERS #
        cardholder1_posx = k
        cardholder1_posy = second_line_h
        cardholder1 = Card(HOLDER1, "sums_game_data/equal.png", cardback,
                        (cardholder1_posx, cardholder1_posy), (cw, ch), self.gamerunner,
                        cardholder=True)
        cardholder1.can_open = False
        cardholder1.draw(self.screen)
        

        cardholder2_posx = cardholder1_posx + cw + cw2 + 2 * s
        cardholder2_posy = cardholder1_posy
        cardholder2 = Card(HOLDER2, "sums_game_data/equal.png", cardback,
                        (cardholder2_posx, cardholder2_posy), (cw, ch), self.gamerunner,
                        cardholder=True)
        cardholder2.can_open = False
        cardholder2.draw(self.screen)
        
        if self.current_sum[1] == 1:
            cardholder1.chosen = True
        else:
            cardholder2.chosen = True

        cards[HOLDER1] = cardholder1
        cards[HOLDER2] = cardholder2

        # SECOND SUM CARD #
        first_target = self.current_sum[0]
        if self.current_sum[1] == 1:
            card = Card(first_target, "sums_game_data/{}.png".format(card_utils.number_to_string(first_target)), cardback,
                        (cardholder1_posx, cardholder1_posy), (cw, ch), self.gamerunner)
        else:
            card = Card(first_target, "sums_game_data/{}.png".format(card_utils.number_to_string(first_target)), cardback,
                        (cardholder2_posx, cardholder2_posy), (cw, ch), self.gamerunner)
        card.draw(self.screen)
        card.can_open = False
        card.in_cardholder = 1
        cards['target'] = card

        # LAST CARD 4
        card3_posx = cardholder2_posx + cw + 2 * s + cw2
        card3 = Card(4, "sums_game_data/four.png", cardback,
                    (card3_posx, cardholder2_posy), (cw, ch), self.gamerunner)
        card3.can_open = False
        card3.draw(self.screen)
        cards['target_sum'] = card3

        # CORRECT AND WRONG BOX
        cory = int(h / 2 + (h - 0.2 * h - 2 * ch) / 4)
        corx = k - s
        corw = w - 2 * k + 2 * s
        corh = int(ch + (h - 0.2 * h - 2 * ch) / 2)
        cor = Card("correct", "sums_game_data/equal.png", cardback,
                (corx, cory), (corw, corh), self.gamerunner, cardholder=True, color=cfg['background_color'])
        cor.can_open = False
        cor.draw(self.screen)
        cards['correctwrong_box'] = cor

        # CORRECT SIGN
        corsign_w = int(0.7 * cw2)
        corsign_h = int(0.7 * ch2)
        corsign_x = corx + corw + s
        corsign_y = int(cory + (corh - corsign_h) / 2)
        corsign = Card("correct_sign", cardback, "sums_game_data/correct.png",
                    (corsign_x, corsign_y), (corsign_w, corsign_h), self.gamerunner)
        corsign.can_open = False
        corsign.draw(self.screen)
        cards['correct'] = corsign

        # WRONG SIGN
        wrongsign_x = corx - s - corsign_w
        wrongsign = Card("wrong_sign", cardback, "sums_game_data/wrong.png",
                        (wrongsign_x, corsign_y), (corsign_w, corsign_h), self.gamerunner)
        wrongsign.can_open = False
        wrongsign.draw(self.screen)
        cards['wrong'] = wrongsign

        # GREEN BORDER AROUND SELECTED CARD #
        cardholderc = Card("cardholderc", "sums_game_data/equal.png", cardback,
                        (cards[0].position[0]-5, cards[0].position[1]-5), (cw + 10, ch + 10), self.gamerunner,
                        cardholder=True, color=(0,255,50), bord_size=10)
        cardholderc.can_open = False
        cardholderc.draw(self.screen)
        cards['cardholderc'] = cardholderc
        cards.pop('cardholderc', None)
        redraw(cards, self.screen)
        
        # PASS THE CARDHOLDERS POSITIONS AS PARAMETERS
        for key in cards:
            cards[key].cardholder_positions = ((cardholder1_posx, cardholder1_posy), (cardholder2_posx, cardholder2_posy))

        # ---- BIND TO OBJECT ---- #
        self.cards = cards
        self.cardholderc = cardholderc
        # GAME DICTIONARY
        self.game_dict = {'cardholders_full': 1, 'current_sum': self.current_sum[0],
                        'current_second': self.current_sum[0], 'frozen_cardholder': self.current_sum[1],
                        'cardholderc': cardholderc, 'robot_select': None}
        # Only in wizard mode
        if self.wizard_mode:
            for key in range(cfg['target_sum'] + 1):
                self.cards[key].can_open = True

    def end_game(self):
        self.end_turn()

    def end_turn(self):
        print('Loading child ' + str(self.child_id) + ' data...')
        with open(os.path.join(cfg['pickle_path'], 'child' + str(self.child_id) + '.pkl'), 'rb') as f:
            d = pickle.load(f)
        d['current_sum'] = (d['current_sum'] + 1) % 5
        print('Saving child ' + str(self.child_id) + ' data...')
        with open(os.path.join(cfg['pickle_path'], 'child' + str(self.child_id) + '.pkl'), 'wb') as f:
            pickle.dump(obj=d, file=f)
    
    def read_new_sums_from_pickle(self):
        print('Loading child ' + str(self.child_id) + ' data...')
        with open(os.path.join(cfg['pickle_path'], 'child' + str(self.child_id) + '.pkl'), 'rb') as f:
            d = pickle.load(f)
        self.sum_codes = d['sums'][d['current_sum']]
        # Get sum1 and sum2 tuples of the form (number, cardholder) where cardholder equals 1 or 2
        self.sum1, self.sum2 = get_playing_sums(codes=self.sum_codes)
        self.current_sum = self.sum1

    def process_game_event(self, event):
        # General events
        if event.name == 'athena.games.sums.enablecards':
            card_utils.activate_numbers(self.cards)
        elif event.name == 'athena.games.sums.disablecards':
            card_utils.deactivate_numbers(self.cards)
        # Card selection events
        elif event.name == 'athena.games.sums.robotwrongsum.select':
            open_user_cardholder(self.cards, self.game_dict, self.screen)
            robot_make_wrong_sum(self.cards, self.game_dict, self.screen, number=event.number)
        elif event.name == 'athena.games.sums.robotcorrectsum.select':
            open_user_cardholder(self.cards, self.game_dict, self.screen)
            robot_make_correct_sum(self.cards, self.game_dict, self.screen)
        elif event.name == 'athena.games.sums.robotsum.make':
            robot_put_number(self.cards, self.game_dict, self.screen)
        elif event.name == 'athena.games.sums.resetcardholder':
            open_user_cardholder(self.cards, self.game_dict, self.screen)
        # Turn related events
        elif event.name == 'athena.games.sums.endturn':
            self.end_turn()
        elif event.name == 'athena.games.sums.reloadequations':
            self.read_new_sums_from_pickle()
        # Sums events
        elif event.name == 'athena.games.sums.playwithsum1':
            self.current_sum = self.sum1
            self.init_game()
        elif event.name == 'athena.games.sums.playwithsum2':
            self.current_sum = self.sum2
            self.init_game()
        

        
        # Initialization event
        elif event.name == 'athena.games.sums.initid':
            self.child_id = event.child_id
            self.read_new_sums_from_pickle()
            print('------------ GAME INFO ----------- Child id: {}, Playing sums: {} and {}'.format(str(self.child_id), str(self.sum1), str(self.sum2)))
        
        elif event.name == 'athena.games.sums.selectequations':
            equations = str(self.sum_codes[0]) + ',' + str(self.sum_codes[1])
            self.gamerunner.send_event('athena.games.sums.equations', 'sums_game', equations)


    def process_mouse_event(self):
        try:
            mouse_pos = pygame.mouse.get_pos()
            for key, card in self.cards.iteritems():
                if card.rect.collidepoint(mouse_pos):
                    self.on_card_selected(card)
                    return
        except:
            pass

    def on_card_selected(self, card):
        if card.chosen_once == False and card.can_open and not card.chosen and self.game_dict['cardholders_full'] < 2:
            # user selects card for the first time
            select_number_once(card=card, cards=self.cards, game_dict=self.game_dict, screen=self.screen)
            if self.wizard_mode:
                activate_numbers(self.cards)
        elif card.chosen_once and card.chosen == False and card.can_open:
            # user selects card for the second time (and card goes to bottom)
            select_number(card=card, cards=self.cards, game_dict=self.game_dict, screen=self.screen)
            if self.wizard_mode:
                activate_numbers(self.cards)
        elif card.chosen and card.can_open:
            # user selects a card from the bottom (and card goes back up)
            deselect_number(card=card, cards=self.cards, game_dict=self.game_dict, screen=self.screen)
            if self.wizard_mode:
                activate_numbers(self.cards)
