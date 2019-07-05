import time
import pygame
from pygame.locals import *

HOLDER1 = 'cardholder1'
HOLDER2 = 'cardholder2'

class Card(object):
    def __init__(self, number, picture, cardback, position, cardsize, gamerunner, cardholder=False,
                 color=(200, 200, 200), bord_size=3):
        self.number = number
        self.picture = picture
        self.cardback = cardback
        self.position = position
        self.cardsize = cardsize
        self.gamerunner = gamerunner
        self.chosen = False
        self.chosen_once = False
        self.can_open = False  # Start deactivated
        self.cardholder = cardholder
        self.color = color

        self.back = pygame.image.load(self.cardback)
        self.back = pygame.transform.scale(self.back, (self.cardsize[0], self.cardsize[1]))
        self.img = pygame.image.load(self.picture)
        self.img = pygame.transform.scale(self.img, (self.cardsize[0], self.cardsize[1]))
        self.cardholder_positions = ((0, 0), (0, 0))
        self.in_cardholder = 0
        self.bord_size = bord_size

        self.current_position = self.position

    def draw(self, screen):
        if self.cardholder:
            pygame.draw.rect(screen, self.color, Rect([self.position[0], self.position[1]], self.cardsize), self.bord_size)
            pygame.display.flip()
            self.rect = Rect(self.current_position[0], self.current_position[1], self.cardsize[0], self.cardsize[1])
            self.screen = screen
        else:
            screen.blit(self.img, self.current_position)
            pygame.display.flip()
            # print(self.position)
            self.rect = Rect(self.current_position[0], self.current_position[1], self.cardsize[0], self.cardsize[1])
            self.screen = screen

    def chose_number(self, i):
        self.current_position = self.cardholder_positions[i]
        self.in_cardholder = i

    def reset_number(self):
        self.current_position = self.position
        i = self.in_cardholder
        self.in_cardholder = 0
        return i

    def clear(self):
        img = pygame.image.load('sums_game_data/green.png')
        self.screen.blit(img, self.position)
        pygame.display.flip()

    def toggle(self):
        tmp = self.back
        self.back = self.img
        self.img = tmp

    def toggle_hidden(self):
        self.toggle()
        self.chosen = not self.chosen

    def hide_if_active(self):
        if self.chosen:
            self.toggle_hidden()
