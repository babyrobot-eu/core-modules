import random
import time
import pygame.time

from card_utils import deactivate_numbers, redraw, activate_numbers

HOLDER1 = 'cardholder1'
HOLDER2 = 'cardholder2'


def select_number_once(card, cards, game_dict, screen):
    flag = False
    for key in [0, 1, 2, 3, 4]:
        if cards[key].chosen_once:
            cards[key].chosen_once = False
            flag = True
            card.chosen_once = True
            redraw(cards, screen)
            game_dict['cardholderc'].position = (card.position[0]-5, card.position[1]-5)
            game_dict['cardholderc'].draw(screen)
    if not flag:
        card.chosen_once = True
        game_dict['cardholderc'].position = (card.position[0]-5, card.position[1]-5)
        game_dict['cardholderc'].draw(screen)

def select_number(card, cards, game_dict, screen):
    if game_dict['cardholders_full'] == 0:
        card.chosen = True
        card.chose_number(0)
        cards[HOLDER1].chosen = True  # Card in first cardholder
        game_dict['cardholders_full'] += 1
        game_dict['current_sum'] += card.number
        # --- SELECT FIRST CARD --- #
        card.gamerunner.send_event('athena.games.sums.card_selected', 'sums_game', str(card.number))
    elif game_dict['cardholders_full'] == 1 and cards[HOLDER1].chosen:
        card.chosen = True
        card.chose_number(1)
        cards[HOLDER2].chosen = True  # Card in second cardholder
        game_dict['user_card'] = card
        game_dict['cardholders_full'] += 1
        game_dict['current_sum'] += card.number
        card.gamerunner.send_event('athena.games.sums.card_selected', 'sums_game', str(card.number))
    elif game_dict['cardholders_full'] == 1 and cards[HOLDER2].chosen:
        card.chosen = True
        card.chose_number(0)
        cards[HOLDER1].chosen = True  # Card in first cardholder
        game_dict['user_card'] = card
        game_dict['cardholders_full'] += 1
        game_dict['current_sum'] += card.number
        card.gamerunner.send_event('athena.games.sums.card_selected', 'sums_game', str(card.number))
    else:
        print('Cardholders full!')
    if game_dict['cardholders_full'] == 2:  # Calculate sum and evaluate
        if game_dict['current_sum'] == 4:
            cards['correctwrong_box'].color = (0, 200, 0)
            cards['correct'].toggle_hidden()
            card.gamerunner.send_event('athena.games.sums.sumcorrect', 'sums_game', text='child')
            deactivate_numbers(cards=cards)
            print('Correct!')
        else:
            cards['correctwrong_box'].color = (200, 0, 0)
            cards['wrong'].toggle_hidden()
            card.gamerunner.send_event('athena.games.sums.sumwrong', 'sums_game', text='child')
            deactivate_numbers(cards=cards)
            print('Wrong!')
    redraw(cards, screen)

def deselect_number(card, cards, game_dict, screen):
    if game_dict['cardholders_full'] > 0:
        card.chosen = False
        card.chosen_once = False
        i = card.reset_number()
        if i == 0:
            cards[HOLDER1].chosen = False  # Card removed from first cardholder
        else:
            cards[HOLDER2].chosen = False  # Card removed from second cardholder
        game_dict['cardholders_full'] -= 1
        game_dict['current_sum'] -= card.number
        cards['correctwrong_box'].color = (81, 193, 206)
        cards['correct'].hide_if_active()
        cards['wrong'].hide_if_active()
        redraw(cards, screen)
    else:
        print('All cardholders empty!')

def open_user_cardholder(cards, game_dict, screen):
    if game_dict['frozen_cardholder'] == 1:
        if cards[HOLDER2].chosen:
            card = game_dict['user_card']
            card.chosen = False
            card.chosen_once = False
            i = card.reset_number()
            cards[HOLDER2].chosen = False  # Card removed from first cardholder
            game_dict['cardholders_full'] -= 1
            game_dict['current_sum'] -= card.number
            cards['correctwrong_box'].color = (81, 193, 206)
            cards['correct'].hide_if_active()
            cards['wrong'].hide_if_active()
            activate_numbers(cards)
            redraw(cards, screen)
    else:
        if cards[HOLDER1].chosen:
            card = game_dict['user_card']
            card.chosen = False
            card.chosen_once = False
            i = card.reset_number()
            cards[HOLDER1].chosen = False  # Card removed from first cardholder
            game_dict['cardholders_full'] -= 1
            game_dict['current_sum'] -= card.number
            cards['correctwrong_box'].color = (81, 193, 206)
            cards['correct'].hide_if_active()
            cards['wrong'].hide_if_active()
            activate_numbers(cards)
            redraw(cards, screen)

def robot_make_wrong_sum(cards, game_dict, screen, number):
    select_number_once(cards[number], cards, game_dict, screen)
    game_dict['robot_select'] = number
    deactivate_numbers(cards)
    cards[0].gamerunner.send_event('athena.games.sums.robotsumready', 'sums_game', text=str(number))

def robot_make_correct_sum(cards, game_dict, screen):
    card_numbers = [0, 1, 2, 3, 4]
    second = game_dict['current_second']
    for i in card_numbers:
        if i + second == 4:
            select_number_once(cards[i], cards, game_dict, screen)
            game_dict['robot_select'] = i
            break
    deactivate_numbers(cards)
    cards[0].gamerunner.send_event('athena.games.sums.robotsumready', 'sums_game', text=str(i))

def robot_put_number(cards, game_dict, screen):
    select_number(cards[game_dict['robot_select']], cards, game_dict, screen )
    print(game_dict['current_sum'])
    # if game_dict['current_sum'] != 4:
    #     cards[0].gamerunner.send_event('athena.games.sums.sumwrong', 'sums_game', text='robot')
    # else:
    #     cards[0].gamerunner.send_event('athena.games.sums.sumcorrect', 'sums_game', text='robot')

def get_playing_sums(codes):
    code1 = codes[0]
    code2 = codes[1]
    tmp = {5: (0, 1), 105: (1, 1), 205: (2, 1), 305: (3, 1), 405: (4, 1),
           1005: (0, 2), 1105: (1, 2), 1205: (2, 2), 1305: (3, 2), 1405: (4, 2)}
    return tmp[code1], tmp[code2]

