def redraw(cards, screen):
    screen.fill((81, 193, 206))
    for key in cards:
        cards[key].draw(screen)

def activate_numbers(cards):
    for key in [0, 1, 2, 3, 4]:
        cards[key].can_open = True

def deactivate_numbers(cards):
    for key in [0, 1, 2, 3, 4]:
        cards[key].can_open = False

def number_to_string(num):
        if num == 0:
            return 'zero'
        elif num == 1:
            return 'one'
        elif num == 2:
            return 'two'
        elif num == 3:
            return 'three'
        elif num == 4:
            return 'four'
        else:
            raise NotImplementedError
