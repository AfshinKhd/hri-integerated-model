#!/usr/bin/env python
from enum import Enum



class BlackboardItems(Enum):
    SPEAKING_IS_RUNNING = "speaking_is_running"
    TABLET_IS_SHOWING_PAINT = "tablet_is_showing_paint"

    PAINTING_DESCRIBTION = "painting_describtion"

    USER_ENGAGED = "user_engaged"
    PAIN_IS_CHOSEN = "paint_is_chosen"
    SELECTED_PAINTING = "selected_painting"

    USERTURN = "User is Allowed Turn"
    ROBOT_IS_SPEAKING = "robot_is_speaking"

    LAST_FURTHER = "last_further"


class Backchannel(Enum):

    CONFIRM = "Umm"


def get_next_tag(previous_tag, new_series=False):
    """
    This function get a tag of speech as previous tag and give back new tag.
    
    param: new_series: give new tag and change series of alphabet
    """
    if not previous_tag:
        return 'a1'
    
    alphabet = previous_tag[0]
    number = int(previous_tag[1:])
    
    
    if new_series:
        next_alphabet = chr(ord(alphabet) + 1)
        next_tag = '{}01'.format(next_alphabet)
    else:
        next_number = number + 1
        next_tag = '{}{:02d}'.format(alphabet, next_number)
    

    
    return next_tag


file_path = "speech_data.txt"

def save_speech_data(speech_data):
    with open(file_path, "w") as text_file:
        for data_dict in speech_data:
            data_str = str(data_dict) + "\n"
            text_file.write(data_str)

def load_speach_data():
    loaded_speech_data = []
    with open(file_path, "r") as text_file:
        for line in text_file:
            data_dict = eval(line)  
            loaded_speech_data.append(data_dict)

