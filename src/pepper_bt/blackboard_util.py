#!/usr/bin/env python
from enum import Enum


class BlackboardItems(Enum):
    SPEAKING_IS_RUNNING = "speaking_is_running"
    TABLET_IS_SHOWING_PAINT = "tablet_is_showing_paint"

    PAINTING_NAME = "painting_name"
    PAINTING_DESCRIBTION = "painting_describtion"

    USER_ENGAGED = "user_engaged"
    PAIN_IS_CHOSEN = "paint_is_chosen"
    SELECTED_PAINTING = "selected_painting"

    USERTURN = "User is Allowed Turn"


class Backchannel(Enum):

    CONFIRM = "Ummm"