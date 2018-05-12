import threading
import time

import pygame
pygame.init()
pygame.joystick.init()

class XboxOneController():
    A_BUTTON_ID = 0
    B_BUTTON_ID = 1
    X_BUTTON_ID = 2
    Y_BUTTON_ID = 3
    LEFT_BUMPER_BUTTON_ID = 4
    RIGHT_BUMPER_BUTTON_ID = 5
    VIEW_BUTTON_ID = 6
    MENU_BUTTON_ID = 7
    XBOX_BUTTON_ID = 8
    LEFT_ANALOG_BUTTON_ID = 9
    RIGHT_ANALOG_BUTTON_ID = 10

    LEFT_JOYSTICK_HORIZONTAL_AXIS_ID = 0
    LEFT_JOYSTICK_VERTICAL_AXIS_ID = 1
    LEFT_TRIGGER_AXIS_ID = 2
    RIGHT_JOYSTICK_HORIZONTAL_AXIS_ID = 3
    RIGHT_JOYSTICK_VERTICAL_AXIS_ID = 4
    RIGHT_TRIGGER_AXIS_ID = 5

    DPAD_HAT_ID = 0

    def __init__(self):
        self._joystick = None
        self._quit = False

        for i in range(pygame.joystick.get_count()):
            joystick = pygame.joystick.Joystick(i)
            joystick.init()

            if 'x-box' in joystick.get_name().lower():
                self._joystick = joystick
                break

            joystick.quit()
        
        assert self._joystick != None, 'XboxOneController(): No Xbox One controller could be found!'
        self._listeners = {}
        self._lock = threading.Lock()
        self._blacklist = (pygame.KEYDOWN, pygame.KEYUP, pygame.MOUSEMOTION, pygame.MOUSEBUTTONUP, pygame.MOUSEBUTTONDOWN, 
                           pygame.JOYBALLMOTION)

        self._button_map = {value: key for key, value in XboxOneController.__dict__.items() if 'button' in key.lower()}
        self._axis_map = {value: key for key, value in XboxOneController.__dict__.items() if 'axis' in key.lower()}
        self._hat_map = {value: key for key, value in XboxOneController.__dict__.items() if 'hat' in key.lower()}

        pygame.event.set_blocked(self._blacklist)

        event_thread = threading.Thread(target=self._event_thread)
        event_thread.daemon = True
        event_thread.start()

    def _event_thread(self):
        while not self._quit:
            events = [pygame.event.wait()]
            events.extend(pygame.event.get())

            with self._lock:
                callbacks, args = (None, None)

                for event in events:
                    if event.type in (pygame.JOYBUTTONUP, pygame.JOYBUTTONDOWN):
                        callbacks = self._listeners.get(self._button_map[event.button])
                        args = (event.type,)
                    elif event.type == pygame.JOYAXISMOTION:
                        callbacks = self._listeners.get(self._axis_map[event.axis])
                        args = (event.value,)
                    elif event.type == pygame.JOYHATMOTION:
                        callbacks = self._listeners.get(self._hat_map[event.hat])
                        args = (event.value,)

                    if callbacks is not None:
                        for callback in callbacks:
                            callback(*args)
    
    def add_button_listener(self, event_type, callback):
        with self._lock:
            key = self._button_map[event_type]

            if self._listeners.get(key) is None:
                self._listeners[key] = []

            self._listeners[key].append(callback)
    
    def add_axis_listener(self, event_type, callback):
        with self._lock:
            key = self._axis_map[event_type]

            if self._listeners.get(key) is None:
                self._listeners[key] = []

            self._listeners[key].append(callback)

    def get_button_state(self, button_id):
        return self._joystick.get_button(button_id)
    
    def get_axis_state(self, axis_id):
        return self._joystick.get_axis(axis_id)

    def quit():
        with self._lock:
            pygame.quit()
            self._quit = True