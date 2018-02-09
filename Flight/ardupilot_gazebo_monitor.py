import os
import sys
import pexpect
import threading 
import traceback
import signal
import psutil
import re

ARDUCOPTER_DIRECTORY = os.path.expanduser('~/ardupilot/ArduCopter')
ARDUPILOT_STARTUP_SCRIPT = 'sim_vehicle.py'
ARDUPILOT_START_COMMAND_FMT_STRING = '%s -f gazebo-iris -S %d'
ARDUPILOT_LOG_FMT_STRING = '[ArduPilot Log]: %s\n'
ARDUPILOT_DISCONNECT_ERROR_STRING = '[ArduPilot Error]: ArduPilot terminated unexpectedly!'

INVALID_SPEEDUP_VALUE_ERROR_STR = 'ArdupilotGazeboMonitor(): speedup must be an integer > 1'

READY_TO_FLY_KEYWORDS = 'using GPS'
GPS_GLITCH_KEYWORDS = 'APM: GPS Glitch'
GPS_GLITCH_CLEARED_KEYWORDS = 'APM: GPS Glitch cleared'
CRASH_KEYWORDS = 'Crash: Disarming'

ON_POSIX = 'posix' in sys.builtin_module_names

TIMEOUT = .1

EOL = re.compile('\r\n')

_DEBUG = True

class ArdupilotDisconnect(Exception):
    pass

class QuadcopterCrash(Exception):
    pass

class ArdupilotGazeboMonitor():
    def __init__(self, speedup=1):
        self.speedup = speedup
        self._stop_event = threading.Event()
        self._ready_to_fly = threading.Event()
        self._gps_glitch = threading.Event()
        self._exception_lock = threading.Lock()
        self._exception = None
        self._start(speedup)
    
    def _start(self, speedup):
        ardupilot_start_command = ARDUPILOT_START_COMMAND_FMT_STRING % (ARDUPILOT_STARTUP_SCRIPT, speedup)
        self._proc = pexpect.spawn(ardupilot_start_command, cwd=ARDUCOPTER_DIRECTORY)

        self._thread = threading.Thread(target=self._event_thread, args=(self._proc, self._stop_event))
        self._thread.daemon = True
        self._thread.start()

    @property
    def speedup(self):
        return self._speedup

    @speedup.setter
    def speedup(self, value):
        if value < 1 or not isinstance(value, int):
            raise ValueError(INVALID_SPEEDUP_VALUE_ERROR_STR)
        
        self._speedup = value

    def restart(self):
        self.stop()
        self._start(self.speedup)

    def reset(self):
        self._ready_to_fly.clear()
    
    def ready_to_fly(self, block=False):
        state = False
        
        if not block:
            state = self._ready_to_fly.is_set()
        else:
            self._ready_to_fly.wait()
            state = True
        
        return state

    def stop(self):
        if self.is_alive():
            self._stop_event.set()
            self._thread.join()
            self._stop_event.clear()
    
    def is_alive(self):
        return self._thread.is_alive()

    def update(self):
        with self._exception_lock:
            if self._exception is not None:
                raise self._exception
        
    def _raise(self, exception):
        with self._exception_lock:
            if self._exception is None:
                self._exception = exception
                self._stop_event.set()

    def _process(self, line):
        if isinstance(line, tuple):
            line = line[0]
        
        if line:
            if _DEBUG:
                sys.stdout.write(ARDUPILOT_LOG_FMT_STRING % (line,))

            if READY_TO_FLY_KEYWORDS in line:
                self._ready_to_fly.set()
            elif CRASH_KEYWORDS in line:
                self._raise(QuadcopterCrash())
            elif GPS_GLITCH_CLEARED_KEYWORDS in line:
                self._ready_to_fly.set()
                self._gps_glitch.clear()
            elif GPS_GLITCH_KEYWORDS in line:
                self._ready_to_fly.clear()
                self._gps_glitch.set()

    def wait_for_gps_glitch(self, timeout=1):
        return self._gps_glitch.wait(timeout=timeout)
    
    def _event_thread(self, process, stop_event):
        try:
            cases = [EOL, pexpect.EOF, pexpect.TIMEOUT]
            next_line_of_output = None
            
            while process.isalive() and (not stop_event.is_set()):
                index = process.expect_list(cases, timeout=TIMEOUT)

                if not index:
                    self._process(process.before)
        
        finally:
            process.kill(signal.SIGKILL)
            process.close()

            for proc in psutil.process_iter():
                if proc.name() == 'xterm':
                    proc.kill()
        
        if not stop_event.is_set():
            self._raise(ArdupilotDisconnect(ARDUPILOT_DISCONNECT_ERROR_STRING))