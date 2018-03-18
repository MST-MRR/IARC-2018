import os
import sys
import pexpect
import threading 
import traceback
import signal
import psutil
import re
import time

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
DISCONNECT_KEYWORDS = 'no link'
EXCEPTION_KEYWORDS = 'Exception'

TIMEOUT = .001
READY_TO_FLY_TIMEOUT = 180

EOL = re.compile('\r\n')

_DEBUG = True

class QuadcopterCrash(Exception):
    pass

class ArdupilotGazeboMonitor():
    def __init__(self, speedup=1):
        self.speedup = speedup
        self._ready_to_fly = threading.Event()
        self._gps_glitch = threading.Event()
        self._has_reset = False
        self._stop = False
        self._proc = None

    def __enter__(self):
        self._start(self.speedup)
        return self
    
    def __exit__(self, *args):
        self._stop = True
        self.stop()
    
    def _start(self, speedup=None):
        if speedup is None:
            speedup = self._speedup
        else:
            self._speedup = speedup
        
        ardupilot_start_command = ARDUPILOT_START_COMMAND_FMT_STRING % (ARDUPILOT_STARTUP_SCRIPT, self._speedup)
        self._proc = pexpect.spawn(ardupilot_start_command, cwd=ARDUCOPTER_DIRECTORY)
        self._has_reset = False

    @property
    def speedup(self):
        return self._speedup

    @speedup.setter
    def speedup(self, value):
        if value < 1 or not isinstance(value, int):
            raise ValueError(INVALID_SPEEDUP_VALUE_ERROR_STR)
        
        self._speedup = value

    def restart(self):
        self._stop = True
        self.stop()
        self._start(self.speedup)

    def reset(self):
        self._ready_to_fly.clear()
    
    def wait_for_ready_to_fly(self, timeout=READY_TO_FLY_TIMEOUT):
        return self._ready_to_fly.wait(timeout=timeout)

    def stop(self, force=False):
        if force or (not self._has_reset and self._stop and (self._proc is not None and self._proc.isalive())):
            if self._proc.isalive():
                self._proc.kill(signal.SIGKILL)
                self._proc.close()


            for proc in psutil.process_iter():
                try:
                    if proc.name() in ['xterm', 'mavproxy.py', 'arducopter']:
                        proc.kill()
                except:
                    pass

            self._has_reset = True
            self._stop = False

    def is_ready(self):
        return self._ready_to_fly.is_set() or (self._proc is not None and self._proc.isalive() and not self._has_reset and not self._stop)

    def update(self):
        if self._proc is not None and self._proc.isalive() and not self._has_reset:
            next_line_of_output = None
            cases = [EOL, pexpect.TIMEOUT, pexpect.EOF]
            index = self._proc.expect_list(cases, timeout=TIMEOUT)
            
            if not index:
                self._process(self._proc.before)
        else:
            self._ready_to_fly.clear()
            
        return self.is_ready()

    def _process(self, line):
        if isinstance(line, tuple):
            line = line[0]
        
        if line:
            if _DEBUG:
                sys.stdout.write(ARDUPILOT_LOG_FMT_STRING % (line,))

            if READY_TO_FLY_KEYWORDS in line:
                self._ready_to_fly.set()
            elif CRASH_KEYWORDS in line:
                raise QuadcopterCrash()
            elif GPS_GLITCH_CLEARED_KEYWORDS in line:
                self._ready_to_fly.set()
                self._gps_glitch.clear()
            elif GPS_GLITCH_KEYWORDS in line:
                self.reset()
                self._gps_glitch.set()
            elif DISCONNECT_KEYWORDS in line:
                self._stop = True
                self.reset()
            elif EXCEPTION_KEYWORDS in line:
                self._stop = True
                self.reset()
    
    def flush(self):
        while self.is_ready():
            try:
                next_line_of_output = None
                cases = [EOL, pexpect.TIMEOUT, pexpect.EOF]
                index = self._proc.expect_list(cases, timeout=TIMEOUT)
                line = self._proc.before

                if index == 1:
                    break
                if line:
                    self._process(line)
            except:
                break

        return self.is_ready()
            

    def wait_for_gps_glitch(self, timeout=1):
        return self._gps_glitch.wait(timeout=timeout)

if __name__ == '__main__':
    try:
        with ArdupilotGazeboMonitor() as monitor:
            while monitor.update():
                pass

    except KeyboardInterrupt as e:
        pass
            