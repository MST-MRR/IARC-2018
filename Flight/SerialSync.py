import serial as s
from time import sleep
class serialSync:
	@staticmethod
	def send(angle):
		he = s.Serial('/de/serial/by-id/usb-Arduino_LLC_Arduino_Leonardo-if00', 9600, timeout=1)
		he.write(str(angle))
		he.close()
		sleep(1.1)