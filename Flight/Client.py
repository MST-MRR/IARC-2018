import os
import asyncore
import logging
import socket
import cv2
import json
from base64 import b64encode
from multiprocessing import Process, JoinableQueue, Event
from multiprocessing.queues import Empty

logging.basicConfig(format='[%(levelname)s][thread %(thread)d at %(asctime)s]: %(message)s', level=logging.INFO)

HOST = '127.0.0.1'
PORT = 8765

TEST_VIDEO_PATH = os.path.join(os.getcwd(), 'ricky.mp4')

class Client(asyncore.dispatcher):
    def __init__(self, queue, client_running, stop_event, host=HOST, port=PORT, chunk_size=8192):
        asyncore.dispatcher.__init__(self)
        self._client_running = client_running
        self._stop_event = stop_event
        self._queue = queue
        self._chunk_size = chunk_size
        self._buffer = ''
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.address = (host, port)
        logging.debug('Client connecting to %s' % (self.address,))
        self.connect(self.address)

    def handle_connect(self):
        logging.debug('Client connected to server.')

    def handle_close(self):
        logging.debug('Closing connection to server.')
        self.close()
        self._client_running.clear()

    def readable(self):
        return False

    def handle_write(self):
        if self._stop_event.is_set(): return
        logging.debug('handle_write()')
        if not len(self._buffer):
            while True:
                try:
                    self._buffer = self._queue.get(timeout=5) + '\0'
                except Empty:
                    logging.debug('Empty')
                    if self._stop_event.is_set():
                        logging.debug('Client stopped.')
                        self.handle_close()
                        return
                    continue
                else:
                    break

        sent = self.send(self._buffer[:self._chunk_size])
        self._buffer = self._buffer[sent:]
        if not bool(self._buffer):
            self._queue.task_done()

queue = JoinableQueue()
client_running = Event()
stop_event = Event()

def run_client(queue, client_running, stop_event):
    if not client_running.is_set():
        client = Client(queue, client_running, stop_event)
        logging.debug('Setting client_running event.')
        client_running.set()
        asyncore.loop()
        exit(0)

def run_client_async(queue=queue, client_running=client_running, stop_event=stop_event):
    if not client_running.is_set():
        process = Process(target=run_client, args=(queue, client_running, stop_event))
        process.start()
        client_running.wait()

def stop_client_async():
    global stop_event
    logging.debug('Stopping client...')
    stop_event.set()

def send_message(msg):
    global queue
    global client_running
    queue.put(msg)
    if client_running.is_set():
        queue.join()

if __name__ == '__main__':
    cap = cv2.VideoCapture(TEST_VIDEO_PATH)
    run_client_async(queue, client_running, stop_event)

    try:
        while True:
            ret, frame = cap.read()
            if (frame is None): break
            json_message = json.dumps([{'img': b64encode(cv2.imencode('.jpeg', frame)[1].tostring())}])
            send_message(json_message)
            
    finally:
        cap.release()
        cv2.destroyAllWindows()

    stop_client_async()