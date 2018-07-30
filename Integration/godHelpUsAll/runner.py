import threading
from server import Server
from test   import Test



threads = []
threads.append(threading.Thread(target=Server))
threads.append(threading.Thread(target=Test))
threads[0].start()
threads[1].start()