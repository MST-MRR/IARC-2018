import pickle
import requests
from PIL import Image
import json

im = Image.open("image2.jpg")

d = {
    'image':pickle.dumps(im)
}

print("Sending serialized image...")

r = requests.post('http://localhost:5000/api/image', data=json.dumps(d))

r = requests.get('http://localhost:5000/api/image')

print("Received image response. Deserializing...")

json_req = json.loads(r.content.decode('utf-8'))

print("Showing image...")

image = pickle.loads(json_req['image'])

image.show()