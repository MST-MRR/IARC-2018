import falcon
import json

class ImageHandler:

    image = None
    
    def on_get(self, req, resp):
        ret = {
            'image':self.image
        }
        resp.body = json.dumps(ret)

    def on_post(self, req, resp):
        req_text = json.loads(req.stream.read())
        if('image' in req_text):
            print("Received image of length: " + str(len(req_text['image'])))
            self.image = req_text['image']

api = falcon.API()
api.add_route('/api/image', ImageHandler())
print("Server online...")