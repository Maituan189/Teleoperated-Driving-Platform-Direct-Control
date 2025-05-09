# Importing flask module in the project is mandatory
# An object of Flask class is our WSGI application.
from flask import Flask, request, Response # type: ignore
import json
import cv2
import time
import logging

# Function to generate video frames from the camera
def generate_frames():
    camera = cv2.VideoCapture(0)
    while True:
        start_time = time.time()
        success, frame = camera.read()
        if not success:
            break
        else:
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            # Concatenate frame and yield for streaming
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n') 
            elapsed_time = time.time() - start_time
            logging.debug(f"Frame generation time: {elapsed_time} seconds")

# Flask constructor takes the name of 
# current module (__name__) as argument.
app = Flask(__name__)
data = {}

@app.route('/')
def hello_world():
    return 'Hello World'

@app.route('/test')
def test():
    # return Response (json.dumps(data),status = 200, mimetype='application/json')
    return Response('{"status":"ok"}', status=200, mimetype='application/json')

@app.route('/answer',methods=['POST'])
def answer():
    if request.form["type"] == "answer":
        data["answer"] = {"id" : request.form['id'], "type" : request.form['type'], "sdp" : request.form['sdp']}
        return Response(status=200)
    else:
        return Response(status=400) 

@app.route('/offer', methods=['POST'])
def offer():
    if request.form["type"] == "offer":
       data["offer"] = {"id" : request.form['id'], "type" : request.form['type'], "sdp" : request.form['sdp']}
       return Response(status=200)
    else:
       return Response(status=400)

@app.route('/get_answer', methods=['GET'])
def get_answer():
    if "answer" in data:
        j = json.dumps(data["answer"])
        del data["answer"]
        return Response(j, status=200, mimetype='application/json')
    else:
        return Response(status=503) 

@app.route('/get_offer', methods=['GET'])
def get_offer():
    if "offer" in data:
        j = json.dumps(data["offer"])
        del data["offer"]
        return Response(j, status=200, mimetype='application/json')
    else:
        return Response(status=503) 
    
@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# main  function
if __name__ == '__main__':
    # run() method of Flask class runs the application 
    # on the local development server.
    app.run(host='0.0.0.0', port = 8080)