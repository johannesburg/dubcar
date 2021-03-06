from flask import Flask, render_template
from flask_socketio import SocketIO, send, emit
import eventlet
from Queue import Queue
import numpy as np

eventlet.monkey_patch()

# Shared/Global data
POLL_TIME = 5 # Seconds
shared_countries = [
  {"name": "Sweden"}, {"name": "China"}, {"name": "Peru"}, {"name": "Czech Republic"},
  {"name": "Bolivqia"}, {"name": "Latvia"}, {"name": "Samoa"}, {"name": "Armenia"},
  {"name": "Greenland"}, {"name": "Cuba"}, {"name": "Western Sahara"}, {"name": "Ethiopia"},
  {"name": "Malaysia"}, {"name": "Argentina"}, {"name": "Uganda"}, {"name": "Chile"},
  {"name": "Aruba"}, {"name": "Japan"}, {"name": "Trinidad and Tobago"}, {"name": "Italy"},
  {"name": "Cambodia"}, {"name": "Iceland"}, {"name": "Dominican Republic"}, {"name": "Turkey"},
  {"name": "Spain"}, {"name": "Poland"}, {"name": "Haiti"}
]
start = 0
finish = 10

app = Flask(__name__, static_url_path='/static')
app.config['SECRET_KEY'] = 'fuckhaters'
socketio = SocketIO(app, logger=True, engineio_logger=True)

def ack():
    print 'message received callback'

def wrap(angle):
    return angle 


def rotate_and_emit():
   
    roll = pitch = yaw = 0
    roll_v = 5
    pitch_v = 0.1
    yaw_v = 0
    dt = 0.01
    step = 0
    while True:
        roll = wrap(roll_v * step * dt)
        pitch = wrap(pitch_v * step * dt)
        yaw = wrap(yaw_v * step * dt)
        step = step + 20
        socketio.emit('imu', (roll, pitch, yaw))
        eventlet.sleep(dt)

@socketio.on('message')
def handle_message(message):
    print('received message: ' + message)

@socketio.on('test')
def handle_message(message):
    print('received test message: ' + message)
    #send(message)

@app.route('/')
def index():
    return render_template('index.html')


@app.route('/hello')
def hello():
    return render_template('hello.html')


if __name__ == '__main__':
   # emit_thread = threading.Timer(POLL_TIME, rotate_and_emit, ())    
   # emit_thread.daemon = True
   # emit_thread.start()
   # 
    print 'starting server'
    eventlet.spawn(rotate_and_emit)
    socketio.run(app, port=8080, debug=True)
