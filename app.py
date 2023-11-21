from flask import Flask, request, render_template, jsonify
from flask_cors import CORS
import threading
import time
import rclpy
from geometry_msgs.msg import Twist

app = Flask(__name__)
CORS(app)

data = None
data_lock = threading.Lock()

def print_data_thread():
    global data
    node = rclpy.create_node('accelerometer_publisher')
    publisher = node.create_publisher(Twist, 'accelerometer_data', 10)
    msg = Twist()

    while True:
        with data_lock:
            if data is not None:
               
                #print(data)
                msg.linear.x = data['gamma']
                # msg.linear.y = data['y']
                msg.angular.z = data['alpha']
                publisher.publish(msg)
                data = None
                time.sleep(0.01)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/store_accelerometer_data', methods=['POST'])
def store_accelerometer_data():
    global data
    new_data = request.get_json()
    with data_lock:
        data = new_data
    return jsonify({'message': 'Data received successfully'})

@app.route('/store_gyroscope_data', methods=['POST'])
def store_gyroscope_data():
    global data
    new_data = request.get_json()
    with data_lock:
        data = new_data
    return jsonify({'message': 'Gyroscope data received successfully'})


if __name__ == '__main__':
    # Start the printing thread
    printing_thread = threading.Thread(target=print_data_thread)
    printing_thread.start()

    # Initialize the ROS2 node
    rclpy.init()

    try:
        # Run the Flask app
        app.run(ssl_context=('server.crt', 'server.key'), host='0.0.0.0', port=8032)
    finally:
        # Clean up ROS2 resources when the program exits
        rclpy.shutdown()
