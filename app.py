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

SCALE_X = 0.009
SCALE_Z = 0.006
DEADZONE_WIDTH_Z = 4
DEADZONE_WIDTH_X = 2

def print_data_thread():
    global data
    node = rclpy.create_node('accelerometer_publisher')
    publisher = node.create_publisher(Twist, '/twist_topic', 10)
    msg = Twist()

    while True:
        with data_lock:
            if data is not None:

                alpha = data['alpha']
                scaled_alpha = 0.0
                if abs(alpha) <= DEADZONE_WIDTH_Z:
                    scaled_alpha = 0.0
                else:
                    alpha = (alpha + 180) % 360 - 180
                    scaled_alpha = (abs(alpha) - DEADZONE_WIDTH_Z) * SCALE_Z
                    scaled_alpha = scaled_alpha if alpha > 0 else -scaled_alpha
                msg.angular.z = scaled_alpha

                #gamma = data['gamma'] 
                gamma = -data['beta'] 
                scaled_gamma = 0.0
                if abs(gamma) <= DEADZONE_WIDTH_X:
                    scaled_gamma = 0.0
                else:
                    gamma = gamma-DEADZONE_WIDTH_X*SCALE_X 
                    scaled_gamma = (abs(gamma) - DEADZONE_WIDTH_X) * SCALE_X
                    scaled_gamma  = scaled_gamma if gamma > 0 else -scaled_gamma
                msg.linear.x = scaled_gamma

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


@app.route('/recenter_gyro', methods=['POST'])
def recenter_gyro():
    global data

    with data_lock:
        if data is not None:
            current_alpha = data['alpha']
            data['alpha'] = (data['alpha'] - current_alpha) % 360  # Apply offset based on the current alpha
            if data['alpha'] >= 180:
                data['alpha'] -= 360  # Convert to range [-180, 180)

    return jsonify({'message': 'Gyro recentered successfully'})





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
