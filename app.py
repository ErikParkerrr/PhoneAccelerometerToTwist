from flask import Flask, request, render_template
from flask_cors import CORS




app = Flask(__name__)
CORS(app)


@app.route('/')
def index():
    return render_template('index.html')

@app.route('/imu', methods=['GET', 'POST'])
def imu_data():
    if request.method == 'POST':
        # Assuming the IMU data is sent as JSON in the request body
        imu_data = request.json

        # Process the IMU data as needed
        # For example, print the received data
        print("Received IMU data:", imu_data)

        # You can perform further processing or store the data in a database

        # Send a response back to the iPhone (optional)
        return {'message': 'IMU data received successfully'}, 200

if __name__ == '__main__':
    # Run the Flask app on a specific port
    app.run(host='0.0.0.0', port=8032)
