# Description

This project hosts a webserver using Flask. Navigating to the webserver with a phone prompts the user with a few buttons. After providing permission to the website to grab IMU data from the user's phone, the python script takes that data and publishes it on a ROS2 Twist topic to control a robot via tilting the user's phone.

# Key features

There are variables in the Python script to change the scaling and dead zone. Change these to meet the needs of your robot

    SCALE_X = 0.009
    SCALE_Z = 0.004
    DEADZONE_WIDTH_Z = 2
    DEADZONE_WIDTH_X = 2

In the index.html file, there is a variable to change the number of requests that are made to the client to get the gyroscope data. Decreasing this number too much causes a buffer overflow creating huge input lag. Increasing this number too much creates too much time between requests so the robot may feel unresponsive. Tune as needed.

    const delayBetweenRequests = 190; // this value will need to be tuned to get everything working right

In the Python file, changing the topic and node names is easy.

    twist_topic_name = '/twist_topic'
    node_name = 'phone_gyro_node'

The E-STOP button on the webpage will send all zeros in the Twist message. In the event of an issue, press this button. Swipe back to the main page to regain control. *THIS DOES NOT REPLACE A HARDWARE E-STOP. PLEASE USE A HARDWARE E-STOP IF YOUR ROBOT HAS THE ABILITY TO CAUSE DAMAGE OR INJURY.*

# Notes

* This has only been tested with Apple iPhones. Android devices are untested. support may be added for them in the future.

* On iPhone, the Google web browser app seems to handle a higher rate of data requests than Safari can. YMMV based on your browser app, device, and connection stability.

* This repo has a dummy HTTPS certificate included with it. This is for testing purposes only. HTTPS is required to get a phone to share IMU data so some certificate is needed. If using a dummy certificate, you will need to tell your browser to continue to the website since it won't be able to confirm the authenticity of the certificate. 

* You must navigate to the webserver using HTTPS in the address bar or it may not work. Your browser may not add HTTPS by default.

# Installation and Usage

This program has only been tested to work with ROS2 Humble Desktop. I'm rather sure it would work with most distributions of ROS2, but please test. Please install ROS2 Humble before continuing to the rest of the instructions.

This software makes use of the flask and flask_cors packages

    pip install flask
    pip install flask_cors

Configure all your options highlighted in the *Key Features* section above. Then run the app.py file with Python.

    python3 app.py


