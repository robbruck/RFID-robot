# RFID-robot
self-balancing, two-wheeled robot that searches, picks up and transports RFID tags to specific landing zones

this is a project to learn about and apply various technologies:
- An Arduino to stabilize and operate a 2-wheeled, self-balancing robot
- A Raspberry Pi 4B as master controller, using a camera, an RFID-reader, a servo to control a mechanical lever, an electro-magnet and a network
- An EC2 instance on AWS to which logfiles and images are uploaded

high-resolution videos are here:
- the initial video: https://youtu.be/hJhErZ3bX6Q  
- plus an update with 2 tags and obstacles: https://youtu.be/Sfq0CtV01Rc  
- and something unexpected towards the end (starting at 1:05): https://youtu.be/7XMK1tC53_U

---------------------------
**Technology overview:**

**Arduino**
- Standard Arduino IDE, C++ based
- MPU6050 3-axis gyro & accelerometers
- PWM-control of motors for wheels, angle measurement with 2 segment phase detectors
- Kalman-Filter to estimate the deviation angle from upright position
- balance control based on 5 inputs (angle, angular speed, position, speed, integrated position deviation)
- robot positioning is tricky due to soft wheel rubber and uneven floor surface
- serial interface to communicate to Raspberry via USB
- custom-made interface between Arduino & Raspberry
- multiple interrupt levels
- Bluetooth serial interface to local PC (not used anymore)

**Raspberry Pi 4B**
- Debian Linux 
- command/control of Arduino via serial interface
- read/write to RFID devices
- servo to handle lever, electromagnet on/off
- wide angle camera readout, compensation of lens distortion, transforming pixel coordinates to floor coordinates
- Socket connection to AWS, used for logging of traces and storing of taken images
- background thread to add logging (esp. of power status) and ensure timeout of the electromagnet
- Python 3.7.3 using various libraries (OpenCV, picamera, socket, numpy, threading, serial, ...)

**local PC**
- PyCharm IDE
- Jupyter Notebook for data analysis, symbolic calculations (e.g. equations of motion)

**AWS**
- t2-micro EC2 instance
- Amazon Linux
- Python 3.7.9 to receive data from robot and store log information or images
