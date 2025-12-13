# obd_infotainment_system
IoT project - we are creating an infotainment system for my old car (a volvo v70 2002) using the data gathered from the obd and an imu - and then display it on a raspberry pi.

## Initial Project Description

We would like to create a car dashboard to display fun and useful statistics to the driver. For this project, we plan on implementing a network of three Raspberry Pi devices connected to the following peripherals:

- A touchscreen display 
- A grove GPS module
- A grove IMU (gyroscope, accelerometer and electronic compass)
- A USB OBD-II reader to collect diagnostics from the car

We plan on using MQTT to send sensor data to the main Raspberry Pi, where it will be logged in a database and displayed on the screen. The network will be hosted via the driver’s mobile hotspot.

 

We are using the following hardware components for this project:

- 2 Raspberry Pi 3 Model B+: https://www.seeedstudio.com/Raspberry-Pi-3-Model-B-p-2756.html 

- 1 additional Raspberry Pi with 8GB of RAM: https://www.seeedstudio.com/Raspberry-Pi-5-8GB-p-5810.html

- 1 Raspberry Pi Touch Display https://www.seeedstudio.com/Raspberry-Pi-Touch-Display-2-p-6255.html

1 Grove GPS module https://www.seeedstudio.com/SeeedGrove-GPS-Air530-p-4584.html

1 Grove IMU https://www.seeedstudio.com/Grove-IMU-9DOF-ICM20600-AK09918.html

 

The OBD-II reader will be ordered separately by us.

We would appreciate it if you could review our plan and let us know if the proposed hardware and approach meet your expectations. If there’s anything you’d like us to adjust, add, or clarify, we’d be happy to do so. Please let us know if you have any feedback or follow-up questions.

 

Kind regards