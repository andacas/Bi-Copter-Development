# Stage 1: Preliminary Research and Conceptualization
Work to do:
- [x] Define Objectives
- [x] Literature Review
- [x] Conceptual Design

## Define Objectives

 1. **Payload of 3-4kg**
 this means that the drone  design should weight around 3kg, this is because this project is one operation mode of a mobile robot.
 
 2. **Teleoperate with ROS2**
 This project will handle the comunications using ros, for the sensors and also for the fly controller.
 
 3. **Been able to stay on a single position**
 Use sensors to estimate the position control, investigate what is the most reliable way.
 
 4. **Trayectory tracking**
 The idea is to be able to create waypoints for the robot to follow.

## Literature Review
### Papers
1. **Attitude Control and Low Cost Design of UAV Bicopter**
https://doi.org/10.48550/arXiv.2309.08209

2. **Design and Development of a Flying Humanoid Robot Platform with Bi-copter Flight Unit**
https://doi.org/10.1109/HUMANOIDS47582.2021.9555801

3. **Design of a Robotic Bicopter**
https://doi.org/10.1109/ICCMA46720.2019.8988694

4. **Modeling and Attitude Control of Bi-copter**
DOI:10.1109/AUS.2016.7748042

### Some interesting videos
Modelization and control of a quadrotor:   https://www.youtube.com/watch?v=MlnagaOBSiY
TYI 5010 340 KV Brushless Motor Thrust Test : https://www.youtube.com/watch?v=8cfKnhwWTH8


1 | How to simulate a drone motor mathematically : https://www.youtube.com/watch?v=XjhY2tyhYZ8

2 | How to simulate drone dynamics mathematically : https://www.youtube.com/watch?v=edBmZOR61RM


built videos:
https://www.youtube.com/watch?v=nDwtfH9CiLk
https://www.youtube.com/watch?v=Ot3Em2lI9y8

## Conceptual Design
conceptual design for a bi-copter with 2 BLDC (Brushless DC) motors for the propellers and 2 servo motors for tilt control of the motors:

### Frame Design:

The frame will consist of a lightweight and durable material such as carbon fiber or aluminum.
It will have a symmetrical X-shaped configuration to accommodate the two BLDC motors and servo motors.
The frame will be designed to provide stability and structural integrity while minimizing weight.
### Propulsion System:

Two BLDC motors will be mounted on opposite ends of the frame.
Each motor will be connected to a propeller to generate thrust for lift.
The propellers will be selected based on their size, pitch, and efficiency to provide optimal performance.
### Tilt Mechanism:

Two servo motors will be installed on the frame, one for each pair of BLDC motors.
The servo motors will be responsible for tilting the BLDC motors and propellers to control the direction of thrust.
The tilt angle of the motors will be adjustable to facilitate maneuverability and stability during flight.
### Flight Control Surfaces:

In addition to the propulsion system, the bi-copter may incorporate flight control surfaces such as ailerons or elevons to provide additional control authority.
These surfaces will be integrated into the frame design and controlled electronically to assist in stabilization and maneuvering.
### Payload Integration:

The frame will have provisions for mounting payloads such as cameras, sensors, or other equipment.
Payload mounting locations will be strategically chosen to maintain the center of gravity and balance of the bi-copter.
### Power and Control Electronics:

The bi-copter will be equipped with an electronic speed controller (ESC) for each BLDC motor to regulate speed and direction.
A flight controller unit will be responsible for processing sensor data, stabilizing the aircraft, and implementing control algorithms.
Power distribution and wiring will be organized to ensure reliable operation and minimize interference.
### Safety Features:

The design will include safety features such as fail-safe mechanisms, redundant systems, and emergency landing procedures to mitigate potential risks during operation.
This conceptual design outlines the basic components and features of a bi-copter with BLDC motors for propulsion and servo motors for tilt control. Further refinement and detailed engineering will be necessary to translate this concept into a functional and reliable aircraft.

