# RoboClaw Control with Python

## Objective
  The RoboClaw will be used to operate the Yellow Jacket motors responsible for the movement of the arm. One controller will be used to control both motors and the controller will be connected via USB to the Jetson directly. Using the example code and links listed below, rcr-controls-arm should be able to communicate and begin setting up the movement controls for the motors.

## Example Code
- Look through roboclaw_python folder examples to see how to start communication and configure
  the contoller operation.

## Progress Update
- We are using BasicMicro Motion Studio (for 2x30A) to control the motors
- One of the motors have broken wires and were labeled defective
- `roboclaw_position.py` had incorrect encoder values when read from the defective motor

## Useful Links
- https://www.basicmicro.com/RoboClaw-2x30A-Motor-Controller_p_9.html
- https://resources.basicmicro.com/using-the-roboclaw-python-library/
- https://resources.basicmicro.com/using-encoders-with-the-python-library/
- https://www.basicmicro.com/downloads