## Project 6: Robot Intention Communication
Date: 29.04.2019

Priteshkumar Gohil:

   * Last week, I was working on the message framework and updated the same in github issue.
   * Following week, my plan is to work on parsing message received on serial port and execute a command based on the request.
   * No this week I don't see anything that can prevent mer to achieve my sprint goal.

Eduardo Cervantes:

   * Last week I was learning the basic commands used by Adafruit CircuitPlayground to interact with the internal components (NeoLeds,  Buzer, Buttons, Accelerometer, etc.). Additionally, I developed an initial sketch of basic communication between our board and Python.
   
   * Going from simple parsing to implement a more robust method into our board
    
   * Until now, I haven't detect any possible problem.


Date: 06.05.2019

Priteshkumar Gohil:

   * Added a code to decode the LED number from message format and blink respective LED.
   * Added a code to decode the LED blinking pattern from message format and blink LED according to action (pattern code) performed by the robot.
   * Refactoring the code to follow certain naming convention.

Eduardo Cervantes:

   * The communication with the Circuit Playground pid=219A is now made automatically. Additionaly the color now comes from the message sent by our interface
   * Start to implement the different methods supported by Ropod. 
   * Depending on the amount of boards to be implemented on the robopod the automatic connection shall follow a different method. Which represent more time.



Date: 13.05.2019

Priteshkumar Gohil and Eduardo Cervantes:

* Last week we did pair programming in lab class and finished our weekly task.
* The weekly task was to refactor the code.
* Created a package of playground interface python file so that we can import the package while working with ROS.
* Created a simple test case with ROS package. For example, create a publisher which publish our message frame. Creat subscriber, which receives the message published by publisher, then import playground interface python package and send message framework to the arduino.


Date: 20.05.2019

Priteshkumar Gohil and Eduardo Cervantes:

* Last week we did integration test with the ROPOD and our demo code was successfully working.
* We also started looking into ROPOD maneuver navigation and planner code to get access of the publishing messages. 


Date: 10.06.2019

Priteshkumar Gohil
* In last two week, I have refactored the cmd_vel code and placed it in proper function in python class.
* Then as per requirement, I have written the code that can check the robot motion from cmd_vel msg and generate message frame dynamically instead of fixed pattern.
* Together with eduardo, we tested new dynamic logic of intention communication with ROPOD platform.
* We found some bugs and fixed it during the demonstration in lab.

Next task: 
* Add intention when the ROBOT is rotating.
* Add the sound notification during docking and undocking state.
