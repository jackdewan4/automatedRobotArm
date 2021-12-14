# automatedRobotArm
In this project I developed an application for the robot arm to complete a task of picking up an object and placing it 90 degrees to the right of itself and stacking the rest of the objects on top of the first placed. The system utilizes a combination of one infrared sensor and an ultrasonic sensor to find the distance of the object to pick up and confirm that the object is within the range of the arm. The program takes 10 distance measurements from both sensors, averages them and takes an average of the two averages from both types of sensors. Another ultrasonic sensor which is located on the left of the robot arm’s base is used to check that there are no persons who could be in the range of danger of the robot arm and will stop the process if there is a person within range. An on/off switch is also implemented on the back of the base to allow for disabling the robot without the need to cut power to the system. The finalProject.ino file (Arduino file) is the application source code, while the other files are libraries used for the robot arm system. A CAD model of the robot arm used in this application can be seen in the automatedRobotArmCADdesign.pdf file. 