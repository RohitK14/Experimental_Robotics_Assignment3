# Experimental_Robotics_Assignment3

February 25, 2020

## <a name="SS-Index"></a>Index


* [Introduction](#S-Introduction)
* [Software Architecture](#S-Sofar)
* [Packages and Files List](#S-PFL)
* [Installation and Running Procedure](#S-IRP)
* [Working Hypothesis](#S-WH)
* [System’s Features](#S-SF)
* [System’s Limitations and Possible Technical Improvements](#S-SL)
* [Authors and Contacts](#S-AC)


# <a name="S-Introduction"></a>Introduction
The aim of this assignment is to implement four states of the robot which are Normal, Play, Sleep and Find. We use smach_viewer in ROS to implement these behaviors and visualize the states. The world environment is provided with six rooms. Each room has a different colored ball.The robot roams in the environment and follows the colored balls to note positions of the respective rooms. When the human commands to play, the robot decides to reach that location as it is already explored. Next, if the exploration of the room is not done, the robot explore the environment through Find behavior. To implement, we use to sensor, which are hokuyo laser scanner and camera sensor for navigating in the environment.

# <a name="S-Sofar"></a>Software Architecture
## <a name="SA-SMD"></a>The State Machine Diagram
For implementing the concept, we can see the finite state machine diagram in the image below. Here, there are three states of the robot. The transitions are mentioned in the image. As, the find algorithm could not be implemented successfully, it is not shown in the image.

![Assignment1_ExperimentalRobotics](images/state_machine.png)

* Normal - This state has two possible transitions "go_sleep" and "start_play"
* Play - This state has one outcome where it goes back to the normal state with transition "go_to_normal" and the other state Find when we don't have the location of the room
* Sleep - This state also has one tansition where it goes to normal state with "wake_up"
* Find - This state enables the robot to explore the environment to find the ball and return to play behavior when done.
## <a name="SA-CD"></a>Component Diagram
The following figure illustrates how the idea was supposed to be implemented. There are theree ROS nodes. The node *human_command* publishes the string rostopic */command* which is subscribes by the *state_behavior*. Similarly, the node "robot_following" subscribes to camera sensor images and searches for the balls. Once, the ball is detected it goes to the ball and records the location. Lastly, the node "state_behavior" should apply the four finite states using the above nodes and their information.

### <a name="SA-MSG"></a>The Messages 
This package has some messages which are described in the following.
* geometry_messages/Point - It is used for the x,y coordinate of the robot.
* std_msgs/Bool - To check if the robot has reached the position or not.
* std_msgs/String - To receive the command from person to play with the robot.
* sensor_msgs CompressedImage: To get and show the image using opencv.
* geometry_msgs Twist: To publish robot velocities to reach the colored balls.
* MoveBaseGoal(): The actionlib action goal to move the robot towards a randomly decided location.
* nav_msgs Odom: It is required for odometry data and taking the position of the robot while going near the ball.

### <a name="MSG-P"></a>The Parameters
The following parameters are launched from the launch file. These can be altered using the launch file in robot_motion/launch/behaviors.launch thus allowing flexibility.
* home_x and home_y: These define the SLEEP coordinates of the behavior. The robot goes to this position when it is tired.
* tireness_level: This defines the threshold of the robot when it is under any state. 


# <a name="S-PFL"></a>Packages and Files List

There are three packages in python, which are *human_command.py*, *robot_following.py* and *state_behavior.py* under exp_assignment3-main/scripts folder.

The doc folder contains the sphinx file as well as the index.html file which can be used to see the documentation in the browser.

# <a name="S-IRP"></a>Installation and Running Procedure
Firstly, the repository should be cloned into the ROS workspace with the command

    git clone https://github.com/RohitK14/Experimental_Robotics_Assignment3.git
    
As the state machine is implemented using smach_viewer, we need to install it, if it is not available.

    sudo apt-get install ros-<melodic>-smach-viewer

where <meldoc> is the ROS distribution in the system. 

Now, we need to build the workspace. It can be done using catkin build command. In the workspace, use catkin build

    catkin build

The python scripts are not executable. Hence, go to exp_assignment3-main/scripts folder and run the following command to make it executatble,

    chmod +x state_behavior.py
    chmod +x human_command.py
    chmod +x robot_following.py

To run the application, we can run it using the single launch file,

    roslaunch exp_assignment3 main.launch

In order to see the documentation, there is an html file in the doc folder. To see the documentation we run,

    firefox _builds/html/index.html 

# <a name="S-WH"></a>Working Hypothesis and Environment
The hypothesis that is considered here is that the robot starts in the NORMAL state. When the robot is randomly moving in the 2D environment, it gets tired. We have a parameter that decided the tireness level. If the level is 5, the robot reaches 5 positions and goes to SLEEP state. The sleep state is considered to be at origin which can be changed using the parameters. The robot sleeps for 10 seconds as per our hypothesis.
The robot goes between these two states unless there the user publish "play" command at random. Whenever, we receive play command, the robot finishes it's last motion and goes to the person. The person position is fixed. It is confirmed that the robot reahces the person and waits for the gesture coordinates. The person gives the command GoTo a particular room to play with the robot. The coordinates of the room are either available through exploration in Normal behavior. If not, the robot switches to Find state and search the given world using expore-light package in the repository. After that, the robot returns to the human. This is repeated until a random number of times. The robot is not expected to get tired in between and go to sleep position. The only possible state for the transition is normal state before going to sleep state. In all the autonomous navigation gslam mapping is used and it avoids the obstacles perfectly.

# <a name="S-SF"></a>System’s features
* All states and information available on the terminal where we launch the file.
* All files launched through a single launch file.
* The publisher command is decided randomly after certain time.
* smach_viewer for visualising the states.
* Play command can be send any time between the states.
* The priority was to detect the ball first in any state.


# <a name="S-SF"></a>System’s limitations and Possible Technical Improvements
* The robot senses the ball, but fails to go towards in some cases.
* Find state was failed to be implemented.
* There is a possibilty that the robot can go to sleep while playing when it is tired or in the find state.
* When combined with explore-lite package, the robots goes towards the wall and becomes out of control sometimes.
* No perfect coordination between states.

Overall, the idea is clear to implement but was not able to implement in a perfect manner.

# <a name="S-AC"></a>Authors and Contacts
This project was relized by Rohit Kumar

University email s5089482@studenti.unige.it
gmail: rohitkb114@gmail.com
