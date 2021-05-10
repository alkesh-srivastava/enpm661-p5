# ENPM661
## Project 5 - Implementation of A Distributed & Optimal Path Planning Approach for Multiple Mobile Robots'', Guo et al. 2002
#### **Contributors :**
**Alkesh Kumar Srivastava**&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;**Hari Krishna Prannoy Namala** <br />
UID -117451788&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;UID -117507409 <br />
_alkesh@umd.edu_&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;_pnamala@umd.edu_ <br/>
This project developed a multi-robot system in a partially observable environment that implements online planning in a distributed yet optimal manner. This project will be using the D-star algorithm which accounts for the dynamic nature of a multi
robot system and produces optimal solutions even if robots are on a collision course. By partially observable environment, we do not refer to an environment whose obstacles and obstructions are known to the robot rather we refer to an environment whose such parameters are known to the knowledge base of the robot but the path plan produced earlier do not take into account the location and motion of the other robots in the environment.


We will be using the following maze as our main playground for the robot the multirobot system. This map has been taken from Project 3 Phase 3 and is an appropriate map to test the feasibility of the implementation done in the chosen research paper.
![Input](https://github.com/alkesh-umd/enpm661-p3-phase3/blob/main/images/image6.png)
### Objective
The goal of the project is to simulate results from the paper mentioned in the title of this Readme file. This project will fall under the **OPTION 1** in the proposed options to choose for this project.
1. The end result of the project produced a simulation where a system of robots are assigned to reach their respective goal positions, from a user defined start position
on the above pre-defined map.
2. The simulation video can be accessed from <a>here.</a>

In the root folder you will notice that there are multiple files and directories. To run this file you will have to run `main.py ` file.
You will be required to enter the starting ang goal position of each of the robot. Please ensure that you properly enter the start and goal position to run the file correctly. 
Do not worry about entering co-ordinates that will coincide with obstacle, the program is smart enough to detect obstacle before you run the file.
![Input](https://github.com/alkesh-umd/enpm661-p3-phase3/blob/main/images/image3.png)

As soon as you hit enter, the python environment simulation will begin as shown in the figure : 
![Envrionment](https://github.com/alkesh-umd/enpm661-p3-phase3/blob/main/images/image4.png)

You will notice that the program is ending before the robots reach their goal position. Actually, that is the objective of this project. Take the following co-ordinates as an example :
1. `Robot 1 - (0,0) to (70,70)`
2. `Robot 2 - (25,0) to (25,70)`

With these input, you will notice that as soon as the robots are about to collide the python environment will stop simulating. It will generate a co-ordination graph that might look something like this:

As soon as you will close graph the program ends. Although it might not seem like a successful completeion, but on checking the terminal you will see that its filled with co-ordinates that have been modified as per the velocity profile generated using the co-ordination graph that you just saw. That's the whole point of this project. 
![Output](https://github.com/alkesh-umd/enpm661-p3-phase3/blob/main/images/image5.png)
This was the objective of the project. We successfully implemented collision detection and obtained modified pathway that can be used to simulate in Robot Operating System and justify the result.

![Output](https://github.com/alkesh-umd/enpm661-p3-phase3/blob/main/images/image5.png)


This is a beautiful project that entails how dynamic path detection can be coupled with replanning using coordination graph of coordination space and avoid collision in a humane way.
You can find the report for this project <a>here</a>. The report includes suggested extension for the study and a small part of the implementation of the suggested methodology. 


In case you encounter any difficulty please feel free to contact the creators - <br/>
***Alkesh K Srivastava*** - `{alkesh@umd.edu}`, University of Maryland, College Park <br/>
***Hari Krishna Prannoy Namala*** - `{pnamala@umd.edu}`, University of Maryland, College Park <br/>
