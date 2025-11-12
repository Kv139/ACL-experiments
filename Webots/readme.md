This folder has some of the preliminary structure for implementing RL in webots with Scenic. In order to run this repository you will need to follow a few key steps:

1. Clone and build the ScenicGym branch of the Scenic repository: https://github.com/BerkeleyLearnVerify/Scenic/tree/Kai/ScenicGym-checkpoint (check the Scenic documentation for install directions)

2. Install the latest version of Webots: https://cyberbotics.com/#download

3. In order to run Webots with Scenic you will need to do two things. 
   1. Update your python path to include the path to include the controller\python library provided with Webots and/or update the "Python command" option in Webots preferences to point to your virtual enviroments python file.
   2. In your virtual enviroment define the variable $WEBOTS_HOME to point to your installation of Webots

4. Modify the controllers\scenic_supervisor to define your training loop etc. 

5. Modify WebotsFrankaSimulation inside "franka_simulator.py" to define simulation specific details such as:
   1. observation/actions space -- and how they map to the robot controls
   2. reward function
   3. Any other relevent simulation details

6. To run your program open Webots from inside your venv via the "Webots" command and open your corresponding world file (.wbt) within webots.

   
