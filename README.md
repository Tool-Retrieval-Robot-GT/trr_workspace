# Tool Retrieval Robot
The Tool Retrieval Robot aims to increase the effectiveness of receiving tools on the factory work floor. To prevent the excessive need to run around and grab tools to continue working, the TRR will do just that. By utilizing mapping via a user interface, a user can select the tools they want and where, and the TRR will grab it from a central storage location and bring it to that location. 

### Capabilities:
The Tool Retrieval Robot is programmed for these capabilities:<br />
1. Moves around with control over ROS2 Humble utilizing NAV2 or Telop Twist Keyboard by sending to an Arduino NANO<br />
2. Utilizes NAV2 to read a map and vaviage the map<br />
3. A full user interface that utilizies both the map and an active database to keep track of the tools that have been utilized <br />
4. Control a forklift with a ROS2 node<br />
5. Reads a QR code and adjusts the robot accordingly to pick up a bin with the QR code on it<br />


### Solutions:
There are two different versions of the Tool Retieval Robot:<br />
Main:<br />
One solution is highly inspired by the Preceptron Bot (<https://github.com/PedroS235/perceptron_bot>) which is the active main for this repo.<br />
Old Main:<br />
The second solution is inspired by the Articubot One (<https://github.com/joshnewans/articubot_one>) however it does differ from the project as most of the code is relatively original with some of the parameter files being copied.<br />


### Dependencies
- Ubuntu 22.04<br />
- ROS Humble<br />

Main:<br />
- PlatformIO (CLI)<br />

Old Main:<br />
- Arduino IDE<br />
