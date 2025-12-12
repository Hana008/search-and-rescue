# search-and-rescue

Search and rescue robot: the robot’s job is to explore a known map in an efficient manner returning the victim’s positions to human responders, while avoiding unknown obstacles. This idea is interesting and useful because, in wake of disasters, known and mapped areas often become dangerous for humans and new obstacles appear, e.g. rubble. The robot used is a Pioneer 3-DX, equipped with a camera. There are four independent main modules with one group member assigned to each. The four modules are as follows: path planning, obstacle avoidance, victim detection, and localisation. 

## The packages we used are: 
- pioneer 3dx robot
- Enum
- Math
- Cv2
- Numpy
- Heapq
- OpenCV


## Components implemented by the team

### 1. python_main.py (Main Script)
- Integrates all modules (path planning, victim detection, localisation)
- Coordinates the robot workflow and handles control flow
- Calls each module in the correct sequence and manages interactions

### 2. path_planning.py
- Implemented by Patrick
- Custom logic for robot navigation based on the project proposal
- Includes decision-making for movement, route selection, and obstacle handling

### 3. victim_detection.py
- Implemented by Hana
- Initiates camera feed and performs area scanning
- Detects victim markers
- Retrieves localisation/orientation

### 4. localisation.py
- Implemented by Patrick
- Handles determination of robot position and orientation in the environment.

