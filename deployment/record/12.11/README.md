# Test time
12.11
# Test environment
- indoor hall
- outdoor square
# Navigate strategy: 
manipulated circle
# Test result:  
- indoor2_test nighttime
- outdoor2_test daytime

To simulate the exploration of the robot, we use rosbags to make topomaps, the robot add nodes to the map every few seconds, also check if it is close to a ever arrived position while moving, nearest node in the map is shown at top left corner and last node in the map is shown at bottom left corner, at the end of the journey, a visualized map is recored and shown visualized.
# Achievement
We now can create topomap while driving, and visualize it.
# Problem
It is a huge cost to check all the nodes using the model reasoning, which has shown big time cost differenct between CPUs and GPUs. It is a strategy like ViNG, searching all the nodes in the map. We consider use extra information such as GPS to cut down the afford.