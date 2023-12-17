# Test time
12.02 Daytime
# Test environment
- indoor corridor 
- outdoor road
# Navigate strategy: 
track (the GNM demo)
# Test result:  
- indoor_test1 failed, crashing obstacle by the roadside
- indoor_test2 successed, after manual adjustment
- outdoor_test1 successed, without manual adjustment
- outdoor_test2 interrupted by passerby, thinking having reached the goal
# Conclusion
The GNM model is quite capable to navigate on the given waypoints of a trajectory, spending about 3min to arrive at a 30m goal.
# Achievement
We have succesfully deployed the GNM model to our robot, and proved that the model actually works as the researcher claimed.
# Problem
The initial config of the robot is not suitable for out classbot, so we need change the robot config to adapt so that we can get an expected performance. We also need to look carefully through the given structure to make our own progress.