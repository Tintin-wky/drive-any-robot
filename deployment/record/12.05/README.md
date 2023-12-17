# Test time
12.05 Nighttime
# Test environment
- indoor hall 
# Navigate strategy: 
manipulated circle
# Test result:  
- indoor2_test1
- indoor2_test2

To find out how the robot recognized the distance between current position and goal position, two tests given different goal image sampled from the circle run, in test1 it recognized the goal twice but in test2 it only recognized once.
# Conclusion
It is not certain that the robot can recognized similar images as close points in the real world, due to the fact that we have not tune the model by using our local dataset.
# Problem
We still don't know the error distribution of estimate distance,which may cause big problems in the futher work. To improve its performance, we should make datasets and tune the model sometime in the future.