# Test time
daytime
# Test environment
- indoor hall
- outdoor square
# Navigate strategy: 
navigate
# Test result:  
- indoor2_test
- outdoor2_test

Record an ideal trajectory and create topomap, then navigate to the published goal iamge sequences from the topomap, record the navigate trajectory.
# Conclusion
The success rate is low. If the images of nearby topommap nodes are not countinuous visually, then the risk rises.
# Problem
The zero points of each trajectory are not strictly overlapped. The robot did not stop when reached goal as expected due to code bugs.