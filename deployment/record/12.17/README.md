# Test time
12.17 daytime
# Test environment
- outdoor square
# Test result:
Use outdoor2.bag, generate topomaps in two ways, the odom is not accurate due to slipping of the tires.
- test 1: generate the whole topomap in one time 
  - rosbag time :0-164s
  - path:
    - 1-1:[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 1, 2, 3, 11, 12, 5, 6, 13, 7, 14, 9, 15, 10, 1, 16, 17, 11, 5, 18, 6, 19, 20, 21, 22, 23, 24]
    - 1-2:[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 1, 2, 11, 12, 13, 5, 6, 14, 7, 15, 8, 15, 9, 16, 10, 1, 17, 18, 4, 12, 5, 19, 6, 20, 21, 22, 23, 24, 25]
    - 1-3:[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 2, 12, 13, 14, 5, 6, 15, 7, 16, 8, 16, 9, 17, 10, 1, 18, 19, 4, 13, 5, 14, 6, 7, 20, 21, 22, 23, 24, 25]
    - 1-4:[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 2, 12, 13, 14, 5, 6, 15, 7, 16, 9, 17, 10, 1, 18, 19, 4, 13, 5, 14, 6, 7, 20, 21, 22, 23, 24, 25]
- test 2: generate the topomap in two times
  - rosbag time : 0-60s 60s-164s
  - path:
    - 2-1&2-2:[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 2] [12, 3, 4, 13, 5, 6, 14, 7, 15, 8, 15, 9, 16, 10, 1, 17, 18, 5, 19, 6, 7, 20, 21, 22, 23, 24, 25]
    - 3-1&3-2:[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 2] [11, 12, 4, 13, 5, 6, 14, 7, 15, 8, 15, 9, 16, 10, 17, 18, 12, 5, 13, 6, 19, 20, 21, 22, 23, 24]
    - 4-1&4-2:[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 2] [12, 3, 4, 13, 5, 6, 14, 7, 15, 8, 15, 9, 16, 10, 1, 17, 18, 4, 5, 19, 6, 20, 21, 22, 23, 24, 25]
# Conclusion
The topomap is generated successfully and steadily.
# Achievement
We can use existed topomap in the same odmometry.
# Problem
How about using different odometry?