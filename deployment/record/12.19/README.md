# Test time
nighttime
# Test environment
- indoor corridor
# Test result:
- create topomap in three times, with explore1 and explore2 using the same odometry but explore3 not. The part of topomap created by explore3 succesfully joins the existed topomap.
- navigate at destination 6 after explore1, destination 13 after explore2, destination 13 after explore3, all succeed.
# Conclusion
Now we can create large topomap in the way that we explore every single part of the world and join them together. Also, the simple navigation works well. 