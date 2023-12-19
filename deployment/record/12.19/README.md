# Test time
daytime
# Test environment
- outdoor square
# Test result:
- Test 1: How to generate topomap nodes
  - time based
  - distance based
  - time and distance based
- Test 2: distance distribution before and after create topomap
- Test 3: explore on exsited topomap 
- Test 4: map pruning
  - path after pruning
    - [0, 1, 2, 48, 5, 6, 7, 22, 9, 23, 11, 12, 13, 28, 15, 16, 42, 17, 18, 19, 20, 24, 43, 44, 49, 22, 23, 11, 12, 50, 13, 46, 28, 15, 16, 42, 17, 18, 47, 24, 25, 7, 26, 23, 11, 27, 12, 28, 51, 52, 30, 31, 32, 33, 34, 35, 36, 38, 39, 40]
# Conclusion
- Test 1: time and distance based is the best among three
- Test 2: show the continuous of nodes choosen in topomap
- Test 3: the new explore succesfully join the origin topomap
- Test 4: the topomap is sparsed after pruning
# Achievment
Now wen can create a topomap making full use of visual information