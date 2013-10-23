Skeleton-based Gesture Recognition
==============================

Using skeleton joint positions provided by openni2_user_selection.


####Changelog:
23.10.2013:
- offset values published, needs confirmation for proper coordinate transformation


22.10.2013:
- gesture area is defined (square perimeter with shoulder joint in the middle)


17.10.2013:
- first commit
- starting pose is implemented (hover hand in front of shoulder for a certain time)


####To Do:
23.10.2013:
- when the user exit/lost, offset should be set back to 0
- offset should be limited to FoV


22.10.2013:
- publish relative hand displacement values for robot input, make sure the sign are correct


17.10.2013:
- define area to do gesture
