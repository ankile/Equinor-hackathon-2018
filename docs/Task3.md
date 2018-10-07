# Task 3: Object Localization

Task 3 consists of combining task 1 and task 2 to create a system capable of navigating a space, locate, read and report the numbers found.  

In this task, once the drone reaches its goal, it must send in the numbers it recognizes at the goal. Once these have been sent in, a new goal will be assigned and the drone must once again navigate to the goal and send new numbers.

## Navigation node

This task will require an exstension to the navigation node described in task 1. Since teams will be scored on all 3 tasks a solution should be made such that both tasks can be run. This can be as simple as creating a copy of the navigation node.  

For information about the navigation node and how to fly the drone, refer back to the documentation of Task 1.

## CV node

The computer vision node is located in `catkin_ws/src/equinor_2018/scripts/cv.py`. Place the algorithm/model created in task 2 into this node. The camera feed from the drone is structered in the same way as the images in the dataset in task 2.  

In addition to integrating your computer vision algorithm with ROS, you will also need to provide some simple functionality to send your prediction once the drone has reached the goal.

### Useful variables

`current_pose` is identical to the variable in `navigation.cpp`.

`three_channel_image` is an object of the class defined in `cv.py`. This object holds the newest image from the drones camera feed. To access the image use `three_channel_image.data`.

## Scoring

Teams are scored based on two conditions. Firstly teams are ranked by the number of images that were correctly predicted. Secondly tiebreakers are resolved based on the algorithm that finishes the task fastest.  

As in task 1 a leaderboard will be made based on the performance on provided maps. For the final new maps will be used.
