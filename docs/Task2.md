# Task 2: Object detection

In task 2 teams will recieve a labeled training dataset of a few hundred images from inside the maze from perspective of the drone. In these all of these images there is atleast one number create a computer vision system that should find and recognize numbers on walls in images from the drones camera.  

For this task no template is given. It is important to keep in mind that this code should transfer to task 3, when it will be placed in the computer vision ros node. So checkout the task 3 documentation before getting started.

## Scoring

Scoring will be based on the accuracy of the system on a provided unlabeled test dataset. To update the leaderboard send in a csv file with predictions in the same order as the test dataset. 

For the final score the judges will run the code on a new unlabeled test set. Due to this it is expected that the code is easy to run on a new dataset of the same structure as the ones provided.

## CSV Submission

The predictions should be formated as follows:

predictions
1
2
6
2

and saved as `team-<number>-predictions.csv`.