# Task 2: Object detection

In task 2 teams are provided a labeled training dataset of a few hundred images from inside the maze from perspective of the drone. The goal is to create a computer vision system that should find and recognize numbers on walls in the images from the drones camera.

For this task no template is given. There is however a util.py file that helps you load in the images from the datasets. Note that teams need to unzip the datasets before running these.

It is important to keep in mind that this code should transfer to task 3, when it will be placed in the computer vision ros node. So checkout the task 3 documentation before getting started.

## Scoring

Scoring will be based on the accuracy of the system on the provided unlabeled dataset. The scoreboard will be updated using the csv file described below.

For the final score the judges will run the code on a new unlabeled test set. Due to this it is expected that the code is easy to run on a new dataset of the same structure as the ones provided.

## CSV Submission

The predictions should be formated as follows:

predictions
1
2
6
2

and saved as `team-<number>-predictions.csv` in the Task2 folder.

Note that there are no spaces or comma's in the file.