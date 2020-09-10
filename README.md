# Thesis for a WBC with Operational Space control for a mobile manipulator

OSF implementation based on the work of Khatib.

As of now the stack of tasks is implemented as recursive projections.

As of now the implementation uses a hybrid control, with the stack of tasks for both force and velocity tasks.


## To create a csv from a rosbag

Run the command:

>
> <code> rostopic echo -b Odometry.bag -p /gazebo/model_states > gazebo.csv </code>
>

## To save selected topics for EKF test

Run the command:

>
> <code> rosbag record /Robotino/commands/velocity /Robotino/odom /clock /gazebo/model_states </code>
>