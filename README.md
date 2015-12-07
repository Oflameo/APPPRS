# APPPRS

##Instructions to test the Clustering and object tracking:
Assuming the `workspace` directory is a valid catkin src directory.
`rosbag play -l loggedData.bag`
`roslaunch appprs_main robo_stats.launch`

The tracked object IDs should be published at `/obj_id`. The points belonging to each clusters (1-6) will be published at the usual `/cluster_1`, `cluster_2` etc.

