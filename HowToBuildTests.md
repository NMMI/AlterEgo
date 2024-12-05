to build tests go to workspace/build and run "make run_tests"

sudo apt install python3-catkin-tools python3-osrf-pycommon

catkin build è meglio. puoi compilare i test senza eseguirli con
catkin build  --catkin-make-args tests



how to run them:
rostest alterego_state_publisher test_launch.test 