#include <ros/ros.h>
#include <gtest/gtest.h>
#include <grav_inv_dyn.h>

// Test fixture for invDyn_GravityComp class
class InvDynGravityCompTest : public ::testing::Test
{
protected:
  InvDynGravityCompTest()
  {
    // Initialize ROS node handle
    ros::NodeHandle nh;
    node_ = new invDyn_GravityComp(&nh);
  }

  ~InvDynGravityCompTest()
  {
    delete node_;
  }

  invDyn_GravityComp* node_;
};

// Test case for invDyn_GravityComp::run() method
TEST_F(InvDynGravityCompTest, RunTest)
{
  // Start the node in a separate thread
  boost::thread node_thread(&invDyn_GravityComp::run, node_);

  // Wait for the node to start up
  ros::Duration(1.0).sleep();

  // Stop the node
  ros::shutdown();
  node_thread.join();
}

// Main function for running the tests
int main(int argc, char** argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "invDyn_GravityComp_test");

  // Initialize Google Test
  ::testing::InitGoogleTest(&argc, argv);

  // Run all the tests
  return RUN_ALL_TESTS();
}
