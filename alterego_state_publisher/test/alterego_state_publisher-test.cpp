#include <gtest/gtest.h>
#include <alterego_state_publisher.h>

#include <thread>
#include <atomic>
#include <chrono>

// using namespace std;
using namespace std::chrono;
using namespace std::this_thread;
/*
TEST(TestSuite, alterego_state_publisher)
{
    ASSERT_TRUE(true);
}

TEST_F(MyTestFixture, TestName) {
    // Test code here, using the fixture
}

In summary, use TEST when you have a single test case that does not need any setup or teardown code, and use TEST_F when you have a test case that requires a fixture to set up the environment and clean up after the test
*/

std::string checkMessage(const alterego_msgs::AlteregoState &msg, size_t ndof)
{
    // Check scalar fields
    std::vector<std::pair<double, std::string>> scalar_fields = {
        {msg.right_meas_neck_shaft, "right_meas_neck_shaft"},
        {msg.left_meas_neck_shaft, "left_meas_neck_shaft"},
        {msg.right_weight, "right_weight"},
        {msg.left_weight, "left_weight"},
        {msg.right_wheel_enc, "right_wheel_enc"},
        {msg.left_wheel_enc, "left_wheel_enc"},
        {msg.right_wheel_pos, "right_wheel_pos"},
        {msg.left_wheel_pos, "left_wheel_pos"},
        {msg.right_wheel_vel, "right_wheel_vel"},
        {msg.left_wheel_vel, "left_wheel_vel"},
        {msg.mobile_base_pos_x, "mobile_base_pos_x"},
        {msg.mobile_base_pos_y, "mobile_base_pos_y"},
        {msg.mobile_base_lin_displacement, "mobile_base_lin_displacement"},
        {msg.mobile_base_lin_vel, "mobile_base_lin_vel"},
        {msg.yaw_angle, "yaw_angle"},
        {msg.mobile_base_ang_vel, "mobile_base_ang_vel"},
        {msg.pitch_angle, "pitch_angle"},
        {msg.pitch_rate, "pitch_rate"},
    };

    std::string non_zero_fields;
    for (const auto &field : scalar_fields)
    {
        if (field.first != 0.0)
        {
            if (!non_zero_fields.empty())
            {
                non_zero_fields += ", ";
            }
            non_zero_fields += field.second;
        }
    }

    // Lambda function to check vector fields
    auto checkVectorField = [&](const std::vector<double> &vec, const std::string &field_name)
    {
        if (vec.size() != ndof)
        {
            return field_name + " (size)";
        }
        for (double val : vec)
        {
            if (val != 0.0)
            {
                return field_name + " (value)";
            }
        }
        return std::string();
    };

    // Check vector fields using the lambda function
    std::vector<std::pair<std::vector<double>, std::string>> vector_fields = {
        {msg.right_meas_arm_shaft, "right_meas_arm_shaft"},
        {msg.right_meas_arm_m1, "right_meas_arm_m1"},
        // ... add all other vector fields here
    };

    for (const auto &field : vector_fields)
    {
        std::string non_zero_field = checkVectorField(field.first, field.second);
        if (!non_zero_field.empty())
        {
            if (!non_zero_fields.empty())
            {
                non_zero_fields += ", ";
            }
            non_zero_fields += non_zero_field;
        }
    }

    return non_zero_fields;
}

class AlterEgoStatePublisherTest : public ::testing::Test
{

private:
    bool must_run;
    std::thread run_loop_thread;

protected:
    void run_loop()
    {
        ros::Rate r(1 / publisher->dt);
        while (must_run)
        {
            publisher->StateInfo();
            publisher->unicycle_kinematics();
            r.sleep();
        }
    }

    /// @brief In the provided example, publisher is a boost::scoped_ptr object that holds a pointer to an instance of the alterego_state_publisher class. boost::scoped_ptr is a smart pointer that automatically deletes the object it points to when the scoped_ptr goes out of scope. This helps to manage the memory of the alterego_state_publisher object and ensures that it is properly deleted when the test fixture is destroyed. The reset function is a member function of boost::scoped_ptr that replaces the managed object with a new one. In the SetUp function, we use publisher.reset(new alterego_state_publisher(400)) to create a new alterego_state_publisher object with a rate of 400 and assign it to the publisher smart pointer. This ensures that the publisher object is properly initialized before each test case is executed.
    void SetUp() override
    {
        std::cout << "\n..........";
        std::string robot_name = std::getenv("ROBOT_NAME");
        AlterEgoVersion = 3;
        nh.getParam("/" + robot_name + "/AlterEgoVersion", AlterEgoVersion);
        must_run = true;
        publisher.reset(new alterego_state_publisher(400));
        ego_state_sub_ = nh.subscribe("/" + robot_name + "/alterego_state", 1, &AlterEgoStatePublisherTest::stateOutput, this);

        run_loop_thread = std::thread(&AlterEgoStatePublisherTest::run_loop, this);
    }

    void TearDown() override
    {
        must_run = false;
        run_loop_thread.join();
    }
    void stateOutput(const alterego_msgs::AlteregoState::ConstPtr &msg)
    {
        if (!received_message)
        {
            first_received_msg = *msg;
            received_message = true;
        }
        received_msg = *msg;
    }

    boost::scoped_ptr<alterego_state_publisher> publisher;
    ros::NodeHandle nh;
    ros::Subscriber ego_state_sub_;
    ros::Publisher random_publisher;
    alterego_msgs::AlteregoState received_msg;
    alterego_msgs::AlteregoState first_received_msg;
    bool received_message = false;
    int AlterEgoVersion;
};

TEST(InitTestSuite, CreationTest)
{
    ASSERT_NO_THROW(alterego_state_publisher object(400);) << "alterego_state_publisher was not succesfully created";
}

TEST_F(AlterEgoStatePublisherTest, StateIsRunning)
{

    // Wait for a message to be received on topic1
    ros::Duration(3.0).sleep();
    ASSERT_TRUE(received_message) << "alterego_state_publisher did not publish any information";
    int ndof_ = 0;
    if (AlterEgoVersion == 3)
        ndof_ = 6;
    else
        ndof_ = 5;
    std::string non_zero_fields = checkMessage(static_cast<const alterego_msgs::AlteregoState &>(received_msg), ndof_);
    ASSERT_TRUE(non_zero_fields.empty()) << "The non-zero fields are: " << non_zero_fields;
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "alterego_state_publisher");
    std::thread t([]
                  {while(ros::ok()) ros::spin(); });

    auto res = RUN_ALL_TESTS();

    ros::shutdown();
    sleep(2);
    return res;
}