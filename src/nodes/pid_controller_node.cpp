#include "pid_example/pid_controller_node.h"

/// Main function. The ROS node is initialized here and the main loop is entered.
/// \param argc The number of command line arguments.
/// \param argv Command line arguments
/// \return Returns 0 if the ROS node is properly terminated.
///

int main(int argc, char **argv)
{
    // Setup ROS node
    ros::init(argc, argv, "pid_controller_node");
    ros::NodeHandle nh;

    // Make a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters
    ros::NodeHandle private_nh("~");

    // Declare variables that can be modified by launch file or command line.
    // ROS node rate (also the PID algorithm rate)
    int rate;
    double kp, ki, kd, u_max, u_min;

    // message for publishing value of controlled variable
    std_msgs::Float32 output_msg;

    // Initialize private node parameters
    private_nh.param("rate", rate, 100);
    private_nh.param("kp", kp, 0.0);
    private_nh.param("ki", ki, 0.0);
    private_nh.param("kd", kd, 0.0);
    private_nh.param("u_max", u_max, std::numeric_limits<double>::infinity());
    private_nh.param("u_min", u_min, -std::numeric_limits<double>::infinity());

    // Create a new PidController ROS object
    PidControllerRos *pid_controller = new PidControllerRos(kp,ki,kd);
    pid_controller->setUMax(u_max);
    pid_controller->setUMin(u_min);


    // Create a subscriber. The parameters of nh.subscribe() are: topic name, message queue length, the callback function,
    //  and the object on which the callback function is called.
    ros::Subscriber ref_sub = nh.subscribe("reference", 1, &PidControllerRos::referenceCallback, pid_controller);
    ros::Subscriber meas_sub = nh.subscribe("measurement", 1, &PidControllerRos::measurementCallback, pid_controller);

    // Publisher for control input
    ros::Publisher output_pub = nh.advertise<std_msgs::Float32>("output",1);

    // Tell ROS how fast to run this node
    ros::Rate r(rate);

    // Main Loop
    while (nh.ok())
    {
        // Run spin function at the beginning of the loop to acquire new data from ROS topics
        ros::spinOnce();
        output_msg.data = pid_controller->compute(pid_controller->getReference(), pid_controller->getMeasurement());

        // publish computed control input
        output_pub.publish(output_msg);

        // sleep node for the 1/rate seconds
        r.sleep();
    }

    return 0;
} // end main()