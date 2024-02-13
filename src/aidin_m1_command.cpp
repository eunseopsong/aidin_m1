#include "func.cpp"

class JointCommandPublisher : public rclcpp::Node
{
public:
    JointCommandPublisher()
    : Node("jointcmd_publishing_node") //, op_mode(0)
    {
        // Create a publisher on the "/aidin_m1/ArmCmd" topic
        aidin_m1_command_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("/aidin_m1/RobotCmd", 100);

        // Create a timer to publish the number every 1 second
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&JointCommandPublisher::getJointCmdInput, this));
    }

private:
    void getJointCmdInput()
    {
        while(rclcpp::ok())
        {
            cout << "Choose control mode ( 1: standing / 2: running ) : " << '\n';
            cin >> op_mode;

            if(op_mode == 1) // joint position input
            {
                cout << "Enter the target joint position : " << '\n';
                cin >> th1 >> th2 >> th3;
                if (abs(th1) > 360 or abs(th2) > 360 or abs(th3) > 360){
                    cout << "Out of Joint Configuration!" << '\n';
                    continue;
                }
                th_cmd[0] = th1*PI/180;
                th_cmd[1] = th2*PI/180;
                th_cmd[2] = th3*PI/180;
            }
            else if(op_mode == 2) // EE pose input
            {
                char s;
                cout << "Enter the target position and the solution type (u for upper, l for lower) : " << '\n';
                cin >> x >> y >> z >> s;
                if(s == 'u')
                    up = true;
                else if(s == 'l')
                    up = false;
                else{
                    cout << "Wrong solution! Type it again." << '\n';
                }

                try{
                    Inverse_K(x, y, z, up, th_cmd);
                }
                catch(...){
                    cout << "Out of Joint Configuration!" << '\n';
                    continue;
                }
            }

            ////////////////// Publish Torque //////////////////
            std_msgs::msg::Float32MultiArray torque_msg;
            torque_msg.data.clear();

            torque_msg.data.push_back(LF_scap_output_torque);
            torque_msg.data.push_back(LF_hip_output_torque);
            torque_msg.data.push_back(LF_knee_output_torque);


            // Create a message and set its data
            auto joint_command_msg = std_msgs::msg::Float32MultiArray();
            joint_command_msg.data.clear();
            for(int i = 0; i < DoF; i++) {
                joint_command_msg.data.push_back(th_cmd[i]);
            }

            // Publish the message
            arm_command_pub->publish(joint_command_msg);
        }
    }

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr arm_command_pub;
    rclcpp::TimerBase::SharedPtr timer_;
    int op_mode;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointCommandPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}