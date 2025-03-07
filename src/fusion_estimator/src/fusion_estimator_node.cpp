#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>
#include <functional>
#include <urdf/model.h>
#include <nav_msgs/Odometry.h>
#include <fusion_estimator/LowState.h>
#include <fusion_estimator/RobotState.h>
#include <fusion_estimator/FusionEstimatorTest.h>

#include "GO2FusionEstimator/Estimator/EstimatorPortN.h"
#include "GO2FusionEstimator/Sensor_Legs.h" 
#include "GO2FusionEstimator/Sensor_IMU.h" 

using namespace DataFusion;

class FusionEstimatorNode : public rclcpp::Node
{
public:
    explicit FusionEstimatorNode(ros::NodeHandle& nh) : nh_(nh)
    {
        LowState_subscriber = nh.subscribe<fusion_estimator::LowState>(
            "/robot_description/anymal/low_state", 1, &FusionEstimatorNode::LowStateCallback, this
        );

        FETest_publisher = nh.advertise<fusion_estimator::FusionEstimatorTest>("fusion_estimator_data", 10);
        SMXFE_publisher  = nh.advertise<nav_msgs::Odometry>("SMXFE_odom", 10);


        for(int i = 0; i < 2; i++)
        {
            EstimatorPortN* StateSpaceModel1_SensorsPtrs = new EstimatorPortN; // 创建新的结构体实例
            StateSpaceModel1_Initialization(StateSpaceModel1_SensorsPtrs);     // 调用初始化函数
            StateSpaceModel1_Sensors.push_back(StateSpaceModel1_SensorsPtrs);  // 将指针添加到容器中
        }

        Sensor_IMUAcc = std::make_shared<DataFusion::SensorIMUAcc>(StateSpaceModel1_Sensors[0]);
        Sensor_IMUMagGyro = std::make_shared<DataFusion::SensorIMUMagGyro>(StateSpaceModel1_Sensors[1]);
        Sensor_Legs = std::make_shared<DataFusion::SensorLegs>(StateSpaceModel1_Sensors[0]);


        ObtainParameter();
        ROS_INFO("Fusion Estimator Node Initialized (ROS1).");

    }

    void ObtainParameter();

private:

    void LowStateCallback(const fusion_estimator::LowState::ConstPtr &low_state)
    {
        // 创建自定义消息对象
        fusion_estimator::FusionEstimatorTest fusion_msg;
        fusion_msg.stamp = ros::Time::now();


        for(int i=0; i<3; i++){
            fusion_msg.data_check_a[0+i] = low_state->imu_state().accelerometer()[i];
            fusion_msg.data_check_a[3+i] = low_state->imu_state().rpy()[i];
            fusion_msg.data_check_a[6+i] = low_state->imu_state().gyroscope()[i];
        }

        for(int LegNumber = 0; LegNumber<4; LegNumber++)
        {
            for(int i = 0; i < 3; i++)
            {
                fusion_msg.data_check_b[LegNumber*3+i] = low_state->motor_state()[LegNumber*3+i].q();
                fusion_msg.data_check_c[LegNumber*3+i] = low_state->motor_state()[LegNumber*3+i].dq();
            }
        }

        // Start Estimation
        double LatestMessage[3][100]={0};
        static double LastMessage[3][100]={0};

        rclcpp::Clock ros_clock(RCL_SYSTEM_TIME);
        rclcpp::Time CurrentTime = ros_clock.now();
        double CurrentTimestamp =  CurrentTime.seconds();

        for(int i = 0; i < 3; i++)
        {
            LatestMessage[0][3*i+2] = low_state->imu_state().accelerometer()[i];
        }
        for(int i = 0; i < 9; i++)
        {
            if(LastMessage[0][i] != LatestMessage[0][i])
            {
                Sensor_IMUAcc->SensorDataHandle(LatestMessage[0], CurrentTimestamp);
                for(int j = 0; j < 9; j++)
                {
                    LastMessage[0][j] = LatestMessage[0][j];
                }
                break;
            }
        }
        for(int i=0; i<9; i++){
            fusion_msg.estimated_xyz[i] = StateSpaceModel1_Sensors[0]->EstimatedState[i];
        }

        for(int i = 0; i < 3; i++)
        {
            LatestMessage[1][3*i] = low_state->imu_state().rpy()[i];
        }
        for(int i = 0; i < 3; i++)
        {
            LatestMessage[1][3*i+1] = low_state->imu_state().gyroscope()[i];
        }
        for(int i = 0; i < 9; i++)
        {
            if(LastMessage[1][i] != LatestMessage[1][i])
            {
                Sensor_IMUMagGyro->SensorDataHandle(LatestMessage[1], CurrentTimestamp);
                for(int j = 0; j < 9; j++)
                {
                    LastMessage[1][j] = LatestMessage[1][j];
                }
                break;
            }
        }
        for(int i=0; i<9; i++){
            fusion_msg.estimated_rpy[i] = StateSpaceModel1_Sensors[1]->EstimatedState[i];
        }

        for(int LegNumber = 0; LegNumber<4; LegNumber++)
        {
            for(int i = 0; i < 3; i++)
            {
                LatestMessage[2][LegNumber*3+i] = low_state->motor_state()[LegNumber*3+i].q();
                LatestMessage[2][12+ LegNumber*3+i] = low_state->motor_state()[LegNumber*3+i].dq();
            }
            LatestMessage[2][24 + LegNumber] = low_state->foot_force()[LegNumber];
            fusion_msg.others[LegNumber] = low_state->foot_force()[LegNumber];
            fusion_msg.others[LegNumber] = fusion_msg.others[LegNumber];
        }
        for(int i = 0; i < 28; i++)
        {
            if(LastMessage[2][i] != LatestMessage[2][i])
            {
                Sensor_Legs->SensorDataHandle(LatestMessage[2], CurrentTimestamp);
                for(int j = 0; j < 28; j++)
                {
                    LastMessage[2][j] = LatestMessage[2][j];
                }
                break;
            }
        }
        for(int i=0; i<4; i++){
            for(int j=0; j<3; j++){
                fusion_msg.data_check_d[3 * i + j] = StateSpaceModel1_Sensors[0]->Double_Par[6 * i + j];
                fusion_msg.data_check_e[3 * i + j] = StateSpaceModel1_Sensors[0]->Double_Par[24 + 6 * i + j];
                fusion_msg.feet_based_position[3 * i + j] = StateSpaceModel1_Sensors[0]->Double_Par[48 + 6 * i + j];
                fusion_msg.feet_based_velocity[3 * i + j] = StateSpaceModel1_Sensors[0]->Double_Par[48 + 6 * i + j + 3];
            }
        }

        FETest_publisher.publish(fusion_msg);

        // 新增：构造标准 odometry 消息，并发布
        nav_msgs::Odometry SMXFE_odom;
        SMXFE_odom.header.stamp = fusion_msg.stamp;
        SMXFE_odom.header.frame_id = "SMXFE_odom";
        SMXFE_odom.child_frame_id = "GO2_SMXFEodom";

        // 使用 fusion_msg.estimated_xyz 的前 3 个元素作为位置
        SMXFE_odom.pose.pose.position.x = fusion_msg.estimated_xyz[0];
        SMXFE_odom.pose.pose.position.y = fusion_msg.estimated_xyz[3];
        SMXFE_odom.pose.pose.position.z = fusion_msg.estimated_xyz[6];

        // 使用 fusion_msg.estimated_rpy 的前 3 个元素（roll, pitch, yaw）转换为四元数
        tf2::Quaternion q;
        q.setRPY(fusion_msg.estimated_rpy[0], fusion_msg.estimated_rpy[3], fusion_msg.estimated_rpy[6]);
        SMXFE_odom.pose.pose.orientation = tf2::toMsg(q);
        // 发布 odometry 消息
        SMXFE_publisher->publish(SMXFE_odom);

    }

    void ObtainParameter()
    {
        // 获得机器狗运动学参数
        Sensor_Legs->KinematicParams  << 
        0.1934,  0.0465,  0.000,   0.0,  0.0955,  0.0,   0.0,  0.0, -0.213,   0.0,  0.0, -0.213,  0.022,
        0.1934, -0.0465,  0.000,   0.0, -0.0955,  0.0,   0.0,  0.0, -0.213,   0.0,  0.0, -0.213,  0.022,
        -0.1934, -0.0465,  0.000,   0.0, -0.0955,  0.0,   0.0,  0.0, -0.213,   0.0,  0.0, -0.213,  0.022,
        -0.1934,  0.0465,  0.000,   0.0,  0.0955,  0.0,   0.0,  0.0, -0.213,   0.0,  0.0, -0.213,  0.022;
        
        std::filesystem::path current_file(__FILE__);
        std::filesystem::path package_dir = current_file.parent_path().parent_path();
        std::filesystem::path urdf_path = package_dir / "cfg" / "go2_description.urdf";
        std::ifstream urdf_file(urdf_path);
        if (!urdf_file.is_open())
        {
            std::cout << "无法打开文件: " << urdf_path << "，使用默认值。" << std::endl;
            return;
        }
        // 读入文件内容
        std::string urdf_xml((std::istreambuf_iterator<char>(urdf_file)), std::istreambuf_iterator<char>());
        urdf_file.close();
        urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_xml);
        if (!model)
        {
            std::cout << "解析URDF失败: " << urdf_path << "，使用默认值。" << std::endl;
            return;
        }
        // 设置输出格式：固定小数点，保留四位小数
        std::cout << std::fixed << std::setprecision(4);
        // 定义腿名称顺序，与 KinematicParams 的行对应
        std::vector<std::string> legs = {"FL", "FR", "RL", "RR"};
        // 定义关节映射结构，每个关节在 13 维向量中的起始列号（每个关节占 3 列）
        struct JointMapping {
            std::string suffix; // 关节后缀，如 "hip_joint"
            int col;            // 起始列号
        };
        // 对每条腿，映射 hip, thigh, calf, foot_joint 对应的参数
        std::vector<JointMapping> jointMappings = {
            { "hip_joint",   0 },
            { "thigh_joint", 3 },
            { "calf_joint",  6 },
            { "foot_joint",  9 }
        };
        // 对每条腿更新各关节参数
        for (size_t i = 0; i < legs.size(); i++)
        {
            const std::string& leg = legs[i];
            for (const auto& jm : jointMappings)
            {
                // 拼接完整关节名称，如 "FL_hip_joint"
                std::string jointName = leg + "_" + jm.suffix;
                urdf::JointConstSharedPtr joint = model->getJoint(jointName);
                if (!joint)
                {
                    std::cout << "未找到关节: " << jointName << " (" << leg << ")，使用默认值: ";
                    std::cout << Sensor_Legs->KinematicParams.row(i).segment(jm.col, 3) << std::endl;
                }
                else
                {
                    urdf::Vector3 pos = joint->parent_to_joint_origin_transform.position;
                    Sensor_Legs->KinematicParams(i, jm.col)     = pos.x;
                    Sensor_Legs->KinematicParams(i, jm.col + 1) = pos.y;
                    Sensor_Legs->KinematicParams(i, jm.col + 2) = pos.z;
                    std::cout << "Obtained KinematicPar for " << jointName << ": ";
                    std::cout << Sensor_Legs->KinematicParams.row(i).segment(jm.col, 3) << std::endl;
                }
            }
            // 更新该腿 foot 连杆 collision 的球半径（存储在列 12）
            std::string footLinkName = leg + "_foot";
            urdf::LinkConstSharedPtr footLink = model->getLink(footLinkName);
            if (!footLink)
            {
                std::cout << "未找到连杆: " << footLinkName << " (" << leg << ")，使用默认值: " << Sensor_Legs->KinematicParams(i, 12) << std::endl;
            }
            else
            {
                if (footLink->collision && footLink->collision->geometry &&
                    footLink->collision->geometry->type == urdf::Geometry::SPHERE)
                {
                    urdf::Sphere* sphere = dynamic_cast<urdf::Sphere*>(footLink->collision->geometry.get());
                    if (sphere)
                    {
                        Sensor_Legs->KinematicParams(i, 12) = sphere->radius;
                        std::cout << "Obtained KinematicPar for " << footLinkName << ": " << Sensor_Legs->KinematicParams(i, 12) << std::endl;
                    }
                }
            }

            Sensor_Legs->Par_HipLength = std::sqrt(Sensor_Legs->KinematicParams(0, 3)*Sensor_Legs->KinematicParams(0, 3) + Sensor_Legs->KinematicParams(0, 4)*Sensor_Legs->KinematicParams(0, 4) + Sensor_Legs->KinematicParams(0, 5)*Sensor_Legs->KinematicParams(0, 5));
            Sensor_Legs->Par_ThighLength = std::sqrt(Sensor_Legs->KinematicParams(0, 6)*Sensor_Legs->KinematicParams(0, 6) + Sensor_Legs->KinematicParams(0, 7)*Sensor_Legs->KinematicParams(0, 7) + Sensor_Legs->KinematicParams(0, 8)*Sensor_Legs->KinematicParams(0, 8));
            Sensor_Legs->Par_CalfLength = std::sqrt(Sensor_Legs->KinematicParams(0, 9)*Sensor_Legs->KinematicParams(0, 9) + Sensor_Legs->KinematicParams(0, 10)*Sensor_Legs->KinematicParams(0, 10) + Sensor_Legs->KinematicParams(0, 11)*Sensor_Legs->KinematicParams(0, 11));
            Sensor_Legs->Par_FootLength = abs(Sensor_Legs->KinematicParams(0, 12));
        }

        // 获得IMU安装位置
        Eigen::Vector3d IMUPosition(-0.02557, 0, 0.04232);
        std::string jointName = "imu_joint";
        urdf::JointConstSharedPtr joint = model->getJoint(jointName);
        if (!joint)
        {
            std::cout << "未找到关节: " << jointName << ", 使用默认值： ";
            std::cout << IMUPosition.transpose() << std::endl;
        }
        else
        {
            urdf::Vector3 pos = joint->parent_to_joint_origin_transform.position;
            std::cout << "Obtained Position for " << jointName << ": ";
            std::cout << IMUPosition.transpose() << std::endl;
        }
        Sensor_IMUAcc->SensorPosition[0] = IMUPosition(0);
        Sensor_IMUAcc->SensorPosition[1] = IMUPosition(1);
        Sensor_IMUAcc->SensorPosition[2] = IMUPosition(2);
        Sensor_IMUMagGyro->SensorPosition[0] = IMUPosition(0);
        Sensor_IMUMagGyro->SensorPosition[1] = IMUPosition(1);
        Sensor_IMUMagGyro->SensorPosition[2] = IMUPosition(2);
    }


    // 传感器状态空间模型创建
    ros::Subscriber Lowstate_subscriber;
    ros::Publisher FETest_publisher;
    ros::PublisherSMXFE_publisher;

    std::vector<EstimatorPortN*> StateSpaceModel1_Sensors = {};// 容器声明

    std::shared_ptr<DataFusion::SensorIMUAcc> Sensor_IMUAcc; 
    std::shared_ptr<DataFusion::SensorIMUMagGyro> Sensor_IMUMagGyro; 
    std::shared_ptr<DataFusion::SensorLegs> Sensor_Legs; 

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fusion_estimator_node");
    ros::NodeHandle nh;         // 全局命名空间
    ros::NodeHandle nh_private("~");  // 私有命名空间，用于 ~param

    // 创建FusionEstimatorNode对象
    FusionEstimatorNode node(nh, nh_private);

    // 阻塞等待回调
    ros::spin();
    return 0;
}
