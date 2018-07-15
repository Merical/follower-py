#ifndef _COMMUNICATION_H
#define _COMMUNICATION_H

#define HF_LINK_NODE_MODEL 1

#include <vector>
#include <iostream>
#include <inttypes.h>
#include <fstream>
#include <boost/make_shared.hpp>
#include <vector>
#include <deque>
#include <queue>
#include "string.h"

extern std::string handsfree_state;
extern char handsfree_message[1024];
extern bool debugFlag;
//extern char* handsfree_message;
//extern int handsfree_state;

enum Command{
    SHAKING_HANDS,
    READ_SYSTEM_INFO,
    SET_MOTOR_PARAMETERS,
    SAVE_MOTOR_PARAMETERS,
    SET_CHASSIS_PARAMETERS,
    SAVE_CHASSIS_PARAMETERS,
    SET_HEAD_PARAMETERS,
    SAVE_HEAD_PARAMETERS,
    SET_ARM_PARAMETERS,
    SAVE_ARM_PARAMETERS,
    SET_GLOBAL_SPEED,
    READ_GLOBAL_SPEED,
    SET_ROBOT_SPEED,
    READ_ROBOT_SPEED,
    SET_MOTOR_SPEED,
    READ_MOTOR_SPEED,
    READ_MOTOR_MILEAGE,
    READ_GLOBAL_COORDINATE,
    READ_ROBOT_COORDINATE,
    CLEAR_COORDINATE_DATA,
    SET_HEAD_STATE,
    READ_HEAD_STATE,
    SET_ARM_STATE,
    READ_ARM_STATE,
    READ_IMU_BASE_DATA,
    READ_IMU_FUSION_DATA,
    READ_GPS_DATA,
    LAST_COMMAND_FLAG};


//typedef std::vector<uint8_t> Buffer;
enum ChassisTFType{
    DIFFERENTIAL2,
    DIFFERENTIAL4,
    OMNI3,
    OMINI4,
    MECANUM4,
    CARLIKE };

typedef struct{
    float p1;                         //1 is pid outside ring parameters : mileage loop
    float i1;
    float d1;
    float p2;                         //2 is pid inside ring parameters : speed loop
    float i2;
    float d2;
}MotorPID;

typedef struct {
    unsigned char motor_id;
    float encoder_num;    //the encoder sensor count when the  motor turning one circle
    float pwm_max;          //set the max value for pwm
    float pwm_dead_zone;     //when the pwm in this zone , the motor disable
    float speed_low_filter;      //0~1;  default = 0.3
    float protect_current;      //unit : A default = 1
    MotorPID pid;
}MotorParameters;

enum HeadType{
    HFANALOG,
    HFDIGITAL,
    OTHERS_HEAD};

typedef  struct{
    float  servo1;
    float  servo2;
    float  servo3;
}HeadDOFVector;

typedef  struct{
    float  pitch;
    float  roll;
    float  yaw;
}HeadPose;

typedef  struct{
    HeadType type;
    float speed_low_filter;
    HeadPose range;  // radian
    HeadPose offset;  // radian
    HeadPose id;
    unsigned char  imu_fusion_enalbe;
    unsigned char control_enable;
}HeadParameters;

typedef struct{
    unsigned short int year;
    unsigned char month;
    unsigned char date;
    unsigned char hour;
    unsigned char min;
    unsigned char sec;
}UtcTime;

typedef  struct{
    float  pitch;
    float  roll;
    float  yaw;
    float bar_altitude;   //unit : m
    float magnetic_angle;
}IMUSensorData;

typedef  struct{
    UtcTime uct_time;
    unsigned char satellite_num;
    float altitude;     //unit : m
    float ground_speed;   //unit: m/s
    unsigned int latitude;         //纬度 分扩大100000倍,实际要除以100000
    unsigned char nshemi;        //北纬/南纬,N:北纬;S:南纬
    unsigned int longitude;      //经度 分扩大100000倍,实际要除以100000
    unsigned char ewhemi;        //东经/西经,E:东经;W:西经
}GPSData;

enum ArmType{
    DOBOT1,
    DOBOT2,
    OTHERS_ARM };

typedef  struct{
    float  servo1;
    float  servo2;
    float  servo3;
    float  servo4;
    float  servo5;
    float  servo6;
    float  servo7;
    float  servo8;
}ArmDOFVector;

typedef  struct{
    ArmType type;
    float speed_low_filter;
    unsigned char dof;
    unsigned char imu_fusion_enalbe;
    unsigned char control_enable;
}ArmParameters;

typedef struct{
    float  servo1;
    float  servo2;
    float  servo3;
    float  servo4;
}ChassisDOFVector;

typedef struct {
    float  x;
    float  y;
    float  z;
}ChassisCoord;

typedef struct{
    ChassisTFType type;
    float wheel_radius;
    float body_radius;
    float speed_low_filter;
    float motor_pid_t;
    unsigned char dof;
    unsigned char simulation_model;
    unsigned char  imu_fusion_enalbe;
    unsigned char control_enable;
}ChassisParameters;

static const unsigned short int MESSAGE_BUFFER_SIZE = 120;
typedef struct HFMessage{
    unsigned char sender_id;
    unsigned char receiver_id;
    unsigned short int length;
    unsigned char data[MESSAGE_BUFFER_SIZE];
}HFMessage;

typedef struct {
    float  system_time;
    float  cpu_temperature;
    float  cpu_usage;
    float  battery_voltage;
    float  power_remain; // 0% ~ 100%
}SystemInfo;

enum Recstate{
    WAITING_FF1,
    WAITING_FF2,
    SENDER_ID,
    RECEIVER_ID,
    RECEIVE_LEN_H,
    RECEIVE_LEN_L,
    RECEIVE_PACKAGE,
    RECEIVE_CHECK
};

typedef struct{
    double x_cmd_value;
    double y_cmd_value;
    double theta_cmd_value;
    double head_servo1_cmd_value;
    double head_servo2_cmd_value;
    double wheel_cmd0_value;
    double wheel_cmd1_value;
    double wheel_cmd2_value;
}DriverValue;


class RobotParameters
{
public:
    RobotParameters(){
        degree_to_radian = 0.017453f;
        radian_to_degree = 57.2958f;
        memset(&motor_para , 0 , sizeof(motor_para));
        memset(&chassis_para , 0 , sizeof(chassis_para));
        memset(&head_para , 0 , sizeof(head_para));
        memset(&arm_para , 0 , sizeof(arm_para));
    }

public:
    MotorParameters motor_para;
    ChassisParameters chassis_para;
    HeadParameters head_para;
    ArmParameters arm_para;
    float degree_to_radian ,  radian_to_degree;
};

class RobotAbstract
{
public:
    RobotAbstract()
    {
        para = RobotParameters();

        /************************************system info*******************************************/
        memset(&system_info , 0 , sizeof(system_info));

        /************************************motor info*******************************************/


        /************************************Chassis**********************************************/
        memset(&expect_motor_speed , 0 , sizeof(expect_motor_speed));
        memset(&measure_motor_speed , 0 , sizeof(measure_motor_speed));
        memset(&expect_robot_speed , 0 , sizeof(expect_robot_speed));
        memset(&measure_robot_speed , 0 , sizeof(measure_robot_speed));
        memset(&expect_global_speed , 0 , sizeof(expect_global_speed));
        memset(&measure_global_speed , 0 , sizeof(measure_global_speed));
        memset(&measure_motor_mileage , 0 , sizeof(measure_motor_mileage));
        memset(&measure_global_coordinate , 0 , sizeof(measure_global_coordinate));
        memset(&measure_robot_coordinate , 0 , sizeof(measure_robot_coordinate));

        /************************************arm*************************************************/
        memset(&expect_arm_state , 0 , sizeof(expect_arm_state));
        memset(&measure_arm_state , 0 , sizeof(measure_arm_state));

        /************************************head************************************************/
        memset(&expect_head_state , 0 , sizeof(expect_head_state));
        memset(&measure_head_state , 0 , sizeof(measure_head_state));

        /************************************IMU Sensors******************************************/
        memset(&gyro_acc , 0 , sizeof(gyro_acc));
        memset(&magnetic_fusion , 0 , sizeof(magnetic_fusion));
    }

    /************************************system info*********************************************/
    SystemInfo system_info;   //(meter,meter,factor(0~1))

    /************************************robot parameters*********************************************/
    //unit  distances : metres
    //angle： radian    void chassisDatatUpdate(void);
    RobotParameters para;

    /************************************chassis************************************************/
    ChassisDOFVector  expect_motor_speed;   //(x1,x2,x3)(radian/s,radian/s,radian/s)
    ChassisDOFVector  measure_motor_speed;
    ChassisCoord   expect_robot_speed;   //(x,y,w)(meter/s,meter/s,radian/s) reference system:robot
    ChassisCoord   measure_robot_speed;
    ChassisCoord   expect_global_speed;  //(x,y,w)(meter/s,meter/s,radian/s) reference system:global such as /map /odmo ;
    ChassisCoord   measure_global_speed;
    ChassisDOFVector  measure_motor_mileage; //(x1,x2,x3)(radian,radian,radian)
    ChassisCoord   measure_global_coordinate;  //(x,y,w)(meter,meter,radian)
    ChassisCoord   measure_robot_coordinate;

    /************************************arm***************************************************/
    ArmDOFVector expect_arm_state;
    ArmDOFVector measure_arm_state;

    /************************************head**************************************************/
    HeadPose   expect_head_state;    //(pitch,roll,yaw)(radian,radian,radian)
    HeadPose   measure_head_state;

    /************************************IMU sensors********************************************/
    IMUSensorData gyro_acc , magnetic_fusion; //(pitch,roll,yaw)(radian,radian,radian)
    GPSData gps_data;
};

class Communication
{
    public:
        Communication(RobotAbstract* robot_)
        {
            robot = robot_;
            hf_link_node_model = 1;
            hf_link_ack_en = 0;
        }
        void StateMachine(unsigned char my_id_, unsigned char friend_id_, unsigned char port_num_)
        {
            my_id = my_id_;   //0x11 means slave ,  read Hands Free Link Manua.doc for detail
            friend_id = friend_id_;   // 0x01 means master
            port_num = port_num_;
        }
        unsigned char masterSendCommand(const Command command_state);
        void sendStruct(const Command command_type , unsigned char* p ,  unsigned short int len);
        void sendMessage(const HFMessage* tx_message_);
        unsigned char sendBuffer(int port_num, unsigned char* buffer, unsigned short int size);
        unsigned char byteAnalysisCall(const unsigned char rx_byte);
        unsigned char receiveStates(const unsigned char rx_data);
        unsigned char packageAnalysis(void);
        unsigned char readCommandAnalysis(Command command_state , unsigned char* p ,  unsigned short int len);
        unsigned char setCommandAnalysis( Command command_state , unsigned char* p ,  unsigned short int len);
        unsigned char my_id, friend_id;
        HFMessage rx_message, tx_message;
        unsigned char receive_package_renew[LAST_COMMAND_FLAG];
        RobotAbstract* robot;      //robot abstract pointer to hflink
        unsigned char hf_link_node_model;      //1   0 slave , 1 master
        unsigned char hf_link_ack_en;                //0   enable hflink ack

    private:
        Recstate receive_state_;
        int send_message_count ,receive_message_count ;
        unsigned char tx_buffer[MESSAGE_BUFFER_SIZE];
        unsigned char rx_buffer[MESSAGE_BUFFER_SIZE];
        unsigned char tran_buffer[MESSAGE_BUFFER_SIZE];
        unsigned int receive_check_sum_;
        short int byte_count_;
        short int receive_message_length_;
        unsigned int tx_buffer_length;
        unsigned int rx_buffer_length;
        unsigned char shaking_hands_state;     //1 Success   0 Failed
        float analysis_package_count;
        Command command_state_;
        unsigned char port_num;

};

class HF_HW
{
    public: 
        void hf_hw(unsigned char my_id_, unsigned char friend_id_, unsigned char port_num_)
        {
            my_id = my_id_;   //0x11 means slave ,  read Hands Free Link Manua.doc for detail
            friend_id = friend_id_;   // 0x01 means master
            port_num = port_num_;
            hflink_ = boost::make_shared<Communication>(&my_robot_);
        }
        unsigned char updateCommand(const Command &command);
        unsigned char sendHandsShake(const Command &command);

        inline RobotAbstract* getRobotAbstract()
        {
            return &my_robot_;
        }

        float systemInfo()
        {
            voltageOfBattery = getRobotAbstract() -> system_info.battery_voltage;
            //std:: cout << "The battery_voltage is "<< voltageOfBattery <<" V " <<  std::endl;
            
        }

        void paraInitial()
        {
            wheel_cmd_.resize(3,0);
            x_cmd_ = 0;
            y_cmd_ = 0;
            theta_cmd_ = 0;
            head_servo1_cmd_ = 0;
            head_servo2_cmd_ = 0;
            wheel_cmd_[0] = 0;
            wheel_cmd_[1] = 0;
            wheel_cmd_[2] = 0;
        }


        void paraUpdate(DriverValue commandVal )
        {
            x_cmd_ = commandVal.x_cmd_value;
            //std::cout << "LCH: The x_cmd_value is " << x_cmd_ << std::endl;
            y_cmd_ = commandVal.y_cmd_value;
            //std::cout << "LCH: The y_cmd_value is " << y_cmd_ << std::endl;
            theta_cmd_ = commandVal.theta_cmd_value;
            //std::cout << "LCH: The theta_cmd_value is " << theta_cmd_ << std::endl;
            head_servo1_cmd_ = commandVal.head_servo1_cmd_value;
            head_servo2_cmd_ = commandVal.head_servo2_cmd_value;
            wheel_cmd_[0] = commandVal.wheel_cmd0_value;
            wheel_cmd_[1] = commandVal.wheel_cmd1_value;
            wheel_cmd_[2] = commandVal.wheel_cmd2_value;

            if (x_cmd_ > 0) handsfree_state = "Going forward!";
            else if (x_cmd_ < 0) handsfree_state = "Going backward";
            if (theta_cmd_ > 0) handsfree_state = "Turing Left!";
            else if (theta_cmd_ < 0) handsfree_state = "Turing Right!";

/*          if (x_cmd_ > 0) handsfree_state = 1;
            else if (x_cmd_ < 0) handsfree_state = 2;
            if (theta_cmd_ > 0) handsfree_state = 3;
            else if (theta_cmd_ < 0) handsfree_state = 4;
*/
        }

        float  voltageOfBattery;
        unsigned char my_id, friend_id;

    private:
        unsigned char recv_buffer[MESSAGE_BUFFER_SIZE];
        boost::shared_ptr<Communication> hflink_;
        RobotAbstract my_robot_;
        unsigned char port_num;
        std::vector<double> wheel_pos_, wheel_vel_, wheel_eff_, wheel_cmd_;
        double x_, y_, theta_, x_cmd_, y_cmd_, theta_cmd_;
        double x_vel_, y_vel_, theta_vel_;

        double head_servo1_pos_, head_servo1_vel_, head_servo1_eff_;
        double head_servo2_pos_, head_servo2_vel_, head_servo2_eff_;
        double head_servo1_cmd_, head_servo2_cmd_;

        float pitch_val, roll_val, yaw_val;

    public:
        void writeBufferUpdate()
        {

            if (debugFlag == 1) std::cout << "LCH: writeBufferUpdate!" << std::endl;
            getRobotAbstract()->expect_motor_speed.servo1 = wheel_cmd_[0];
            getRobotAbstract()->expect_motor_speed.servo2 = wheel_cmd_[1];
            getRobotAbstract()->expect_motor_speed.servo3 = wheel_cmd_[2];

            getRobotAbstract()->expect_robot_speed.x = x_cmd_;
            getRobotAbstract()->expect_robot_speed.y = y_cmd_;
            getRobotAbstract()->expect_robot_speed.z = theta_cmd_;


            // the servo num is different
            getRobotAbstract()->expect_head_state.pitch  = head_servo1_cmd_;
            getRobotAbstract()->expect_head_state.yaw  = head_servo2_cmd_;
        }

        void readBufferUpdate()
        {
            if (debugFlag == 1) std::cout << "LCH: readBufferUpdate!" << std::endl;
//char* json_data;
            char sensor_data[80]={0};
            if (debugFlag == 1) std::cout << "LCH: readBufferUpdate start! " << std::endl;
            x_     =  getRobotAbstract()->measure_global_coordinate.x;
            y_     =  getRobotAbstract()->measure_global_coordinate.y;
            theta_ =  getRobotAbstract()->measure_global_coordinate.z;

            if (debugFlag == 1) std::cout << "LCH: Read vel " << std::endl;
            x_vel_ =  getRobotAbstract()->measure_robot_speed.x;
            y_vel_ =  getRobotAbstract()->measure_robot_speed.y;
            theta_vel_ =  getRobotAbstract()->measure_robot_speed.z;

            if (debugFlag == 1) std::cout << "LCH: Read IMU " << std::endl;
            pitch_val =  getRobotAbstract()->magnetic_fusion.pitch;
            roll_val =  getRobotAbstract()->magnetic_fusion.roll;
            yaw_val =  getRobotAbstract()->magnetic_fusion.yaw;
            if (debugFlag == 1) std::cout << "LCH: IMU DATA IS: PITCH " << pitch_val << "ROLL " << roll_val << "YAW " << yaw_val << std::endl;

            sprintf(sensor_data,"pitch%4f,roll%4f,yaw%4f",pitch_val,roll_val,yaw_val);
            if (debugFlag == 1) std::cout << "LCH: Sensor data is " << sensor_data << std::endl;
            

//          std::cout << "LCH: creating json data " << std::endl;
//            sprintf(sensor_data,"{\"sensors\":{\"pitch\":%0.2f,", pitch_val);
//            std::cout << "sprintf done" << std::endl;
//            strcat(json_data, sensor_data);
//            std::cout << "strcat done" << std::endl;
//            sprintf(sensor_data, "\"roll\":%0.2f,", roll_val);
//            strcat(json_data, sensor_data);
//            sprintf(sensor_data, "\"yaw\":%0.2f}}\r\n", yaw_val);
//            strcat(json_data, sensor_data);
//            std::cout << "LCH: The json_data is : " << json_data << std::endl;
//
            memcpy(&handsfree_message, sensor_data, sizeof(sensor_data));
//
            //std::cout << "LCH: Read wheel_pos \n" << std::endl;
            //wheel_pos_[0] =  getRobotAbstract()->measure_motor_mileage.servo1;
            //wheel_pos_[1] =  getRobotAbstract()->measure_motor_mileage.servo2;
            //wheel_pos_[2] =  getRobotAbstract()->measure_motor_mileage.servo3;

            //robot_state.battery_voltage =  getRobotAbstract()->system_info.battery_voltage;
            //robot_state.cpu_temperature =  getRobotAbstract()->system_info.cpu_temperature;
            //robot_state.cpu_usage =  getRobotAbstract()->system_info.cpu_usage;
            //robot_state.system_time =  getRobotAbstract()->system_info.system_time;

            //std::cout << "LCH: Read wheel_vel " << std::endl;
            //wheel_vel_[0] =  getRobotAbstract()->measure_motor_speed.servo1;
            //wheel_vel_[1] =  getRobotAbstract()->measure_motor_speed.servo2;
            //wheel_vel_[2] =  getRobotAbstract()->measure_motor_speed.servo3;

            //std::cout << "LCH: Read head_state" << std::endl;
            head_servo1_pos_ =  getRobotAbstract()->measure_head_state.pitch ;
            head_servo1_vel_ = 0 ;
            head_servo1_eff_ = 0 ;

            head_servo2_pos_ =  getRobotAbstract()->measure_head_state.yaw ;
            head_servo2_vel_ = 0 ;
            head_servo2_eff_ = 0 ;

        }
};

        
#endif
