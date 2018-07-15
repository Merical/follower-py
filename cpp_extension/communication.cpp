#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include "string.h"
#include "communication.h"
using namespace std;

/***********************************Send Function**************************/
unsigned char HF_HW::sendHandsShake(const Command &command)
{
    //unsigned char * recv_buff;
    unsigned char * data;
    int  recv_length ;
    unsigned char ack_ready_ = false;
    hflink_->StateMachine(0x01,0x11,port_num);

    //if (tcflush(port_num, TCIOFLUSH) != 0) perror("tchflush error");//printf("UpdateCommand:The queue has been flushed! \n");

    hflink_->masterSendCommand(command);
    //sleep(0.5);
    //std:://cout << "LCH: HANDS SHAKE sent !" << std::endl;
}

/***********************************updateCommand**************************/
unsigned char HF_HW::updateCommand(const Command &command)
{
    if (debugFlag == 1) cout << "LCH: UpdateCommand! " << endl;
    //unsigned char * recv_buff;
    unsigned char * data;
    int  recv_length ;
    unsigned char ack_ready_ = false;
    hflink_->StateMachine(0x01,0x11,port_num);

    //if (tcflush(port_num, TCIOFLUSH) != 0) perror("tchflush error");//printf("UpdateCommand:The queue has been flushed! \n");

    hflink_->masterSendCommand(command);
    sleep(0.05);

    while (!ack_ready_)
    {
        recv_length = read(port_num, recv_buffer, MESSAGE_BUFFER_SIZE);// CHECK_ERROR
        sleep(0.05);
        for (int i = 0 ; i < recv_length; i++)// CHECK_ERROR
        {
            if (hflink_->byteAnalysisCall(recv_buffer[i]))
            {
                // one package ack arrived
                ack_ready_ = true;
            }
        }

        
    }
    //printf("Received package OK! \n");
    return true;
}

/**********************************masterSendCommand***********************/
unsigned char Communication::masterSendCommand(const Command command_state)
{
    if (debugFlag == 1) cout << "LCH: MasterSendCommand! " << endl;
    switch (command_state)
    {
    case SHAKING_HANDS :
        sendStruct(command_state , (unsigned char *)&robot->measure_global_coordinate , sizeof(robot->measure_global_coordinate) );
        break;

    case READ_SYSTEM_INFO :
        sendStruct(command_state , NULL , 0);
        break;

    case SET_MOTOR_PARAMETERS :
        sendStruct(command_state , (unsigned char *)&robot->para.motor_para , sizeof(robot->para.motor_para) );
        break;

    case SAVE_MOTOR_PARAMETERS :
        sendStruct(command_state , NULL , 0);
        break;

    case SET_CHASSIS_PARAMETERS :
        sendStruct(command_state , (unsigned char *)&robot->para.chassis_para , sizeof(robot->para.chassis_para) );
        break;

    case SAVE_CHASSIS_PARAMETERS :
        sendStruct(command_state , NULL , 0);
        break;

    case SET_HEAD_PARAMETERS :
        sendStruct(command_state , (unsigned char *)&robot->para.head_para , sizeof(robot->para.head_para) );
        break;

    case SAVE_HEAD_PARAMETERS :
        sendStruct(command_state , NULL , 0);
        break;

    case SET_ARM_PARAMETERS :
        sendStruct(command_state , (unsigned char *)&robot->para.arm_para , sizeof(robot->para.arm_para) );
        break;

    case SAVE_ARM_PARAMETERS :
        sendStruct(command_state , NULL , 0);
        break;

    case SET_GLOBAL_SPEED :
        sendStruct(command_state , (unsigned char *)&robot->expect_global_speed , sizeof(robot->expect_global_speed));
        break;

    case READ_GLOBAL_SPEED :
        sendStruct(command_state , NULL , 0);
        break;

    case SET_ROBOT_SPEED :
        sendStruct(command_state , (unsigned char *)&robot->expect_robot_speed , sizeof(robot->expect_robot_speed));
        break;

    case READ_ROBOT_SPEED :
        sendStruct(command_state , NULL , 0);
        break;

    case SET_MOTOR_SPEED :
        sendStruct(command_state , (unsigned char *)&robot->expect_motor_speed, sizeof(robot->expect_motor_speed));
        break;

    case READ_MOTOR_SPEED :
        sendStruct(command_state , NULL , 0);
        break;

    case READ_MOTOR_MILEAGE :
        sendStruct(command_state , NULL , 0);
        break;

    case READ_GLOBAL_COORDINATE :
        sendStruct(command_state , NULL , 0);
        break;

    case READ_ROBOT_COORDINATE :
        sendStruct(command_state , NULL , 0);
        break;

    case CLEAR_COORDINATE_DATA :
        sendStruct(command_state , NULL , 0);
        break;

    case SET_HEAD_STATE :
        sendStruct(command_state , (unsigned char *)&robot->expect_head_state, sizeof(robot->expect_head_state));
        break;

    case READ_HEAD_STATE :
        sendStruct(command_state , NULL , 0);
        break;

    case SET_ARM_STATE :
        sendStruct(command_state , (unsigned char *)&robot->expect_arm_state , sizeof(robot->expect_arm_state ));
        break;

    case READ_ARM_STATE :
        sendStruct(command_state , NULL , 0);
        break;

    case READ_IMU_BASE_DATA :
        sendStruct(command_state , NULL , 0);
        break;

    case READ_IMU_FUSION_DATA :
        sendStruct(command_state , NULL , 0);
        break;

    case READ_GPS_DATA :
        sendStruct(command_state , NULL , 0);
        break;

    default :
        return 0;
        break;
    }
    return 1;
}

/**********************************sendStruct******************************/
void Communication::sendStruct(const Command command_type , unsigned char* p ,  unsigned short int len)
{
    if (debugFlag == 1) cout << "LCH: SendStruct! " << endl;
    tx_message.sender_id = my_id;
    tx_message.receiver_id = friend_id;
    tx_message.length=len+1;
    tx_message.data[0] = (unsigned char)command_type;
    if(len > 0)
    {
        memcpy(&tx_message.data[1] , p , len);
    }
    sendMessage(&tx_message);
}

/********************************sendMessage***********************/
void Communication::sendMessage(const HFMessage* tx_message_)
{

    if (debugFlag == 1) cout << "LCH: SendMessage! " << endl;
    unsigned int check_sum_=0;
    //printf("LiShenghao:The command_state is %02x \n",tx_message_->data[0]);
    //printf("LiShenghao:The command is %02x \n",tx_message_);
    tx_buffer[0]=0xff;
    check_sum_ += 0xff;
    if (debugFlag == 1) printf("LiShenghao:The tx_message is %02x",tx_buffer[0]);

    tx_buffer[1]=0xff;
    check_sum_ += 0xff;
    if (debugFlag == 1) printf("%02x",tx_buffer[1]);

    tx_buffer[2]=tx_message_->sender_id;
    check_sum_ += tx_buffer[2];
    if (debugFlag == 1) printf("%02x",tx_buffer[2]);

    tx_buffer[3]=tx_message_->receiver_id;
    check_sum_ += tx_buffer[3];
    if (debugFlag == 1) printf("%02x",tx_buffer[3]);

    tx_buffer[4]=(unsigned char)( tx_message_->length >> 8);  //LEN_H
    check_sum_ += tx_buffer[4];
    if (debugFlag == 1) printf("%02x",tx_buffer[4]);

    tx_buffer[5]=(unsigned char)tx_message_->length;   //LEN_L
    check_sum_ += tx_buffer[5];
    if (debugFlag == 1) printf("%02x",tx_buffer[5]);

    unsigned short int tx_i  = 0;
    for(tx_i = 0 ;  tx_i < tx_message_->length ; tx_i++)   //package
    {
        tx_buffer[ 6 + tx_i] = tx_message_->data[ tx_i ];
        check_sum_ += tx_buffer[6+tx_i];
        if (debugFlag == 1) printf("%02x",tx_buffer[ 6+tx_i ]);
    }

    check_sum_=check_sum_%255;
    tx_buffer[6+tx_i] = check_sum_;
    if (debugFlag == 1) printf("%02x \n",tx_buffer[6+tx_i]);

    tx_buffer_length= 6 + tx_message_->length + 1;

//#if HF_LINK_NODE_MODEL==0
    sendBuffer(port_num , tx_buffer , tx_buffer_length);
//#endif

    send_message_count++;
}

/*****************************sendBuffer***********************************/
unsigned char Communication::sendBuffer(int port_num, unsigned char* buffer, unsigned short int size)
{
    if (debugFlag == 1) cout << "LCH: SendBuffer! "<< port_num << endl;
    if (tcflush(port_num, TCIOFLUSH) != 0) perror("tchflush error");//printf("UpdateCommand:The queue has been flushed! \n");
    if (write(port_num, buffer,size) <= 0)
        printf("Send error,please check! \n");
    if (debugFlag == 1) cout << "LCH: SendBuffer Done! " << endl;
    return 0;
}


/***********************************************************************************/
/**********************************Recive Functions*********************************/

/**********************************byteAnalysisCall********************************/
unsigned char Communication::byteAnalysisCall(const unsigned char rx_byte)
{
    if (debugFlag == 1) cout << "LCH: byteAnaoysisCall! " << endl;
    //if(receiveStates(rx_data))
    if(receiveStates(rx_byte))
    {
        //receive a new message
        sleep(0.05);
        //printf("LCH: Begin to analysis package!\n");
        unsigned char package_update = packageAnalysis();
        if(package_update == 1) 
        {   
            analysis_package_count++;
//          if (tcflush(port_num, TCIOFLUSH) != 0) perror("tchflush error");//printf("UpdateCommand:The queue has been flushed! \n");
            //if (tcflush(port_num, TCIOFLUSH) == 0) printf("byteAnalysisCall: The queue has been flushed!\n");
            //else perror("tcflush error");
        }
        return package_update;

    }
    //else printf("LCH: Waiting for check_sum \n");
    return 0;
}

/**********************************receiveStates*************************************/
unsigned char Communication::receiveStates(const unsigned char rx_data)
{
    if (debugFlag == 1) printf ("LCH: receiveStates! the rx_data is -%02x- \n", rx_data); 
    //cout << "LCH: receiveStates! the rx_data is " << rx_data << endl;
    switch (receive_state_)
    {
    default:
        receive_state_ = WAITING_FF1;

    case WAITING_FF1:
        if (rx_data == 0xff)
        {
            receive_state_ = WAITING_FF2;
            receive_check_sum_ =0;
            receive_message_length_ = 0;
            byte_count_=0;
            receive_check_sum_ += rx_data;
            if (debugFlag == 1) cout << "LCH: receiveStates: FF1 " << endl;
        }
        break;

    case WAITING_FF2:
        if (rx_data == 0xff)
        {
            receive_state_ = SENDER_ID;
            receive_check_sum_ += rx_data;
            if (debugFlag == 1) cout << "LCH: receiveStates: FF2 " << endl;
        }
        else
        {
            receive_state_ = WAITING_FF1;
            if (debugFlag == 1) cout << "LCH: receiveStates: ERROR IN FF2 " << endl;
        }
        break;

    case SENDER_ID:
        //printf ("DBG RCV : recv head flag\n "); 
        rx_message.sender_id = rx_data ;
        if (rx_message.sender_id == friend_id)  //id check
        {
            receive_check_sum_ += rx_data;
            receive_state_ = RECEIVER_ID;
            if (debugFlag == 1) cout << "LCH: receiveStates: SENDER_ID " << endl;
        }
        else
        {
            printf("error , the sender_ID is not my friend \n");
            receive_state_ = WAITING_FF1;
        }
        break;

    case RECEIVER_ID:
        rx_message.receiver_id = rx_data ;
        if (rx_message.receiver_id == my_id)  //id check
        {
            receive_check_sum_ += rx_data;
            if (debugFlag == 1) cout << "LCH: receiveStates: RECEIVER_ID " << endl;
            receive_state_ = RECEIVE_LEN_H;
        }
        else
        {
            printf("error , the reciver_ID is not my_ID \n");
            receive_state_ = WAITING_FF1;
        }
        break;

    case RECEIVE_LEN_H:
        receive_check_sum_ += rx_data;
        receive_message_length_ |= rx_data<<8;
        receive_state_ = RECEIVE_LEN_L;
        if (debugFlag == 1) cout << "LCH: receiveStates: RECEIVER_LENGTH_H " << endl;
        break;

    case RECEIVE_LEN_L:
        receive_check_sum_ += rx_data;
        receive_message_length_ |= rx_data;
        rx_message.length = receive_message_length_;
        receive_state_ = RECEIVE_PACKAGE;
        if (debugFlag == 1) cout << "LCH: receiveStates: RECEIVER_LENGTH_L " << endl;
        //printf ("DBG RCV : recv length (%d) \n", rx_message.length); 
        break;

    case RECEIVE_PACKAGE:
        receive_check_sum_ += rx_data;
        rx_message.data[byte_count_++] = rx_data;
        if (debugFlag == 1) printf ("LCH: receiveStates: rx_data (%02x) \n", rx_data); 
        if(byte_count_ >= receive_message_length_)
        {
            receive_state_ = RECEIVE_CHECK;
            receive_check_sum_=receive_check_sum_ % 255;
        }
        //printf ("DBG RCV : recv message (%02x) \n", rx_message.data[byte_count_-1]); 
        break;

    case RECEIVE_CHECK:
        if (debugFlag == 1) cout << "LCH: receiveStates: CHECK_SUM " << endl;
        if(rx_data == (unsigned char)receive_check_sum_)
        {
            //printf("LiShenghao:the receive_message_is %02x",rx_message.sender_id);
            //printf("%02x",rx_message.receiver_id);
            //printf("%04x",rx_message.length);
            //printf("%s",rx_message.data);
            /*for(int k=0;k<=byte_count_;k++)
            {
                printf("%02x",rx_message.data[k]);
            }
            //printf("%02x \n",receive_check_sum_);*/
            receive_check_sum_=0;
            receive_state_ = WAITING_FF1;
            //printf("receive a message \n");
            receive_message_count ++ ;
            //printf ("DBG RCV : (%d) success recv package\n",receive_message_count );
            return 1;
        }
        else
        {
            printf("check sum error \n");
            receive_state_ = WAITING_FF1;
        }
        break;
    }
    return 0;
}

/**********************************packageAnalysis********************************/
unsigned char Communication::packageAnalysis(void)
{

    if (debugFlag == 1) cout << "LCH: packageAnalysis! " << endl;
    if(robot == NULL){
        printf(" error , the robot is NULL  \n");
        return 0;
    }
    //printf("LCH: PackageAnalysis start! \n");
    //printf("LCH: The rx_command_state is %02x \n",rx_message.data[0]);

    command_state_ = (Command) rx_message.data[0];
    //printf("LCH: The rx_command_state is %02x \n",command_state_);

    /*
    if (hf_link_node_model == 0)  //the slave need to check the SHAKING_HANDS"s state
    {
        if(shaking_hands_state==0 && command_state_ != SHAKING_HANDS) //if not  shaking hands
        e
            sendStruct(SHAKING_HANDS  , NULL , 0);
            return 1;
        }
    }*/

    unsigned char analysis_state =0;
    switch (command_state_)
    {
    case SHAKING_HANDS :
        analysis_state=setCommandAnalysis(command_state_ , (unsigned char *)&robot->measure_global_coordinate , sizeof(robot->measure_global_coordinate));
        break;

    case READ_SYSTEM_INFO :
        analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&robot->system_info , sizeof(robot->system_info));
        break;

    case SET_MOTOR_PARAMETERS :
        analysis_state=setCommandAnalysis(command_state_ , (unsigned char *)&robot->para.motor_para  , sizeof(robot->para.motor_para));
        break;

    case SAVE_MOTOR_PARAMETERS :
        analysis_state=setCommandAnalysis(command_state_ , NULL  , 0);
        break;

    case SET_CHASSIS_PARAMETERS :
        analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&robot->para.chassis_para , sizeof(robot->para.chassis_para));
        break;

    case SAVE_CHASSIS_PARAMETERS :
        analysis_state=setCommandAnalysis(command_state_ , NULL  , 0);
        break;

    case SET_HEAD_PARAMETERS :
        analysis_state=setCommandAnalysis(command_state_ , (unsigned char *)&robot->para.head_para , sizeof(robot->para.head_para));
        break;

    case SAVE_HEAD_PARAMETERS :
        analysis_state=setCommandAnalysis(command_state_ , NULL  , 0);
        break;

    case SET_ARM_PARAMETERS :
        analysis_state=setCommandAnalysis(command_state_ , (unsigned char *)&robot->measure_global_coordinate , sizeof(robot->measure_global_coordinate));
        break;

    case SAVE_ARM_PARAMETERS :
        analysis_state=setCommandAnalysis(command_state_ , NULL  , 0);
        break;

    case SET_GLOBAL_SPEED :
        analysis_state=setCommandAnalysis(command_state_ , (unsigned char *)&robot->expect_global_speed , sizeof(robot->expect_global_speed));
        break;

    case READ_GLOBAL_SPEED :
        analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&robot->measure_global_speed , sizeof(robot->measure_global_speed));
        break;

    case SET_ROBOT_SPEED :
        analysis_state=setCommandAnalysis(command_state_ , (unsigned char *)&robot->expect_robot_speed , sizeof(robot->expect_robot_speed));
        break;

    case READ_ROBOT_SPEED :
        analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&robot->measure_robot_speed , sizeof(robot->measure_robot_speed));
        break;

    case SET_MOTOR_SPEED :
        analysis_state=setCommandAnalysis(command_state_ , (unsigned char *)&robot->expect_motor_speed, sizeof(robot->expect_motor_speed));
        break;

    case READ_MOTOR_SPEED :
        analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&robot->measure_motor_speed , sizeof(robot->measure_motor_speed));
        break;

    case READ_MOTOR_MILEAGE :
        analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&robot->measure_motor_mileage , sizeof(robot->measure_motor_mileage));
        break;

    case READ_GLOBAL_COORDINATE :
        analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&robot->measure_global_coordinate , sizeof(robot->measure_global_coordinate));
        break;

    case READ_ROBOT_COORDINATE :
        analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&robot->measure_robot_coordinate , sizeof(robot->measure_robot_coordinate));
        break;

    case CLEAR_COORDINATE_DATA :
        analysis_state=setCommandAnalysis(command_state_ , NULL  , 0);
        break;

    case SET_HEAD_STATE :
        analysis_state=setCommandAnalysis(command_state_ , (unsigned char *)&robot->expect_head_state, sizeof(robot->expect_head_state));
        break;

    case READ_HEAD_STATE :
        analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&robot->measure_head_state , sizeof(robot->measure_head_state));
        break;

    case SET_ARM_STATE :
        analysis_state=setCommandAnalysis(command_state_ , (unsigned char *)&robot->expect_arm_state, sizeof(robot->expect_arm_state));
        break;

    case READ_ARM_STATE :
        analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&robot->measure_arm_state , sizeof(robot->measure_arm_state));
        break;

    case READ_IMU_BASE_DATA :
        analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&robot->gyro_acc , sizeof(robot->gyro_acc));
        break;

    case READ_IMU_FUSION_DATA :
        analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&robot->magnetic_fusion , sizeof(robot->magnetic_fusion));
        break;

    case READ_GPS_DATA :
        analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&robot->gps_data, sizeof(robot->gps_data));
        break;

    default :
        analysis_state = 0;
        break;

    }

    rx_message.sender_id=0;    //clear flag
    rx_message.receiver_id=0;
    rx_message.length=0;
    rx_message.data[0]=0;

    //printf("Lishenghao: PackageAnalysis finished! \n");
    return analysis_state;
}

/********************************readCommandAnalysis********************************/
unsigned char Communication::readCommandAnalysis(Command command_state , unsigned char* p ,  unsigned short int len)
{
    if (debugFlag == 1) cout << "LCH: readCommandAnalysis! " << endl;
    if (hf_link_node_model == 1)
    { // master   , means the slave feedback a package to master , and the master save this package
        if((rx_message.length-1) != len)
        {
            //printf("I'm a master , can not read the message from slave , the length is not mathcing to struct \n");
            return 0;
        }
        memcpy(p , &rx_message.data[1] , len);
        receive_package_renew[(unsigned char)command_state] = 1 ;
    }
    else if(hf_link_node_model == 0)
    { // slave   , means the master pub a read command to slave ,and the slave feedback the a specific info to him
        sendStruct(command_state  , p , len);
        receive_package_renew[(unsigned char)command_state] = 1 ;
    }
    //printf("Lishenghao: readCommandAnalysis finished! \n");
    return 1;
}

/****************************************setCommandAnalysis*******************************************/
unsigned char Communication::setCommandAnalysis( Command command_state , unsigned char* p ,  unsigned short int len)
{

    if (debugFlag == 1) cout << "LCH: setCommandAnalysis! " << endl;
    if (hf_link_node_model == 1)
    { // master  , the slave can set the master's data ,so this code means received the slave's ack
        if(command_state == SHAKING_HANDS)
        {
            shaking_hands_state = 1;   //wait he master send SHAKING_HANDS
            //printf("received a SHAKING_HANDS commmand and the slave is waiting master send SHAKING_HANDS data ");
        }
        /*else
        {
            printf("I'm master , received a ack ");
        }*/
        receive_package_renew[(unsigned char)command_state] = 1 ;
    }
    else if(hf_link_node_model == 0)
    { // slave  , means the master pub a set command to slave ,and the slave save this package then feed back a ack

        receive_package_renew[(unsigned char)command_state] = 1 ;   //update receive flag , and wait the cpu to deal
        if(len > 0)
        {
            if( (rx_message.length-1) != len)
            {
                printf("I'm a slave , can not read the message from master , the length is not mathcing \n");
            }
            else memcpy(p , &rx_message.data[1] , len);
        }

        if(command_state == SHAKING_HANDS) shaking_hands_state=1;   //SHAKING_HANDS not need ack to master
        else sendStruct(command_state  , NULL , 0);   // returns a ack to master , i receive your set package
    }
    //printf("Lishenghao: setCommandAnalysis finished! \n");
    return 1;
}
