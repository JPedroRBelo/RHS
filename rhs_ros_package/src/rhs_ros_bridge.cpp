#include "ros/ros.h"
#include "std_msgs/String.h"
#include "rhs_ros_package/Response.h"
#include "rhs_ros_package/CmdId.h"
#include "rhs_ros_package/CmdPos.h"
#include "rhs_ros_package/CmdString.h"
#include "rhs_ros_package/CmdAngle.h"
#include "rhs_ros_package/CmdDirect.h"
#include "rhs_ros_package/Commands.h"
#include <algorithm>
#include <netdb.h>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <signal.h>
#include <string>
#include <sstream> 
#include <iostream>
#include <typeinfo>
#include <iomanip>
#include <algorithm>
using namespace std;



#define HOST_NAME "172.26.175.128"
#define PORT 6321           // the port client will be connecting to
#define MAX_SIZE 4096    // max number of bytes we can get at once
#define DELIMITER "<|>"


    int fd;
    int nBytes;
    char buf[MAX_SIZE];
    struct hostent *he;
    struct sockaddr_in server;


static void err(const char* s) {
    ROS_INFO("%s",s);
    exit(EXIT_FAILURE);
}


/*
        .=======================================.===================.
        |               CommandID               |   CommandType     |  
        :===========================================================:
        | 1E009820-008C-2E00-756C-E0CB4BB729EC  |     True        | 
        |---------------------------------------|-------------------|
        | 1E009820-008C-2E00-756C-E0CB4EE729FE  |     False       |
        '---------------------------------------'-------------------'
    */

enum CommandType{
        ActivateLeft = 1,
        ActivateRight = 2,
        DeactivateLeft =3,
        DeactivateRight = 4,
        HeadReset = 5,
        LeaveLeft = 6,
        LeaveRight = 7,
        LookAt = 8,
        LookFor = 9,
        Move = 10,
        Rotate = 11,
        SmellLeft = 12,
        SmellRight = 13,
        Speech = 14,
        TakeLeft = 15,
        TakeRight = 16,
        TasteLeft = 17,
        TasteRight = 18,
        Turn = 19,
        CancelCommands = 20,
        Error = 999
};

enum ParameterType{
  WithId = 1,
  WithPos = 2,
  WithAngle = 3,
  WithString = 4,
  WithoutParameter = 5
};


CommandType convertMsgRosToCommand(string type) 
{
    rhs_ros_package::Commands commands;
    if(type == commands.ACTIVATE_LEFT)     return ActivateLeft;    
    if(type ==  commands.ACTIVATE_RIGHT)   return ActivateRight;   
    if(type ==  commands.DEACTIVATE_LEFT)  return DeactivateLeft;  
    if(type ==  commands.DEACTIVATE_RIGHT) return DeactivateRight; 
    if(type ==  commands.HEAD_RESET)       return HeadReset; 
    if(type ==  commands.LEAVE_LEFT)       return LeaveLeft;       
    if(type ==  commands.LEAVE_RIGHT)      return LeaveRight;      
    if(type ==  commands.LOOK_AT)          return LookAt;      
    if(type ==  commands.LOOK_FOR)         return LookFor;         
    if(type ==  commands.MOVE)             return Move;         
    if(type ==  commands.ROTATE)           return Rotate;
    if(type ==  commands.SMELL_LEFT)       return SmellLeft;       
    if(type ==  commands.SMELL_RIGHT)      return SmellRight;      
    if(type ==  commands.SPEECH)           return Speech;
    if(type ==  commands.TAKE_LEFT)        return TakeLeft;       
    if(type ==  commands.TAKE_RIGHT)       return TakeRight;       
    if(type ==  commands.TASTE_LEFT)       return TasteLeft;       
    if(type ==  commands.TASTE_RIGHT)     return TasteRight;
    if(type ==  commands.TURN)            return Turn;
    if(type ==  commands.CANCEL_COMMANDS) return  CancelCommands;
    return Error;
}


bool checkParameters(CommandType cType, ParameterType pType){

    return false;
}

string generateUUID(){
    string cmd = "cat /proc/sys/kernel/random/uuid";
    string data;
    FILE * stream;
    const int max_buffer = 256;
    char buffer[max_buffer];
    cmd.append(" 2>&1");
    stream = popen(cmd.c_str(), "r");
    if (stream) {
      while (!feof(stream)){
        if (fgets(buffer, max_buffer, stream) != NULL){ 
          data.append(buffer);
        }
      }
      pclose(stream);
    }
    data.erase(std::remove(data.begin(), data.end(), '\n'), data.end());
    return data;
}

string toString(float number){
    stringstream ss;
    ss << number;
    return ss.str();
}

float toFloat(string str){
    return strtof(str.c_str(),0);
}

int toInt(string str){
    /*string::size_type sz;   // alias of size_t
    return  stoi(str,&sz);    */
  int number;
  stringstream ss(str);
  ss >> number;
  return number;
}

//Get substring before DELIMITER
string getNext(string str){
    size_t pos = str.find(DELIMITER); //First Delimiter
    return str.substr(0,pos);
}

//Get substring after first DELIMITER
string getAfter(string str){
    size_t pos = str.find(DELIMITER); //First Delimiter
    string subData = str.substr(pos+strlen(DELIMITER));
    return subData;
}

string getRespCommandId(string data){
    string subData = getNext(data); //First Parameter
    return subData;
}
bool getRespExecuted(string data){
    string subData = getAfter(data); //After first Parameter
    subData = getNext(subData); //First Parameter
    bool executed = toInt(subData);
    return executed;
}

string buildCommandWithId(string commandId, CommandType commandType, ParameterType parameterType, int id){
    string message = commandId+
    DELIMITER+toString(commandType)+
    DELIMITER+toString(parameterType)+
    DELIMITER+toString(id);
    return message;
}

string buildCommandWithPos(string commandId,CommandType commandType, ParameterType parameterType, float x, float y, float z){
    string message = commandId+
    DELIMITER+toString(commandType)+
    DELIMITER+toString(parameterType)+
    DELIMITER+toString(x)+
    DELIMITER+toString(y)+
    DELIMITER+toString(z);
    return message;
}

string buildCommandWithText(string commandId,CommandType commandType, ParameterType parameterType, string text){
    string message = commandId+
    DELIMITER+toString(commandType)+
    DELIMITER+toString(parameterType)+
    DELIMITER+text;
    return message;
}

string buildCommandWithAngle(string commandId,CommandType commandType, ParameterType parameterType, float angle){
    string message = commandId+
    DELIMITER+toString(commandType)+
    DELIMITER+toString(parameterType)+
    DELIMITER+toString(angle);
    return message;
}

string buildCommandWithoutPar(string commandId,CommandType commandType, ParameterType parameterType){
    string message = commandId+
    DELIMITER+toString(commandType)+
    DELIMITER+toString(parameterType);
    return message;
}

void sendMessage(string message){
  sprintf(buf, "%s", message.c_str());
  send(fd, buf, strlen(buf), 0);
}

void commandsWithIdCallback(const rhs_ros_package::CmdId::ConstPtr& msg)
{  
  string message = buildCommandWithId(msg->command.id.c_str(),convertMsgRosToCommand(msg->command.type), WithId,msg->objectId);
  sendMessage(message);
  ROS_INFO("Command Received: [ID: %s][Type: %s][ObjectId: %d] \n", msg->command.id.c_str(),msg->command.type.c_str(),msg->objectId);
}

void commandsWithPositionCallback(const rhs_ros_package::CmdPos::ConstPtr& msg)
{  
  string message = buildCommandWithPos(msg->command.id.c_str(),convertMsgRosToCommand(msg->command.type), WithPos,msg->position.x,msg->position.y,msg->position.z);
  sendMessage(message);
  ROS_INFO("Command Received: [ID: %s][Type: %s][Pos: (%f,%f,%f)]", msg->command.id.c_str(),msg->command.type.c_str(),msg->position.x,msg->position.y,msg->position.z);
}

void commandsWithStringCallback(const rhs_ros_package::CmdString::ConstPtr& msg)
{  
  string message = buildCommandWithText(msg->command.id.c_str(),convertMsgRosToCommand(msg->command.type), WithString,msg->message.c_str());
  sendMessage(message);
  ROS_INFO("Command Received: [ID: %s][Type: %s][Message: %s] \n", msg->command.id.c_str(),msg->command.type.c_str(),msg->message.c_str());
}

void commandsWithAngleCallback(const rhs_ros_package::CmdAngle::ConstPtr& msg)
{  
  string message = buildCommandWithAngle(msg->command.id.c_str(),convertMsgRosToCommand(msg->command.type), WithAngle,msg->angle);
  sendMessage(message);
  ROS_INFO("Command Received: [ID: %s][Type: %s][Angle: %f] \n", msg->command.id.c_str(),msg->command.type.c_str(),msg->angle);
}

void commandsWithoutParameterCallback(const rhs_ros_package::Commands::ConstPtr& msg)
{  
  string message = buildCommandWithoutPar(msg->id.c_str(),convertMsgRosToCommand(msg->type),WithoutParameter);
  sendMessage(message);
  ROS_INFO("Command Received: [ID: %s][Type: %s] \n", msg->id.c_str(),msg->type.c_str());
}

int main(int argc, char **argv)
{
    
    if ((he = gethostbyname(HOST_NAME)) == NULL) {
        err("gethostbyname");
    }
    if ((fd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        err("socket");
    }
    
    bzero(&server, sizeof(server));
    server.sin_family = AF_INET;
    server.sin_port = htons(PORT);
    server.sin_addr = *((struct in_addr *)he->h_addr);
    ROS_INFO("Connecting in %s:%d",HOST_NAME,PORT);
    if (connect(fd, (struct sockaddr *)&server, sizeof(struct sockaddr)) == -1) {
        err("connect");
    }
    ROS_INFO("Connected!");
    string msg;
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Publisher publResponse = n.advertise<rhs_ros_package::Response>("response", 1000);
  ros::Subscriber subId = n.subscribe("idcommands", 1000, commandsWithIdCallback);
  ros::Subscriber subPos = n.subscribe("poscommands", 1000, commandsWithPositionCallback);
  ros::Subscriber subAngle = n.subscribe("anglecommands", 1000, commandsWithAngleCallback);
  ros::Subscriber subString = n.subscribe("stringcommands", 1000, commandsWithStringCallback);
  ros::Subscriber subSimple = n.subscribe("simplecommands", 1000, commandsWithoutParameterCallback);



  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */

    std_msgs::String msg;
    rhs_ros_package::Response response;
        
    int size;
    ioctl(fd, FIONREAD, &size);
    if(size>0){
    nBytes = recv(fd, buf, MAX_SIZE, 0);
    buf[nBytes] = '\0';
    string data = buf;
    string respCmdId = getRespCommandId(data);
    bool respExecuted = getRespExecuted(data); 

    ROS_INFO("Command with Id %s was executed with %s.\n",respCmdId.c_str(),respExecuted ? "success":"failure"); 

    
    response.commandId = respCmdId;
    response.executed = respExecuted;
    publResponse.publish(response);       
  }
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  ros::spin();
  close(fd);
  return 0;
}