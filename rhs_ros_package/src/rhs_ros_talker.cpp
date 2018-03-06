#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <ctype.h>
#include <string.h>
#include <algorithm>
#include <sstream>
#include "rhs_ros_package/Response.h"
#include "rhs_ros_package/CmdId.h"
#include "rhs_ros_package/CmdPos.h"
#include "rhs_ros_package/CmdString.h"
#include "rhs_ros_package/CmdAngle.h"
#include "rhs_ros_package/Commands.h"
#include <vector>



using namespace std;

const int NUM_COMMANDS = 20;
const int MAX_NUMBER_PARAMETERS = 5;


enum ParameterType{
  WithId, WithPos, WithAngle, WithString, WithoutParameter
};

struct parameters
{
  int nPar;
  vector<ParameterType> types;
};

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

string getTypeLabel(ParameterType parType){
  switch(parType){
    case WithId:            return "ID";
    case WithPos:           return "Position";
    case WithAngle:         return "Angle";
    case WithString:        return "String";
    case WithoutParameter:  return "Without Parameter";
    default: return "";
  }
}

string getCommandLabel(CommandType cmdType) 
{
  switch(cmdType){
    case ActivateLeft:    return "Activate Left";
    case ActivateRight:   return "Activate Right";
    case DeactivateLeft:  return "Deactivate Left";
    case DeactivateRight: return "Deactivate Right";
    case HeadReset:       return "Head Reset";
    case LeaveLeft:       return "Leave Left";
    case LeaveRight:      return "Leave Right";
    case LookAt:          return "Look At";
    case LookFor:         return "Look For";
    case Move:            return "Move";
    case Rotate:          return "Rotate";
    case SmellLeft:       return "Smell Left";
    case SmellRight:      return "Smell Right";
    case Speech:          return "Speech";
    case TakeLeft:        return "Take Left";
    case TakeRight:       return "Take Right";
    case TasteLeft:       return "Taste Left";
    case TasteRight:      return "Taste Right";
    case Turn:            return "Turn";
    case CancelCommands:  return "Cancel Commands";
    default:              return "";
  }
}

string getCommandDescription(CommandType cmdType) 
{
  switch(cmdType){
    case ActivateLeft:    return "Activate/Open a object with Left hand.";
    case ActivateRight:   return "Activate/Open a object with Right hand.";
    case DeactivateLeft:  return "Deactivate/Close a object with Left hand.";
    case DeactivateRight: return "Deactivate/Close a object with Right hand.";
    case HeadReset:       return "Reset the vision focus of the Humanoid Robot.";
    case LeaveLeft:       return "Leave Left";
    case LeaveRight:      return "Leave Right";
    case LookAt:          return "LookAt";
    case LookFor:         return "Look For";
    case Move:            return "Move";
    case Rotate:          return "Rotate";
    case SmellLeft:       return "Smell Left";
    case SmellRight:      return "Smell Right";
    case Speech:          return "Speech";
    case TakeLeft:        return "Take Left";
    case TakeRight:       return "Take Right";
    case TasteLeft:       return "Taste Left";
    case TasteRight:      return "Taste Right";
    case Turn:            return "Turn";
    case CancelCommands:  return "Cancel Commands";
    default:              return "";
  }
}

string convertCommandTypeToMsgROS(CommandType cmdType) 
{
  rhs_ros_package::Commands commands;
  switch(cmdType){
    case ActivateLeft:    return commands.ACTIVATE_LEFT;
    case ActivateRight:   return commands.ACTIVATE_RIGHT;
    case DeactivateLeft:  return commands.DEACTIVATE_LEFT;
    case DeactivateRight: return commands.DEACTIVATE_RIGHT;
    case HeadReset:       return commands.HEAD_RESET;
    case LeaveLeft:       return commands.LEAVE_LEFT;
    case LeaveRight:      return commands.LEAVE_RIGHT;
    case LookAt:          return commands.LOOK_AT;
    case LookFor:         return commands.LOOK_FOR;
    case Move:            return commands.MOVE;
    case Rotate:          return commands.ROTATE;
    case SmellLeft:       return commands.SMELL_LEFT;
    case SmellRight:      return commands.SMELL_RIGHT;
    case Speech:          return commands.SPEECH;
    case TakeLeft:        return commands.TAKE_LEFT;
    case TakeRight:       return commands.TAKE_RIGHT;
    case TasteLeft:       return commands.TASTE_LEFT;
    case TasteRight:      return commands.TASTE_RIGHT;
    case Turn:            return commands.TURN;
    case CancelCommands:  return commands.CANCEL_COMMANDS;
    default:              return "";
  }
}


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

/*list<ParameterType> getParameterType(CommandType cmdType)
{
  switch(cmdType){
    case ActivateLeft:    return {WithId};
    case ActivateRight:   return {WithId};
    case DeactivateLeft:  return {WithId};
    case DeactivateRight: return {WithId};
    case HeadReset:       return {WithoutParameter};
    case LeaveLeft:       return {WithId,WithPos};
    case LeaveRight:      return {WithId,WithPos};
    case LookAt:          return {WithId,WithPos};
    case LookFor:         return {WithString};
    case Move:            return {WithId,WithPos};
    case Rotate:          return {WithAngle,WithAngle};
    case SmellLeft:       return {WithoutParameter};
    case SmellRight:      return {WithoutParameter};
    case Speech:          return {WithString};
    case TakeLeft:        return {WithId};
    case TakeRight:       return {WithId};
    case TasteLeft:       return {WithoutParameter};
    case TasteRight:      return {WithoutParameter};
    case Turn:            return {WithId,WithPos};
    case CancelCommands:  return {WithoutParameter};
    default:              return {WithoutParameter};
  }
}*/

map<CommandType, vector<ParameterType> >  dictMap;

void initCommandsParameters()
{
  
  //WithId
  vector<ParameterType> type1;
  type1.push_back(WithId);

  //With Id and Pos
  vector<ParameterType> type2;
  type2.push_back(WithId);
  type2.push_back(WithPos);

  //With String
  vector<ParameterType> type3;
  type3.push_back(WithString);

  //With Angle
  vector<ParameterType> type4;
  type4.push_back(WithAngle);


  //Without Parameter
  vector<ParameterType> type5;
  type5.push_back(WithoutParameter);

  dictMap[ActivateLeft] = type1;
  dictMap[ActivateRight] = type1;
  dictMap[DeactivateLeft] = type1;
  dictMap[DeactivateRight] = type1;
  dictMap[HeadReset] = type5;
  dictMap[LeaveLeft] = type2;
  dictMap[LeaveRight] = type2;
  dictMap[LookAt] = type2;
  dictMap[LookFor] = type3;
  dictMap[Move] = type2;
  dictMap[Rotate] = type4;
  dictMap[SmellLeft] = type5;
  dictMap[SmellRight] = type5;
  dictMap[Speech] = type3;
  dictMap[TakeLeft] = type1;
  dictMap[TakeRight] = type1;
  dictMap[TasteLeft] = type5;
  dictMap[TasteRight] = type5;
  dictMap[Turn] = type2;
  dictMap[CancelCommands] = type5;
}
/*string GetStdoutFromCommand(string cmd) {

    string data;
    FILE * stream;
    const int max_buffer = 256;
    char buffer[max_buffer];
    cmd.append(" 2>&1");

    stream = popen(cmd.c_str(), "r");
    if (stream) {
    while (!feof(stream))
    if (fgets(buffer, max_buffer, stream) != NULL) data.append(buffer);
    pclose(stream);
    }
    return data;
*/

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
  /*
  std::ostringstream ss;
  cout << "teste\n";
  ss << system("cat /proc/sys/kernel/random/uuid");
  return ss.str();
  */
}


float getFloatInput(string message)
{
    cout << message;
    float value = 0;
    cin >> value;
    while(cin.fail()) {
      cout << "Error" << endl;
      cin.clear();
      cout << message;
      cin.ignore(256,'\n');
      cin >> value;
    }
    return value;
}

int getIntInput(string message)
{
    cout << message;
    int value = 0;
    cin >> value;
    while(cin.fail()) {
      cout << "Error" << endl;
      cin.clear();
      cout << message;
      cin.ignore(256,'\n');
      cin >> value;
    }
    return value;
}


void responseCallback(const rhs_ros_package::Response::ConstPtr& msg)
{  
  
  ROS_INFO("Command with Id %s was executed with %s.\n",msg->commandId.c_str(),msg->executed ? "success":"failure"); 
}

int main(int argc, char **argv)
{
  static vector<parameters> cmdParameters;

  /*(
    {ActivateLeft, {WithId,WithId}},
    {ActivateRight, {WithId,WithId}}
  );*/

   initCommandsParameters();

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher commandId_pub = n.advertise<rhs_ros_package::CmdId>("idcommands", 1000);
  ros::Publisher commandPos_pub = n.advertise<rhs_ros_package::CmdPos>("poscommands", 1000);
  ros::Publisher commandAngle_pub = n.advertise<rhs_ros_package::CmdAngle>("anglecommands", 1000);
  ros::Publisher commandString_pub = n.advertise<rhs_ros_package::CmdString>("stringcommands", 1000);
  ros::Publisher commandDirect_pub = n.advertise<rhs_ros_package::Commands>("simplecommands", 1000);
  ros::Subscriber response_sub = n.subscribe("response", 1000, responseCallback);

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
    rhs_ros_package::CmdId cmdId;
    rhs_ros_package::CmdPos cmdPos;
    rhs_ros_package::CmdAngle cmdAngle;
    rhs_ros_package::CmdString cmdString;
    rhs_ros_package::Commands cmdDirect;

    string ss;
    //ss << "hello world " << count;
    cout<< "\nRHS AVAILABLE COMMANDS:\n";
    for ( int i = ActivateLeft; i <= CancelCommands; i++ )
    {
      CommandType auxCmdType = static_cast<CommandType>(i);
      string cmdName = getCommandLabel(auxCmdType);
      transform(cmdName.begin(), cmdName.end(), cmdName.begin(), ::toupper);
      cout << "["<< std::setw(2) << std::setfill('0') << auxCmdType <<"]  "<< cmdName << "\n";
    }
    std::cout << "Insert a command number>>> ";
    int cmdNumber;
    cin >> cmdNumber;
    while(cin.fail()) {
        cout << "Error" << endl;
        cin.clear();
        std::cout << "Insert a command number>>> ";
        cin.ignore(256,'\n');
        cin >> cmdNumber;
    }

    CommandType cmdType;
    cmdType = static_cast<CommandType>(cmdNumber);
    string selectedCommand = getCommandLabel(cmdType);
    transform(selectedCommand.begin(), selectedCommand.end(), selectedCommand.begin(), ::toupper);
    std::cout << "Selected: "<< selectedCommand  << "\n\n";
    
    ParameterType parType;
    int nParameters = dictMap[cmdType].size();
    if(nParameters!=1){
      std::cout << "PARAMETER TYPES: "  << "\n";
      for(int i = 0; i < nParameters;i++){       
          cout << "["<< std::setw(2) << std::setfill('0') << i+1 <<"]  "<< getTypeLabel(dictMap[cmdType][i]) << "\n";        
      }
      cmdNumber = getIntInput("Insert the number of desired parameter>>> ");
      parType = dictMap[cmdType][cmdNumber-1];
    }else{
      parType = dictMap[cmdType][0];
    }
    std::cout << "Selected: " <<getTypeLabel(parType) << "\n";     
    switch(parType){
      case WithId:
      {
        int idObject = getIntInput("Insert the object ID>>> ");
        cmdId.command.id = generateUUID();
        cmdId.command.type = convertCommandTypeToMsgROS(cmdType);
        cmdId.objectId = idObject;
        commandId_pub.publish(cmdId);
        break;
      }
      case WithPos:
      {
        float positionX = 0;
        positionX = getFloatInput("Insert the position x>>> ");
        float positionY = 0;
        positionY = getFloatInput("Insert the position y>>> ");
        float positionZ = 0;
        positionZ = getFloatInput("Insert the position z>>> ");
        cout << positionX << " " << positionY << " " << positionZ << "\n";
        cmdPos.command.id = generateUUID();
        cmdPos.command.type = convertCommandTypeToMsgROS(cmdType);
        cmdPos.position.x = positionX;
        cmdPos.position.y = positionY;
        cmdPos.position.z = positionZ;
        commandPos_pub.publish(cmdPos);
        break;
      }
      case WithString:
      {
        cmdString.command.id = generateUUID();
        cmdString.command.type = convertCommandTypeToMsgROS(cmdType);
        string message;
        cout << "Insert the message>>> ";
        cin.ignore();
        cout << getline(cin,message);
        cout << message;
        cmdString.message = message;
        commandString_pub.publish(cmdString);
        break;
      }
      case WithAngle:{
        float angle = 0;
        angle = getFloatInput("Insert the angle (-180 to 180)>>> ");
        cmdAngle.command.id = generateUUID();
        cmdAngle.command.type = convertCommandTypeToMsgROS(cmdType);
        cmdAngle.angle = angle;
        commandAngle_pub.publish(cmdAngle);   
        break;
      }
      case WithoutParameter:{
        cmdDirect.id = generateUUID();
        cmdDirect.type = convertCommandTypeToMsgROS(cmdType);
        commandDirect_pub.publish(cmdDirect);
        break;
      }
      default:
        break;
    }


    //ROS_INFO("%s", cmdId.command.type.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
       


    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  ros::spin();
  return 0;
}