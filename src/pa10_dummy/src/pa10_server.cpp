#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/String.h>
#include "serial/serial.h"

#include <iostream>
#include <stdint.h>

//#include <plc_control/SendPLCCommand.h>
#include <pa10_dummy/getJointConfig.h>
#include <pa10_dummy/setJointConfig.h>
#include <pa10_dummy/addToQueue.h>
#include <pa10_dummy/clearJointQueue.h>

//#include <rsd_msgs/packML_command.h>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#include <boost/program_options.hpp>


#define REPLY_SIZE 3
#define TIMEOUT 2000

#define ADR_STATUS 				0x0000
#define ADR_CONTROL 			0x0004
#define ADR_ENABLE_SERVOS	 	0x001F
#define ADR_ERROR			 	0x001B
#define ADR_COMMIT_WRITE	 	0x0400

#define ADR_TARGET_POS_JOINT1 	0x0005
#define ADR_TARGET_POS_JOINT2 	0x0006
#define ADR_TARGET_POS_JOINT3 	0x0007
#define ADR_TARGET_POS_JOINT4 	0x000C
#define ADR_TARGET_POS_JOINT5 	0x000D
#define ADR_TARGET_POS_JOINT6 	0x000E
#define ADR_TARGET_POS_JOINT7 	0x000F
#define ADR_COMMAND_JOINT1	 	0x0014
#define ADR_COMMAND_JOINT2	 	0x0015
#define ADR_COMMAND_JOINT3	 	0x0016
#define ADR_COMMAND_JOINT4	 	0x0017
#define ADR_COMMAND_JOINT5	 	0x001C
#define ADR_COMMAND_JOINT6	 	0x001D
#define ADR_COMMAND_JOINT7	 	0x001E


#define MASK_CONTROL_ENABLE_CONTROLLER	1 << 0
#define MASK_CONTROL_ENABLE_VEL			1 << 1
#define MASK_CONTROL_ENABLE_POS			1 << 2
#define MASK_CONTROL_ENABLE_FF			1 << 3
#define MASK_CONTROL_CONFIGMODE			1 << 4
#define MASK_CONTROL_CONFIG_PI_GAIN		1 << 5
#define MASK_CONTROL_WRITECONFIG		1 << 6
#define MASK_CONTROL_CMD_GOTO			0x00000000
#define MASK_CONTROL_CMD_PT				0x01000000
#define MASK_CONTROL_CMD_PV				0x02000000


#define OFFSET_VLIMIT			8
#define MASK_VLIMIT				0xFF00
#define OFFSET_HEAD				28
#define MASK_HEAD				0xF << OFFSET_HEAD
#define OFFSET_TAIL				OFFSET_HEAD
#define MASK_TAIL				MASK_HEAD

#define PA10ROBOT 1   //cell 1 or cell 2
#define NOROBOT 1

const uint16_t jointPosAddresses[] = {ADR_TARGET_POS_JOINT1,ADR_TARGET_POS_JOINT2,ADR_TARGET_POS_JOINT3,ADR_TARGET_POS_JOINT4,ADR_TARGET_POS_JOINT5,ADR_TARGET_POS_JOINT6,ADR_TARGET_POS_JOINT7};
const uint16_t jointCmdAddresses[] = {ADR_COMMAND_JOINT1,ADR_COMMAND_JOINT2,ADR_COMMAND_JOINT3,ADR_COMMAND_JOINT4,ADR_COMMAND_JOINT5,ADR_COMMAND_JOINT6,ADR_COMMAND_JOINT7};

const uint32_t jointCmdLimits[] = {130379, 130379, 260759, 260759, 819200, 819200, 819200};

const uint32_t pigains[] = {0x00080001,0x00080001,0x00080001,0x00080001,0x00100001,0x00100001,0x00100001};

using namespace std;


//---------------------------------------------
// Protypes
//---------------------------------------------
//uint32_t readValue(unsigned char address);
bool writeValue(uint16_t address, uint32_t value);
bool setGripper(bool grip);

void getSetup (float *positions, float *commands);
void sendSetup(uint8_t head, float positions[7], float commands[7]);

//void runQueue();

float jointPositions[7];

bool getJointConfig_service(pa10_dummy::getJointConfig::Request  &req, pa10_dummy::getJointConfig::Response  &res);
bool setJoints_service(pa10_dummy::setJointConfig::Request  &req, pa10_dummy::setJointConfig::Response  &res);
bool addtoQueue_service(pa10_dummy::addToQueue::Request  &req, pa10_dummy::addToQueue::Response  &res);
bool clearJointQueue_service(pa10_dummy::clearJointQueue::Request  &req, pa10_dummy::clearJointQueue::Response  &res);

//bool startNode_service(rsd_msgs::packML_command::Request &req,rsd_msgs::packML_command::Response &res);
//bool stopNode_service(rsd_msgs::packML_command::Request &req,rsd_msgs::packML_command::Response &res);
//bool resetNode_service(rsd_msgs::packML_command::Request &req,rsd_msgs::packML_command::Response &res);

//serial::Serial device(portname.c_str(), 115200, serial::Timeout::simpleTimeout(1000));
serial::Serial *device; //(portname.c_str(), 115200, serial::Timeout::simpleTimeout(1000));
ros::ServiceClient addToQueue_client;
uint8_t speedfactor = 255;
uint32_t maxerror = 1000;

int32_t finalOffset = 0;
bool clearJointQueue = false;

typedef struct {
	uint8_t gripping;
	float positions[7];
	float commands[7];
} queueItem;

boost::mutex items_mtx;
vector<queueItem> items;

uint32_t readValue(unsigned char address)
/* **************************************************************************************
 * Input 	:	address of register
 * Output	:	Value from address
 *
 * Function :	Read value from Unity Link
 * **************************************************************************************/
{
	char command[128]; 						//Create register to hold command
	char replyChar[128];							//Create register to hold reply

	uint32_t result = 0;

	sprintf(command,"#R:%04X\n",address);	//Generate the command
    //device->setTimeout((int)TIMEOUT);
    device->write(command);//Request data from the device
	//device->readline(reply);	       		//Read line from the device
	std::string reply = device->readline();	       		//Read line from the device

    char *cstr = new char[reply.length() + 1];
    strcpy(cstr, reply.c_str());
 
	char* start = cstr+5;					//Remove the initial part of the line
	start[8]= '\0';							//Remove any excess from the string

	result = strtoul(start,&start,16);		//Convert the result from string to number

	return result;							//Return the result
}

bool writeValue(uint16_t address, uint32_t value)
/* **************************************************************************************
 * Input 	:	Address of register
 * 				Data to write
 * Output	:	true if successful
 *
 * Function :	Writes data to Unity Link
 * **************************************************************************************/
{
	char command[128] = "";								//Create register to hold command
	char reply[128] = "";								//Create register to hold reply

	sprintf(command,"#W:%04X %08X\n",address, value);	//Generate the command
    ROS_INFO("command #W:%04X %08X", address, value);
    device->write(command);								//Send command to the device
	std::string replyStr= device->readline();//Read the response
	ROS_DEBUG("return value %s Stop", replyStr.c_str());
    command[16] = '\0';
	ROS_DEBUG("%s",command);

	if(strcmp(reply,"#S_W\n") == 0) return true;		//If successful return true

	return false;										//If unsuccessful return false
}


bool getJointConfig_service(pa10_dummy::getJointConfig::Request  &req, pa10_dummy::getJointConfig::Response  &res)
/* **************************************************************************************
 * Input 	:	Request data from getJointConfig
 * Output	:	Return true if successful
 * 				Response data for getJointConfig
 *
 * Function :	Handles requests from ROS clients requesting the current Joint Configuration
 * **************************************************************************************/
{
	ROS_DEBUG("Requesting values from pa10");
	float positions[7];
	float commands[7];
#ifndef NOROBOT 
	getSetup(positions, commands);
#endif
	for(int i = 0; i < 7; i++) {
#ifndef NOROBOT 
		res.positions[i] = positions[i];
		res.commands[i] = commands[i];
#else
		res.positions[i] = jointPositions[i];
		res.commands[i] = 0;
#endif
	}

	return true;
}
bool clearJointQueue_service(pa10_dummy::clearJointQueue::Request  &req, pa10_dummy::clearJointQueue::Response  &res)
/* **************************************************************************************
 * Input 	:	Request data from clearJointQueue
 * Output	:	Return true if successful
 * 				Response data for clearJointQueue
 *
 * Function :	Handles requests from ROS clients requesting the current Joint Configuration
 * **************************************************************************************/
{
	ROS_DEBUG("Stoping robot pa10");
	clearJointQueue = true;

	return true;
}

/*
 * Send command to gripper to either open or close.
 *
 * returns: N/A
 * */
void getSetup (float *positions, float *commands)
/* **************************************************************************************
 * Input 	:	void
 * Output	:	Positions of joints
 * 				Commands of joints
 *
 * Function :	Read joint positions and commands
 * **************************************************************************************/
{

	//if positions requested read them
	if (positions != NULL) {
		for(int i = 0; i < 7; i++)
			if (i == 6)
				positions[i] = (((int32_t)readValue((jointPosAddresses[i]-4))-finalOffset) * 360) / (50.0f * pow(2.0,14));
			else
				positions[i] = (((int32_t)readValue(jointPosAddresses[i]-4)) * 360) / (50.0f * pow(2.0,14));

	}

	//if commands requested read them
	if (commands != NULL)
		for(int i = 0; i < 7; i++)
			commands[i] = readValue(jointCmdAddresses[i]-4);

}

void sendSetup(uint8_t head, float positions[7], float commands[7])
/* **************************************************************************************
 * Input 	:	Value to use in the for the head of the ring-buffer
 * 				Positions for the joints
 * 				Commands for the joints
 * Output	:	void
 *
 * Function :	Send complete configuration to controller
 * **************************************************************************************/
{
	ROS_DEBUG("Sending Setup");

	//Configure Controller
	int config = 0;
	config |= MASK_CONTROL_ENABLE_CONTROLLER;
	config |= MASK_CONTROL_ENABLE_POS;
	config |= ((speedfactor << OFFSET_VLIMIT) & MASK_VLIMIT);
	config |= MASK_CONTROL_CMD_PT;
//	config |= MASK_CONTROL_CMD_GOTO;
	config |= head << OFFSET_HEAD;
	writeValue(ADR_CONTROL, config);

	//Set speed scaler
	writeValue(ADR_ENABLE_SERVOS, 0x7F);

    
	//Send joint configurations
	for(int i = 0; i < 7; i++) {
		if (i == 6)
			writeValue(jointPosAddresses[i], ((positions[i] * 50 * pow(2,14))/360.0f) + finalOffset);
		else
			writeValue(jointPosAddresses[i], ((positions[i] * 50 * pow(2,14))/360.0f)); 
      // writeValue(jointCmdAddresses[i], pigains[i]);
      // (degrees*50*2^14)/360
//def degreeToInt(degree):
//    if degree>360 or degree < -360:
//        raise ValueError('Wrong angles')
 //   return (degree*50*2**14)/360
      //tmp = hex(tmpint & 0xffffffff)[2:-1]
      //std::stringstream stream;
      //stream << std::hex << your_int;
      //std::string result( stream.str() );
      //
      //  
      // 
      writeValue(jointCmdAddresses[i], (commands[i] * 15.0f ));
      //writeValue(jointCmdAddresses[i], (commands[i] * 50 * pow(2,14))/360.0f );
       }
    
	//Commit settings
	ROS_DEBUG("Committing");
	writeValue(ADR_COMMIT_WRITE,0);
}



uint8_t readTail() {
	uint32_t status = readValue(ADR_STATUS);
	return (status & MASK_TAIL) >> OFFSET_TAIL;
}

void enableController(){
	ROS_INFO("Enabling Controller");
	uint8_t tail = readTail();

	int config = 0;
	config |= MASK_CONTROL_ENABLE_CONTROLLER;
	config |= tail << OFFSET_HEAD;
	writeValue(ADR_CONTROL, config);

	//Set speed scaler
	writeValue(ADR_ENABLE_SERVOS, 0x7F);

	//Send joint configurations
	for(int i = 0; i < 7; i++) {
		writeValue(jointPosAddresses[i], 0x00);
		writeValue(jointCmdAddresses[i], 0x00);
	}

	//Commit settings
	ROS_DEBUG("Committing");
	writeValue(ADR_COMMIT_WRITE,0);
}

void disableController(){
	ROS_INFO("Disabling Controller");
	uint8_t tail = readTail();

	int config = 0;
	config |= tail << OFFSET_HEAD;
	writeValue(ADR_CONTROL, config);

	//Set speed scaler
	writeValue(ADR_ENABLE_SERVOS, 0x00);

	//Send joint configurations
	for(int i = 0; i < 7; i++) {
		writeValue(jointPosAddresses[i], 0x00);
		writeValue(jointCmdAddresses[i], 0x00);
	}

	//Commit settings
	ROS_DEBUG("Committing");
	writeValue(ADR_COMMIT_WRITE,0);
}


void sendRegulationParamaters() {

	ROS_INFO("Setting PI Levels");
	uint8_t tail = readTail();

	int config = 0;
	config |= MASK_CONTROL_CONFIGMODE;
	config |= MASK_CONTROL_WRITECONFIG;
	config |= MASK_CONTROL_CONFIG_PI_GAIN;
	config |= tail << OFFSET_HEAD;
	writeValue(ADR_CONTROL, config);

 
	//Send joint configurations
	for(int i = 0; i < 7; i++) {
		writeValue(jointCmdAddresses[i], pigains[i]);
	}

	//Commit settings
	ROS_INFO("Committing");
	writeValue(ADR_COMMIT_WRITE,0);
	//Send joint configurations
//	for(int i = 0; i < 7; i++) {
//		writeValue(jointCmdAddresses[i], pigains[i]);
//	}

	//Commit settings
//	ROS_INFO("Committing");
//	writeValue(ADR_COMMIT_WRITE,0);

}

bool running = true;

// Replacement SIGINT handler
void mySigIntHandler(int sig)
{
	ROS_WARN_STREAM("Hard Shutdown attempt");
	running = false;
	ros::shutdown();
}

bool queueSucess = false;

void runQueueBuffer()
/* **************************************************************************************
 * Input 	:	void
 * Output	:	void
 *
 * Function :	Executes current command queue
 * **************************************************************************************/
{
	ROS_INFO("Running queue");
	//Take Lock on times list, to ensure no changes while running
	items_mtx.lock();

	std::vector<queueItem>::iterator it = items.begin();
	queueItem item = *it;

	bool gripperpos = false;
	queueSucess = true;
	uint8_t head, tail;
	tail = readTail();
	head = tail;
	//	head = (head +1) % 16;
	while (it != items.end() && running && clearJointQueue == false) //Run untill the end of the queue
	{


		do {
			head = (head +1) % 16;
			ROS_DEBUG("head: %02d, tail: %02d",head,tail);
			sendSetup(head, item.positions, item.commands);	//Configure robot
			item = *++it; // Select next item
		} while (running && ((head +1) % 16 != tail && it != items.end()) && clearJointQueue == false);

		if (it == items.end()) break;


	//	if (gripperpos != item.gripping) {
	//		ROS_DEBUG("Waiting to grip");
		if(clearJointQueue == true){ 
			disableController();
			break;
		}
						
			tail = readTail();
			while (head != tail) {
				tail = readTail();
				ROS_DEBUG("Head vs Tail: %d vs %d",head, tail);
				ros::spinOnce();
				usleep(200 * 10);
			}

//			ROS_DEBUG("sleep for: %f s", (it-1)->commands[0]);
//			usleep(((it-1)->commands[0] + 50) * 10 * 1000);

//			ROS_DEBUG("Setting gripper to %x",item.gripping);
		/*	if (!setGripper(item.gripping)) {
				queueSucess = false;
				ROS_WARN("Queue failed");
				uint8_t i = 0;
				/do {
					head = (head +1) % 16;
					ROS_DEBUG("head: %02d, tail: %02d",head,tail);
					sendSetup(head, item.positions, item.commands);	//Configure robot
					item = *++it; // Select next item
				} while (running && ((head +1) % 16 != tail && it != items.end()) && ++i < 2);
				setGripper(false);
				break;
			}
			gripperpos = item.gripping;
			ROS_DEBUG("Grip done"); */
//		} 
  //  else {
//			ROS_DEBUG("Waiting for buffer to move");
//			while (head == tail && running) {
//				tail = readTail();
//				usleep(200 * 1000);
//			}
//		}
//		ROS_DEBUG("Resuming");

	}

	while (running && (head != tail)) {
		tail = readTail();
		usleep(200 * 1000);
	}


	ROS_DEBUG("Queue complete");
	items.clear();
	//Release items
	items_mtx.unlock();

}

bool addtoQueue_service(pa10_dummy::addToQueue::Request  &req, pa10_dummy::addToQueue::Response  &res)
/* **************************************************************************************
 * Input 	:	Request data from addToQueue
 * Output	:	Return true if successful
 * 				Response data for addToQueue
 *
 * Function :	Handles requests from ROS clients trying to add to current
 * 				Joint Configuration queue
 * **************************************************************************************/
{
	ROS_INFO("Adding item to queue");

	//Create queue item
	queueItem item1;
	for (uint8_t i = 0; i < 7; i++) {
		//		item.positions[i] = (float)jointPosLimits[i] * (req.positions[i] / 100.0f);
		//		item.commands[i] = (float)jointCmdLimits[i] * (req.commands[i] / 100.0f);
		item1.positions[i] = req.positions[i];
        if(fabs(item1.positions[i]) <= 0.0005)
            item1.positions[i] = 0.0005;

      ROS_INFO("joints %f",item1.positions[i]);
		item1.commands[i] = req.commands[i];
      ROS_INFO("joints %f",item1.commands[i]);
	}


	if (items.size() > 0)
		if (items.back().gripping != req.gripper) {
			item1.gripping = items.back().gripping;
			items_mtx.lock();			//Lock item queue
			item1.gripping = items.back().gripping;
			items.push_back(item1);		//Add to item queue
			items_mtx.unlock();			//Unlock item queue
		}

	item1.gripping = req.gripper;
	items_mtx.lock();			//Lock item queue
	items.push_back(item1);		//Add to item queue
	items_mtx.unlock();			//Unlock item queue

	//If last item i queue, start processing it.
	if (req.endOfQueue && running) {
		items.push_back(items.back());
		queueSucess = false;
		boost::thread workerThread = boost::thread(runQueueBuffer);
		boost::posix_time::seconds waitTime(1);
		while(!workerThread.timed_join(waitTime)) {
			ros::spinOnce();
		}

		ROS_WARN_COND(!queueSucess,"Queue failed");

		ROS_DEBUG_COND(queueSucess, "Queue sucessfull");
		res.sucess = queueSucess;
	}




	return true;
}


bool setJoints_service(pa10_dummy::setJointConfig::Request  &req, pa10_dummy::setJointConfig::Response  &res)
/* **************************************************************************************
 * Input 	:	Request data from setJointConfig
 * Output	:	Return true if successful
 * 				Response data for setJointConfig
 *
 * Function :	Handles requests from ROS clients trying to set current Joint Configuration
 * **************************************************************************************/
{
    ROS_INFO("setJoints_service call");
	//Create item
	queueItem item;
	for (uint8_t i = 0; i < 7; i++) {
#ifndef NOROBOT 
		item.positions[i] = req.positions[i];
	        if(fabs(item.positions[i]) < 0.0005)
    	        	item.positions[i] = 0.0005 ;
		item.commands[i] = req.commands[i];
      		ROS_INFO("setJoints_service %f %f",item.positions[i],item.commands[i]);
#else

	   for (uint8_t i = 0; i < 7; i++) {
		jointPositions[i] = req.positions[i];
           }
           return true;
#endif

	}
	//item.gripping = req.gripper;
	//setGripper(item.gripping);
        ROS_DEBUG("SendSetup command");
	//Send command to controller
	sendSetup((readTail()+1) % 16, item.positions, item.commands);

	return true;
}

ros::ServiceServer startnode_service;
ros::ServiceServer stopnode_service;
ros::ServiceServer resetnode_service;

ros::ServiceServer getjointconfigservice;
ros::ServiceServer setjointsservice;
ros::ServiceServer addToQueueService;
ros::ServiceServer clearJointQueueService;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pa10_server", ros::init_options::NoSigintHandler);
	signal(SIGINT, mySigIntHandler);
	namespace po = boost::program_options;

	po::options_description desc("Allowed options");
	desc.add_options()
	    		("help", "produce help message")
	    		("port",
                po::value<string>()->default_value("/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A901H77U-if00-port0"), "Set serial port")
//                po::value<string>()->default_value("/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A901H6XI-if00-port0"), "Set serial port")
//	    		  po::value<string>()->default_value("/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A9013GPN-if00-port0"), "Set serial port")
	    		("speedscale", po::value<int>()->default_value(100), "Set speed scaler")
	    		("maxerror", po::value<int>()->default_value(10000), "Set speed scaler")
	    		("getoffset", "Get offset from the final joint, overrides --offset")
	    		("offset", po::value<int>(), "Offset of the final joint")
	    		;

	po::positional_options_description p;
	p.add("port", -1);

	po::variables_map vm;
	po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
	po::notify(vm);
	

	if (vm.count("help") || !vm.count("port")) {
		cout << "rosrun pa10_control pa10_server port [options]" << endl;
		cout << desc << "\n";
		return 1;
	}

	string portname = vm["port"].as<string>();

	if (vm.count("speedscale") )
		speedfactor = (255.0f * (vm["speedscale"].as<int>() / 100.0f));
	else
		speedfactor = 255.0f;


	if (vm.count("maxerror") )
		maxerror = vm["maxerror"].as<int>();
	else
		maxerror = 10000;

	ROS_INFO("Speedfactor is %d",speedfactor);
	ROS_INFO("Starting PA10 control on %s",portname.c_str());
	//try{ 
		//device.open(portname.c_str(), 115200);
 
#ifndef NOROBOT 
        device = new serial::Serial(portname.c_str(), 115200, serial::Timeout::simpleTimeout(1000));
 	//}	
	//catch(cereal::Exception& e)
	if(!device->isOpen()){
		ROS_FATAL("Failed to open the serial port!!!");
		ROS_BREAK();
	}
	ROS_INFO("The serial port is opened.");

/*
	if (vm.count("getoffset") ) {
		finalOffset = readValue(ADR_TARGET_POS_JOINT7 - 4);
	} else if (vm.count("offset") )
		finalOffset = vm["offset"].as<int>();
	else
		finalOffset = 0;
   */

        finalOffset = 0;
	ROS_INFO("End Joint offset: %d",finalOffset);

	disableController();
	usleep(500 * 1000);
	enableController();
	usleep(500 * 1000);

        float commands[7];
	getSetup(NULL, commands);

	if ((uint32_t)commands[0] == 0x81600000) {
		std::stringstream error;
		error << boost::format("Unable to start PA10, status code: 0x%08X") % (uint32_t)commands[0];
		//res.sucess = false;
		//res.message = error.str();
		ROS_ERROR_STREAM(error.str());
		return true;
	}


	sendRegulationParamaters();

#endif
        //ros::NodeHandle nodehandle;
	//startnode_service = nodehandle.advertiseService("pa10/start", startNode_service);
	//stopnode_service = nodehandle.advertiseService("pa10/stop", stopNode_service);
	//resetnode_service = nodehandle.advertiseService("pa10/reset", resetNode_service);

	ROS_INFO("Starting services.");
	ros::NodeHandle nodehandle;
	//startnode_service.shutdown();
   getjointconfigservice = nodehandle.advertiseService("pa10/getJointConfig", getJointConfig_service);
	setjointsservice = nodehandle.advertiseService("pa10/setJointsConfig", setJoints_service);
//	addToQueueService = nodehandle.advertiseService("pa10/addItemtoQueue", addtoQueue_service);
//   clearJointQueueService = nodehandle.advertiseService("pa10/clearJointQueue", clearJointQueue_service);
 
	ROS_INFO("PA10 Server is idle.");

	ros::spin();

	return 0;
}



