#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <fstream>
#include <stdint.h>
#include <pa10controller_dummy/getJointConfig.h>
#include <pa10controller_dummy/setJointConfig.h>
#include <pa10controller_dummy/addToQueue.h>
#include <boost/algorithm/string.hpp>

#define LISTSIZE 11

typedef struct {
	bool gripper;
	float positions[7];
	float commands[7];
} queueItem;

using namespace std;

vector<queueItem> items;


int main(int argc, char **argv)
{
	ros::init(argc, argv, "pa10client");
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<pa10controller::setJointConfig>("pa10/setJointsConfig");
	//   ros::ServiceClient client = n.serviceClient<pa10controller::addToQueue>("pa10/addItemtoQueue");
	//	pa10controller::addToQueue config;

	pa10controller::setJointConfig config;

	config.request.gripper =  0;
	for (uint8_t i = 0; i < 7; i++) {
		config.request.commands[i] = 3;
		config.request.positions[i] = 0;
		if(i == 0)
			config.request.positions[i] = 0;
	}


	if (!client.call(config)) {
		ROS_ERROR("Unable to send config to PA10");
		return 0;
	}
	return 1;
	//

	/*
	ifstream infile;
	string sLine = "";
	infile.open("points.txt");
	uint32_t linecount = 0;

	while (!infile.eof()) {
		getline(infile, sLine);
		vector <string> fields;
		ROS_INFO("Format line %d %s",linecount,sLine.c_str());
       // std::cout << sLine <<std::endl;

        boost::split(fields,sLine,boost::is_any_of(";"));
		//for(int i = 0; i < fields.size(); i++)
        //    ROS_INFO(" %s ", fields[i].c_str());

        if (fields.size() == 16) {
			items.push_back(queueItem());
			items.back().gripper = boost::lexical_cast<bool>(fields[0]);
		    //ROS_INFO("Hertil");
            //printf("%d;",items.back().gripper);
			for (uint8_t i = 0; i < 7; i++) {
				items.back().positions[i] = boost::lexical_cast<float>(fields[i+1]);
		        //ROS_INFO("Hertil 2 %f",items.back().positions[i]);
				printf("%f;",items.back().positions[i]);
			}

			for (uint8_t i = 0; i < 7; i++) {
              //  ROS_INFO("space %d", (int)(i+8));
		      //  ROS_INFO("Hertil%sH", fields[i+8].c_str());
				items.back().commands[i] = boost::lexical_cast<float>(fields[i+8]);
		        //ROS_INFO("Hertil 4");
				printf("%f;",items.back().commands[i]);
			}
			printf("\n");

		} else
			ROS_INFO("Format error in line %d",linecount);

		linecount++;
	}
	infile.close();


	for (vector<queueItem>::iterator it = items.begin(); it != items.end(); it++) {
		queueItem item = *it;
		config.request.gripper =  item.gripper;
		for (uint8_t i = 0; i < 7; i++) {
			config.request.commands[i] = item.commands[i];
			config.request.positions[i] = item.positions[i];
		}

		if (it == (items.end()-1)) {
			config.request.endOfQueue = true;
		}

		if (!client.call(config)) {
			ROS_ERROR("Unable to send config to PA10");
			return 0;
		}*/
	//		std::cout <<"Next point"<< std::endl;
	//		int tmp = 0;
	//		std::cin >> tmp;
	//	}
}
