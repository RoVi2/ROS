/*
 * Path Planning Node
 *
 *  Created on: Mar 28, 2015
 *      Author: Jorge Rodriguez Marin
 */

#include <rw/rw.hpp>
#include <rw/math/Q.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/common.hpp>
#include <fstream>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include <queue>  // std::priority_queue

using namespace std;
using namespace rw;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::proximity;
using namespace rw::common;
using namespace rw::loaders;
using namespace rw::pathplanning;
using namespace rw::trajectory;
using namespace rwlibs::proximitystrategies;

const double maxDist=10;
const double threshold=0.05;

typedef std::pair< math::Q, math::Q> QBox;

class GraphNode {

private:

	Q _configuration;
	int _ID;
	vector<int> _connections;
	double _tempD;
	double _score;
	int _localPlannerCalls;
	int _localPlannerFails;
	double _failureRatio;
	double _nFailureRatio;
	vector<Q> _solution;

public:

	//Default constructor
	GraphNode(){
		_configuration=Q(6, 0,0,0,0,0,0);
		_ID=-1;
		_tempD=0;
		_score=-1;
		_localPlannerCalls=0;
		_localPlannerFails=0;
		_nFailureRatio=0;
		_failureRatio=0;
	}

	//Constructor
	GraphNode(Q q_config, int identifier){
		_configuration=q_config;
		_ID=identifier;
		_tempD=0;
		_score=-1;
		_localPlannerCalls=0;
		_localPlannerFails=0;
		_nFailureRatio=0;
		_failureRatio=0;
	}

	//Methods
	Q getConfig() const {return(_configuration);}
	int getID() const {return (_ID);}
	double getTempD() const {return(_tempD);}
	vector<int> getConnections() const {return(_connections);}
	double getScore() const {return(_score);}
	void setScore(double score) {_score=score;}
	void addSolution(){_solution.push_back(_configuration);}
	void addPreviousSolutions(GraphNode nodeToCopyFrom){
		for (auto q : nodeToCopyFrom._solution){
			_solution.push_back(q);
		}
	}
	vector<Q> getSolution(){return _solution;}

	double calculateMetrics(Q possibleNeighbour, Device::Ptr device) {
			Q q1, q2;
			q1=this->_configuration;
			q2=possibleNeighbour;
			double dist=0;
			for(size_t i=0; i<device->getDOF(); i++){
				dist+=pow(q1[i]-q2[i],2);
			}
			_tempD=dist;
			return (dist);
		}

		void newConnection(int newBrunchID){
			_connections.push_back(newBrunchID);
		}

		void localPlanner(bool result){
			_localPlannerCalls++;
			if(!result){
				_localPlannerFails++;
			}

			_failureRatio=(float)_localPlannerFails/((float)_localPlannerCalls+1);
		}

		double getFailureRatio(){
			return _failureRatio;
		}

		double getNFailureRatio(){
			return _nFailureRatio;
		}

		void setNFailureRatio(double total){
			_nFailureRatio=_failureRatio/total;
		}

	};

	//*************Check!!
	class Metrics{
		public:
			bool operator()(GraphNode N1, GraphNode N2)
			{
			   if (N2.getTempD()>N1.getTempD()) return true;
			   return false;
			}
	};

	//This function only avoids generating cycles of three nodes. For bigger cycles nodes must be used and the problem becomes exponential
	bool checkConnections(vector<int> newNodeCon, vector<int> neighbourCon, map<int, GraphNode> nodes){
		for(unsigned int i=0; i<newNodeCon.size(); i++){
			for(unsigned int j=0; j<neighbourCon.size(); j++){
				if(newNodeCon.at(i)==neighbourCon.at(j)){
					return false;
				}
			}
		}
		return true;
	}

	//************Check and test, specially the condition in the last if!!************
	bool collisionChecking4(Q q1, Q q2, Device::Ptr device, const State &state, const CollisionDetector &detector) {
		State testState;
		CollisionDetector::QueryResult data;

		double epsilon=0.1;
		Q dq = q2 - q1;
		double n = dq.norm2()/epsilon;
		double levels = ceil(log2(n));
		Q dq_extended = dq*(pow(2,levels)/n);
	    double steps;
		Q step;
		Q qi;
		Q q_add;
		for (int i=1; i<=levels; i++) {
			steps = pow(2, i-1);
			step = dq_extended/steps;
			for (int j=1; j<=steps; j++) {
				q_add = (j-1/2)*step;
				qi = q1 + q_add;
				testState=state;
				device->setQ(qi, testState);
				if ((q_add.norm2() < dq.norm2()) && detector.inCollision(testState, &data)) {
					return true;
				}
			}
		}
		return false;
	}


	bool randomConfiguration(Device::Ptr device, const State &state, const CollisionDetector &detector, Q &Qrand){
		State testState;
		CollisionDetector::QueryResult data;
		bool collision=true;

		int a = 0;
		while(collision){
			Qrand=Math::ranQ(device->getBounds());
			testState=state;
			device->setQ(Qrand, testState);
			collision=detector.inCollision(testState,&data);
			if (a>10000) {
					//Qrand.zero(6);
					//cout << "Fail to find a collision free configuration" << endl;	//*******What shall we do here??
					return false;
	    	}
	    	a++;
		}
		return true;
	}

	Q randomBounce(GraphNode nodeInit, Device::Ptr device, const State &state, const CollisionDetector &detector){
		State testState=state;
		CollisionDetector::QueryResult data;
		Q Qfin=nodeInit.getConfig();
		bool collision;
		QBox bounds=device->getBounds();
		Q Qmin=bounds.first;
		Q Qmax=bounds.second;

		while(nodeInit.calculateMetrics(Qfin, device)<maxDist){
			Q Qdir=Math::ranDir(6,0.1);
			collision=false;
			while(!collision && Qmin<(Qfin+Qdir) && (Qfin+Qdir)<Qmax){
				Qfin+=Qdir;
				testState=state;
				device->setQ(Qfin,testState);
				collision=detector.inCollision(testState,&data);
			}
		}

		return Qfin;
	}

	void addNodeToTree(Q configuration, Device::Ptr device, const State state, const CollisionDetector &detector, map <int, GraphNode> &PRMgraph,
			int &ID, 	priority_queue<GraphNode, vector<GraphNode>, Metrics> &candidateNeighbours, int & edgeCounter)
	{
		GraphNode newNode(configuration, ID);
		//cout<<"New configuration: "<<newNode.getConfig()<<" ID: "<<newNode.getID()<<endl;

		int sizeNc=0;
			//Go throught the graph looking for neighbours closer than maxDist and creates Nc
		for(map<int,GraphNode>::iterator it = PRMgraph.begin(); it != PRMgraph.end(); ++it) {
				//If distance<=maxDist, we store the node in Nc (priority queue sorted by distance)
			if(newNode.calculateMetrics(PRMgraph.find(it->second.getID())->second.getConfig(), device)<=maxDist){
				candidateNeighbours.push(PRMgraph.find(it->second.getID())->second);
				//cout<<"-----Found neighbour: "<<it->second.getID()<<" Distance: "<<newNode.calculateMetrics(PRMgraph.find(it->second.getID())->second.getConfig(), device)<<endl;
			}
		}

		//Go through the set of neighbours
		while(!candidateNeighbours.empty()){
			//Check if there is a graph connection already ------> avoid cycles
			if(checkConnections(newNode.getConnections(), candidateNeighbours.top().getConnections(), PRMgraph)){
				//Check for collisions in the edges
				if(!collisionChecking4(newNode.getConfig(), candidateNeighbours.top().getConfig(), device, state, detector)){
					//Create the connection in the graph (update list of connections in both nodes)
					newNode.newConnection(candidateNeighbours.top().getID());									//New connection in the new node
					PRMgraph.find(candidateNeighbours.top().getID())->second.newConnection(newNode.getID());	//New connection in the node already in the graph
					//cout<<"Edge created between "<<newNode.getID()<<" and "<<candidateNeighbours.top().getID()<<endl;
					edgeCounter++;
					sizeNc++;
					//edgesLimit++;
					//A maximum of 30 neighbours are connected and then the queue is empty
					if(sizeNc>=30){
						while(!candidateNeighbours.empty()){
							candidateNeighbours.pop();
						}
					}
					newNode.localPlanner(true);
					PRMgraph.find(candidateNeighbours.top().getID())->second.localPlanner(true);

				}
				else{
					//cout<<"Collision detected in the edge"<<endl;
					newNode.localPlanner(false);
					PRMgraph.find(candidateNeighbours.top().getID())->second.localPlanner(false);
				}
			}
			else{
				//cout<<"Nodes already graph connected"<<endl;
			}
			candidateNeighbours.pop();
		}

		//Add new node to the PRM
		PRMgraph[newNode.getID()]=newNode;
		ID++;
	}

	void expandTree(Device::Ptr device, const State state, const CollisionDetector &detector, map <int, GraphNode> &PRMgraph, int &ID, int &edgeLimit){

		priority_queue<GraphNode, vector<GraphNode>, Metrics> candidateNeighbours2;
		double total=0;
		for(map<int,GraphNode>::iterator it = PRMgraph.begin(); it != PRMgraph.end(); ++it) {
			total+=PRMgraph.find(it->second.getID())->second.getFailureRatio();
		}

		double max=0;
		int max_index=0;
		for(map<int,GraphNode>::iterator it = PRMgraph.begin(); it != PRMgraph.end(); ++it) {
			PRMgraph.find(it->second.getID())->second.setNFailureRatio(total);
			if(PRMgraph.find(it->second.getID())->second.getNFailureRatio()>max){
				max=PRMgraph.find(it->second.getID())->second.getNFailureRatio();
				max_index=it->second.getID();
			}
			//cout << "Failure ratio of " << PRMgraph.find(it->second.getID())->second.getID() << " : " << PRMgraph.find(it->second.getID())->second.getNFailureRatio() << endl;
		}

		while(max>threshold){
			Q r=randomBounce(PRMgraph.find(max_index)->second, device, state, detector);
			addNodeToTree(r,device,state,detector,PRMgraph,ID, candidateNeighbours2, edgeLimit);
			//cout << "New configuration Q: " << r << endl;

			total=0;
			for(map<int,GraphNode>::iterator it = PRMgraph.begin(); it != PRMgraph.end(); ++it) {
				total+=PRMgraph.find(it->second.getID())->second.getFailureRatio();
			}

			max=0;
			max_index=0;
			for(map<int,GraphNode>::iterator it = PRMgraph.begin(); it != PRMgraph.end(); ++it) {
				PRMgraph.find(it->second.getID())->second.setNFailureRatio(total);
				if(PRMgraph.find(it->second.getID())->second.getNFailureRatio()>max){
					max=PRMgraph.find(it->second.getID())->second.getNFailureRatio();
					max_index=it->second.getID();
				}
				//cout << "Failure ratio of " << PRMgraph.find(it->second.getID())->second.getID() << " : " << PRMgraph.find(it->second.getID())->second.getNFailureRatio() << endl;
			}

			//cout << "Max failure: " << max << endl;
		}
	}


/**
 * Given a Graph and the Q of one of its nodes, return the ID
 * @param graph The graph to search in
 * @param q the Q to search
 * @return The ID of the Q inside the graph
 */
int findIDfromQ(map<int, GraphNode> & graph, Q & q){
	int ID = 0;
	for (unsigned int nodeIterator = 0; nodeIterator < graph.size(); nodeIterator++){
		if (q == graph[nodeIterator].getConfig()) ID = graph[nodeIterator].getID();
	}
	return ID;
}

/**
 * This method calculates the score of a node based on the A start algorithm.
 * The map's cost has two values f(x) = g(x) + h(x).
 * g(x) is the already traveled distance.
 * h(x) is the heuristic score based on the distance left from a current state to the
 * goal.
 * @param node The node to calculate its score
 * @param previousNode The father of the node
 * @param goalNode The node to finish in
 * @return The score of the node
 */
float calculateAStarScore(GraphNode & node, double currentPathScore, Q goalQ, Device::Ptr device){
	//Calculates g(x)
	double g_x = currentPathScore;
	//Calculates h(x)
	double h_x = node.calculateMetrics(goalQ, device);
	//Stores the value in the node
	node.setScore(g_x + h_x);
	//Tadaaaaaa
	//cout << "Score for node " << node.getID() <<  " is: " << g_x  << "+" << h_x << " = " << g_x+h_x << endl;
	return g_x + h_x;
}


/**
 * Given a Graph calculates the shortest way from the Start node to the Goal node.
 * @param PRMgraph The graph to search the path from
 * @param startNode The node from which you start
 * @param goalNode The node you want to finish in
 * @return A vector with all the nodes followed
 */
vector<Q> calculatePath( map <int, GraphNode> & PRMgraph, Q startQ, Q goalQ, Device::Ptr device){
	//The open and closed list
	map<int, GraphNode> openList;
	map<int, GraphNode> closedList;
	//And the auxiliar node
	GraphNode * currentNode;

	//Lets find the ID of the start and goal states inside the graph
	int ID_start = findIDfromQ(PRMgraph, startQ);
	int ID_goal = findIDfromQ(PRMgraph, goalQ);

	//Starts with the startNode
	openList[ID_start] = PRMgraph[ID_start];
	currentNode = & openList[ID_start];
	//Put it in the solution vector
	currentNode->addSolution();
	////cout << "ID: " << openList[ID_start].getID() << " Connections: " << openList[ID_start].getConnections().size() << endl;
	//And calculate its score
	calculateAStarScore(*currentNode, 0, goalQ, device);

	//Reset the temporal variables, counter and limit
	int counter = 0;
	int limit = 10000;
	int tempScore=99999;
	int tempID=0;

	//While nodes on the open list
	while (!openList.empty() && counter<limit){
		tempScore = 99999;
		//Choose the node with the smallest score
		//cout << endl << "The openList size is: " << openList.size() << endl;
		for (auto node : openList){
			if (node.second.getScore()<=tempScore && node.second.getScore()>=0){
				tempScore = node.second.getScore();
				tempID = node.second.getID();
			}
		}
		currentNode = &openList[tempID];
		//cout << "ID: " << currentNode->getID() << " Score: " << currentNode->getScore() << " Connections: " << currentNode->getConnections().size() << endl;
		//Check if we are in the goal
		if (currentNode->getID()==ID_goal) return currentNode->getSolution();
		//We add the current map to the closed list
		closedList[currentNode->getID()] = *currentNode;
		//For all the connections, calculate the score of each one
		for (auto nodeConnected : currentNode->getConnections()){
			openList[nodeConnected] = PRMgraph[nodeConnected];
			openList[nodeConnected].addPreviousSolutions(*currentNode);
			openList[nodeConnected].addSolution();
			//cout << "  connected to:" << nodeConnected << endl;
			//Check that it is not in the closed list
			if (closedList.count(nodeConnected)==0 && openList[nodeConnected].getID()>0){
				//If so, calculate its score!
				calculateAStarScore(openList.find(nodeConnected)->second, currentNode->getScore(), goalQ, device);
			}
		}
		//Put it in the solution vector
		currentNode->addSolution();
		//And remove it from the openList
		openList.erase(currentNode->getID());
		//Updates the counter
		counter++;
	}

	//cout << "Solution not found" << endl;

	vector<Q> imsorry;
	return imsorry;
}

/**
 * Given a Graph calculates the shortest way from the Start node to the Goal node.
 * @param PRMgraph The graph to search the path from
 * @param startNode The node from which you start
 * @param goalNode The node you want to finish in
 * @return A vector with all the nodes followed
 */
vector<Q> calculatePathFromID( map <int, GraphNode> & PRMgraph, int ID_start, int ID_goal, Device::Ptr device){
	//The open and closed list
	map<int, GraphNode> openList;
	map<int, GraphNode> closedList;
	//And the auxiliar node
	GraphNode * currentNode;

	//Lets find the ID of the start and goal states inside the graph
	//int ID_start = findIDfromQ(PRMgraph, startQ);
	//int ID_goal = findIDfromQ(PRMgraph, goalQ);

	Q goalQ = PRMgraph[ID_goal].getConfig();

	//Starts with the startNode
	openList[ID_start] = PRMgraph[ID_start];
	currentNode = & openList[ID_start];
	//Put it in the solution vector
	currentNode->addSolution();
	////cout << "ID: " << openList[ID_start].getID() << " Connections: " << openList[ID_start].getConnections().size() << endl;
	//And calculate its score
	calculateAStarScore(*currentNode, 0, goalQ, device);

	//Reset the temporal variables, counter and limit
	int counter = 0;
	int limit = 10000;
	int tempScore=99999;
	int tempID=0;

	//While nodes on the open list
	while (!openList.empty() && counter<limit){
		tempScore = 99999;
		//Choose the node with the smallest score
		//cout << endl << "The openList size is: " << openList.size() << endl;
		for (auto node : openList){
			if (node.second.getScore()<=tempScore && node.second.getScore()>=0){
				tempScore = node.second.getScore();
				tempID = node.second.getID();
			}
		}
		currentNode = &openList[tempID];
		//cout << "ID: " << currentNode->getID() << " Score: " << currentNode->getScore() << " Connections: " << currentNode->getConnections().size() << endl;
		//Check if we are in the goal
		if (currentNode->getID()==ID_goal) return currentNode->getSolution();
		//We add the current map to the closed list
		closedList[currentNode->getID()] = *currentNode;
		//For all the connections, calculate the score of each one
		for (auto nodeConnected : currentNode->getConnections()){
			openList[nodeConnected] = PRMgraph[nodeConnected];
			openList[nodeConnected].addPreviousSolutions(*currentNode);
			openList[nodeConnected].addSolution();
			//cout << "  connected to:" << nodeConnected << endl;
			//Check that it is not in the closed list
			if (closedList.count(nodeConnected)==0 && openList[nodeConnected].getID()>0){
				//If so, calculate its score!
				calculateAStarScore(openList.find(nodeConnected)->second, currentNode->getScore(), goalQ, device);
			}
		}
		//Put it in the solution vector
		currentNode->addSolution();
		//And remove it from the openList
		openList.erase(currentNode->getID());
		//Updates the counter
		counter++;
	}

	//cout << "Solution not found" << endl;

	vector<Q> imsorry;
	return imsorry;
}

void createFile(string robotName){
	ofstream myFile;
	myFile.open("simulation.lua");

	myFile << "wc = rws.getRobWorkStudio():getWorkCell()" << endl;
	myFile << "state = wc:getDefaultState()" << endl;
	myFile << "device = wc:findDevice(\"" << robotName << "\")" << endl;
	myFile << "function setQ(q)" << endl;
	myFile << "qq = rw.Q(#q,q[1],q[2],q[3],q[4],q[5],q[6])" << endl;
	myFile << "device:setQ(qq,state)" << endl;
	myFile << "rws.getRobWorkStudio():setState(state)" << endl;
	myFile << "rw.sleep(1)" << endl;
	myFile << "end" << endl << endl;

	myFile.close();
}

/*****************************************************************************************************************************/
/******Function: addPosition												     */
/******Inputs: next state vector of the robot									             */
/******Outputs: input argument for the LUA function setQ								     */
/******This function calls the setQ function in the LUA script to move the robot to its next configuration updating its state*/
/*****************************************************************************************************************************/
void addPosition(vector<Q> q){
	ofstream myFile;
	myFile.open("simulation.lua",ios::app);

	for(size_t i=0; i<q.size(); i++){
		myFile <<"setQ({"<<q[i][0]<<","<<q[i][1]<<","<<q[i][2]<<","<<q[i][3]<<","<<q[i][4]<<","<<q[i][5]<<"})"<<endl;
	}
	myFile.close();
}


int main(int argc, char** argv) {

	//Initializing workcell
	Math::seed(time(NULL));
	cout << " --- Program started --- " << endl << endl;
	const string wcFile = "/home/veimox/Drive/Robot Systems/RoVi 2/Robotics 2/Projects/PRM/res/KukaKr16/Scene.wc.xml";
	const string deviceName = "KukaKr16";
	//cout << "Trying to use workcell " << wcFile << " and device " << deviceName << endl;

	WorkCell::Ptr wc = WorkCellLoader::Factory::load(wcFile);
	Device::Ptr device = wc->findDevice(deviceName);
	if (device == NULL) {
		cerr << "Device: " << deviceName << " not found!" << endl;
		return 0;
	}
	const State state = wc->getDefaultState();

	//Collision detector and strategy
	CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());

	//Graph: created as a map container. The key is the node's ID
	map <int, GraphNode> PRMgraph;
	PRMgraph.erase(PRMgraph.begin(), PRMgraph.end());
	int ID=0;

	//Create Nc
	priority_queue<GraphNode, vector<GraphNode>, Metrics> candidateNeighbours;

	//***********************************PRM ALGORITHM******************************************

	//***LEARNING PHASE*****//

	//1) CONSTRUCTION STEP
	int edgeCounter = 0;
	int edgeLimit = 1000;
	Timer edgeTimer;
	edgeTimer.reset();
	while(edgeCounter<edgeLimit){	//Limited to the creation of 20 edges (for testing)
		Q Qrandom;
		while(!randomConfiguration(device, state, detector, Qrandom)){}
		addNodeToTree(Qrandom, device, state, detector, PRMgraph, ID, candidateNeighbours, edgeCounter);
		cout << "Edge Number: " << edgeCounter << endl;
	}

	//END OF CONSTRUCTION STEP
	//cout<<"Size of the PRM: "<<PRMgraph.size()<<endl;

	//Just for testing
	/*for(map<int,GraphNode>::iterator it = PRMgraph.begin(); it != PRMgraph.end(); ++it) {
		//cout<<"Connections of node: "<<it->second.getID()<<", "<<endl;
		for(int i=0; i<PRMgraph.find(it->second.getID())->second.getConnections().size(); i++){
			//cout<<PRMgraph.find(it->second.getID())->second.getConnections()[i]<<endl;
		}
	}*/

	//2) EXPANSION STEP
	cout << "Expansion expanded!!" << endl;
	expandTree(device, state, detector, PRMgraph, ID, edgeLimit);

	cout << "Tree size: " << PRMgraph.size() << endl;

	edgeTimer.reset();
	cout << "Total seconds: " << edgeTimer.getTimeSec() << endl;
	//***QUERY PHASE****//

	cout << endl << endl << "Searching for a path:" << endl;
	//Q start=PRMgraph.find(PRMgraph.begin()->second.getID())->second.getConfig();
	//Q goal=PRMgraph.find(PRMgraph.begin()->second.getID()+10)->second.getConfig();
	//Q start = PRMgraph[0].getConfig();
	//Q goal = PRMgraph[9].getConfig();

	Q start;
	Q goal;

	int ID_start;
	int ID_goal;

	while (1){
		Timer globalTimer;
		globalTimer.reset();

		cout << "Tree generated :)" << endl <<
				"Insert the two node's ID to search the path for:" << endl
				<< "    Start:";
		cin >> ID_start;
		cout << "    Goal:";
		cin >> ID_goal;

		cout << endl << endl << "Searching!!";

		vector<Q> solution = calculatePathFromID(PRMgraph, ID_start, ID_goal, device);


		if (solution.empty()){
			cout << "The solution NOT found :(" << endl;
		}
		else{
			cout << "The solution found is:" << endl;
			for (auto i : solution){
				cout << i << ", ";
			}
			createFile(deviceName);
			addPosition(solution);
			cout << "File created!" << endl;

			cout << endl;
		}

		cout << "FINAL TIME, BABY: " << globalTimer.getTimeMs() << endl;
	}

	//cout<<"Size of the graph: "<<PRMgraph.size()<<endl;
	//cout << " --- Program ended ---" << endl;
	return 0;
}
