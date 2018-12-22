/***************************
  Navigator Class Implementation
  Created by Hongyi Fan
  Modified by Han Deng
  Aug. 07. 2018
  *************************/

#include <navigator.h>

using namespace std;
void navigator::threadMain(string filePath, string destinationPath, string lcmChannel, string voiceChannel, string hapticChannel, int mode)
{

	    lcm::LCM lcm;
		if(!lcm.good()) return;
		navigator* nav = &navigator::getInstance();
		nav->_map=new staticMap(filePath, destinationPath);
		nav->_currentNode = -1;
		nav->_currentOrientation = -1;
		nav->_destinationNode = -1;
		nav->_destinationOrientation = -1;
		nav->_positionInThePath = -1;
		nav->_initialized = 0;
		nav->_hapticChannel = hapticChannel;
		nav->_voiceChannel = voiceChannel;
		nav->_navigatorChannel = lcmChannel;
		nav->_mode = mode;
		lcm.subscribe(lcmChannel, &navigator::handleLCMRequests, nav);
		while(0 == lcm.handle()); // blocks

}

navigator& navigator::getInstance()
{
	static navigator nav;
	return nav;
}

navigator::navigator()
{
	_map=NULL;
	_initialized = 0;
}

navigator::navigator(string filePath, string destinationPath, string lcmChannel, string hapticChannel, string voiceChannel, int mode)
{
	_map = new staticMap(filePath, destinationPath);	
//	std::cout<<"StaticMap success"<<std::endl;
//	std::cout<<_map->getDestinationList(2)[0]<<std::endl;
	_currentNode = -1;
	_currentOrientation = -1;
	_destinationNode = -1;
	_destinationOrientation = -1;
	_positionInThePath = -1;
	_initialized = 0;
	_hapticChannel = hapticChannel;
	_voiceChannel = voiceChannel;
	_navigatorChannel = lcmChannel;
	_mode = mode;
}


navigator::~navigator()
{
	delete _map;
}

int minDistance(int dist[], bool sptSet[], int V)
{
	   // Initialize min value
	   int min = INT_MAX, min_index;
	     
	   for (int v = 0; v < V; v++)
		   if (sptSet[v] == false && dist[v] <= min)
			   min = dist[v], min_index = v;
		    
	   return min_index;
}

int navigator::printSolution(int dist[], int n)
{
	printf("Vertex   Distance from Source\n");
	for (int i = 0; i < _map->getNumNodes(); i++)
		printf("%d tt %d\n", i, dist[i]);
}

vector<int> navigator::Dijkstra(int** adjacentMatrix, int src)
{
	int V = _map->getNumNodes();

	vector<vector<vector<int> > > nodes(V);
	
	int dist[V];
	bool sptSet[V];

	for (int i = 0; i < V; i++)
	{
		dist[i] = INT_MAX, sptSet[i] = false;
	}

	sptSet[src] = 1;
	dist[src] = 0;

	for(int k=0; k<V; k++)
	{
		if(k!=src)
		{
			if(adjacentMatrix[src][k])
			{
				dist[k]=adjacentMatrix[src][k];
				vector<vector<int> > minPaths;
				minPaths.clear();
				vector<int> pathList;
				pathList.clear();
				pathList.push_back(src);
				pathList.push_back(k);
				minPaths.push_back(pathList);
				nodes[k] = minPaths;
			}
		}
	}
	//std::cout<<V<<std::endl;
	for(int i = 1; i<V; i++) //Find a nearest node.
	{
		int minDist = INT_MAX;
		int v = -1;
		for(int w = 0; w < V; w++)
		{
			
			if(!sptSet[w]&&minDist>dist[w])
			{
			//	std::cout<<i<<": "<<w<<" dist: "<<dist[w]<<std::endl;
				minDist = dist[w];
				v=w;
			}
		}
		if(v == -1)
		{
			std::cerr<<"sth was wrong"<<std::endl;
			exit(-1);
		}

		sptSet[v]=1;
		for(int w = 0; w < V; w++)
		{
			if(w==src)
			{
				continue;
			}
			if(!sptSet[w]) //If this node is not in the shortest path set
			{
				int newD=0;
				if(adjacentMatrix[v][w])
				{
					newD = minDist+adjacentMatrix[v][w];
				}
				else
				{
					newD = INT_MAX;
				}
				if(newD<dist[w])//if we can find a path from src to v to w shorter than from src to w
				{
					dist[w] = newD;
					vector<vector<int> > minPathsV = nodes[v];
					vector<int> pathList;
					nodes[w].clear(); //All previous path should be cleaned.
					for(int index = 0; index<minPathsV.size(); index++)
					{
						pathList = minPathsV[index];
						pathList.push_back(w);
						nodes[w].push_back(pathList); // re push back a new shorter path,
						//This path will be shortest until it will be chosen as the minDist;
					}
				}
				else if(newD == dist[w]) //find a same length path: two same path lead to the shortest path
				{
					vector<vector<int> > minPathsV = nodes[v];
					vector<int> pathList;
					for(int index = 0; index<minPathsV.size(); index++)
					{
						pathList = minPathsV[index]; // All paths from src to V plus v-w will be a shortest path from src to w
						pathList.push_back(w);
						nodes[w].push_back(pathList);
					}
				}
			}
		}
	}
	vector<vector<int> > specPath = nodes[_destinationNode];
	vector<int> numOfTurnEachPath;
	for(int i = 0; i < specPath.size(); i++)
	{
		vector<int> path = specPath[i];
		int turnNum=0;
		int _curOrientation = _currentOrientation;
		for(int j = 0; j<path.size(); j++)
		{
		 	pair<int, int> _curCoordinate = _map->getNodeCoordinateFromIndex(path[j]);
			pair<int, int> _nextCoordinate = _map->getNodeCoordinateFromIndex(path[j+1]);
			if(j==0)
			{
				switch(_curOrientation)
				{
					case 0:
						if(_nextCoordinate.first - _curCoordinate.first!=1)
							turnNum++;
						break;
					case 90:
						if(_curCoordinate.second - _nextCoordinate.second!=1)
							turnNum++;
						break;
					case 180:
						if(_curCoordinate.first - _nextCoordinate.first!=1)
							turnNum++;
						break;
					case 270:
						if(_nextCoordinate.second - _curCoordinate.second!=1)
							turnNum++;
						break;
					default:
						break;
				}
			}
			else
			{
				pair<int, int> _prevCoordinate = _map->getNodeCoordinateFromIndex(path[j-1]);
				int x_offset_1 = _curCoordinate.first - _prevCoordinate.first;
				int y_offset_1 = _curCoordinate.second - _prevCoordinate.second;
				int x_offset_2 = _nextCoordinate.first - _curCoordinate.first;
				int y_offset_2 = _nextCoordinate.second - _curCoordinate.second;
				if(x_offset_1 != x_offset_2 || y_offset_1 != y_offset_2)
				{
					turnNum++;
				}
			}
		}
		numOfTurnEachPath.push_back(turnNum);
	}
	int min = 0;
	int minTurn = INT_MAX;
	for(int i = 0; i<numOfTurnEachPath.size(); i++)
	{
		if(numOfTurnEachPath[i]<minTurn)
		{
			min = i;
			minTurn = numOfTurnEachPath[i];
		}
	}
	return specPath[min];								   
}

/* Only for test*/
void navigator::setDestination(int destinationNode, int currentOrientation, int currentNode, int destinationOrientation)
{
	this->_destinationNode = destinationNode;
	this->_currentOrientation = currentOrientation;
	this->_currentNode = currentNode;
	this->_destinationOrientation = destinationOrientation;
}

void navigator::getShortestPath()
{
	_path.clear();
	vector<int> pathTemp = Dijkstra(_map->getAdjacentMatrix(), _currentNode);


//	//Code For Test
	for (int i = 0; i < pathTemp.size(); i++)
	{
		std::cout << pathTemp[i] << " "; 
	}
	std::cout << endl;
	std::cout << endl;


	vector<int> path = pathTemp;
	if (path[0] != _currentNode)
	{
		cout << "STH is wrong" << endl;
	}
	_path.push_back(pair<int, int>(path[0], _currentOrientation));

	int oriNew = _currentOrientation;
	for (int i = 1; i < path.size(); i++)
	{
		if (oriNew == 0)
		{
			int xPre = _map->getNodeCoordinateFromIndex(path[i-1]).first;
			int yPre = _map->getNodeCoordinateFromIndex(path[i-1]).second;
			int xNext = _map->getNodeCoordinateFromIndex(path[i]).first;
			int yNext = _map->getNodeCoordinateFromIndex(path[i]).second;
			if (xNext - xPre == 1 && yNext - yPre == 0)
			{
				_path.push_back(pair<int,int>(path[i], 0));
				continue;
			}
			if (xNext - xPre == -1 && yNext - yPre == 0)
			{
				_path.push_back(pair<int,int>(path[i-1],180));
				_path.push_back(pair<int,int>(path[i],180));
				oriNew = 180;
				continue;
			}
			if (xNext - xPre == 0 && yNext - yPre == -1)
			{
				_path.push_back(pair<int,int>(path[i-1], 90));
				_path.push_back(pair<int,int>(path[i],90));
				oriNew = 90;
				continue;
		
			}
			if (xNext - xPre == 0 && yNext - yPre == 1)
			{
				_path.push_back(pair<int, int>(path[i-1], 270));
				_path.push_back(pair<int,int>(path[i],270));
				oriNew = 270;
				continue;
			}
		}
		else if (oriNew==90)
		{
			int xPre = _map->getNodeCoordinateFromIndex(path[i-1]).first;
			int yPre = _map->getNodeCoordinateFromIndex(path[i-1]).second;
			int xNext = _map->getNodeCoordinateFromIndex(path[i]).first;
			int yNext = _map->getNodeCoordinateFromIndex(path[i]).second;
			
			if (xNext - xPre == 0 && yNext - yPre == 1)		
			{
				_path.push_back(pair<int,int>(path[i-1],270));
				_path.push_back(pair<int,int>(path[i],270));
				oriNew = 270;
				continue;
			}
			if (xNext - xPre == 0 && yNext - yPre == -1)
			{
				_path.push_back(pair<int,int>(path[i], 90));
				continue;
			}
			if (xNext - xPre == 1 && yNext - yPre == 0)
			{
				_path.push_back(pair<int, int>(path[i-1],0));
				_path.push_back(pair<int,int>(path[i],0));
				oriNew = 0;
				continue;
			}
			if (xNext - xPre == -1 && yNext - yPre == 0)
			{
				_path.push_back(pair<int, int>(path[i-1],180));
				_path.push_back(pair<int,int>(path[i],180));
				oriNew = 180;
				continue;
			}
		}
		else if(oriNew==180)
		{
			int xPre = _map->getNodeCoordinateFromIndex(path[i-1]).first;
			int yPre = _map->getNodeCoordinateFromIndex(path[i-1]).second;
			int xNext = _map->getNodeCoordinateFromIndex(path[i]).first;
			int yNext = _map->getNodeCoordinateFromIndex(path[i]).second;
			
			if (xNext - xPre == -1 && yNext - yPre == 0)
			{
				_path.push_back(pair<int,int>(path[i], 180));
				continue;
			}
			if (xNext - xPre == 1 && yNext - yPre == 0)
			{
				_path.push_back(pair<int,int>(path[i-1],0));
				_path.push_back(pair<int,int>(path[i],0));
				oriNew = 0;
				continue;
			}
			if (xNext - xPre == 0 && yNext - yPre == 1)
			{
				_path.push_back(pair<int,int>(path[i-1],270));
				_path.push_back(pair<int,int>(path[i],270));
				oriNew = 270;
				continue;
			}
			if (xNext - xPre == 0 && yNext - yPre == -1)
			{
				_path.push_back(pair<int,int>(path[i-1], 90));
				_path.push_back(pair<int,int>(path[i],90));
				oriNew = 90;
				continue;
			}
		}
		else
		{
			int xPre = _map->getNodeCoordinateFromIndex(path[i-1]).first;
			int yPre = _map->getNodeCoordinateFromIndex(path[i-1]).second;
			int xNext = _map->getNodeCoordinateFromIndex(path[i]).first;
			int yNext = _map->getNodeCoordinateFromIndex(path[i]).second;
			if (xNext - xPre == 0 && yNext - yPre == -1)
			{
				_path.push_back(pair<int,int>(path[i-1],90));
				_path.push_back(pair<int,int>(path[i],90));
				oriNew = 90;
				continue;
				
			}
			if (xNext - xPre == 0 && yNext - yPre == 1)
			{
				_path.push_back(pair<int,int>(path[i], 270));
				continue;
			}
			if (xNext - xPre == -1 && yNext - yPre == 0)
			{
				_path.push_back(pair<int,int>(path[i-1],180));
				_path.push_back(pair<int,int>(path[i],180));
				oriNew = 180;
				continue;
			}
			if (xNext - xPre == 1 && yNext - yPre == 0)
			{
				_path.push_back(pair<int,int>(path[i-1],0));
				_path.push_back(pair<int,int>(path[i],0));
				oriNew = 0;
				continue;
			}
		}
	}
	if (_path[_path.size() - 1].second != _destinationOrientation)
	{
		_path.push_back(pair<int,int>(_path[_path.size() - 1].first,_destinationOrientation));
	}
}

string navigator::generateVoiceCommand(string hapticCommand)
{
	string voiceCommand;
	if (hapticCommand == "w")
	{
		voiceCommand = "straight";
	}

	if (hapticCommand == "X")
	{
		voiceCommand = "right";
	}

	if (hapticCommand == "Y")
	{
		voiceCommand = "left";
	}

	if (hapticCommand == "l")
	{
		voiceCommand = "around";
	}
	return voiceCommand;
}

void navigator::navigateInit()
{

	//Check if the map initalized
	if (!_map->getInitStatus())
	{
		cout << "Map is not initalized" << endl;
		exit(1);
	}
	//Check destination & current location is valid
	if (_currentNode == -1 || _destinationNode == -1)
	{
		cout << "Source and Destination is not initialized"  << endl;
		exit(1);
	}
	//Find The Path
	getShortestPath(); //重新规划路线_path,
	//std::cout<<"Shortest path is obtained"<<std::endl;
	_positionInThePath = 0; //把现在的位置当成0，重新开始。
	_initialized = 1;
	std::cout<<"Current position in the path is "<<_positionInThePath<<std::endl;
	cout << "====================Path====================" << endl;
	for (int i = 0; i < _path.size(); i++)
	{
			std::cout << _path[i].first << " " << _path[i].second << std::endl;	 
	}
	cout << "============================================" << endl;
	
	bool inDestinationList=0;
	if(std::find(_destinationList.cbegin(), _destinationList.cend(), _currentNode)!=_destinationList.cend())
	{
		inDestinationList=1;
	}

	if(inDestinationList == 1)
	{
		string nextVoiceCommand = "reached";
		if (_mode == 0)//chagned for test
		{
			std::string front_back[] = {"front", "behind"};
			std::string ori[] = {"left", "right"};
			LCMDataType::voiceCommand voiceMsg;
			voiceMsg.command = nextVoiceCommand;
			int curr_x = _currentCoordinate.first;
			int curr_y = _currentCoordinate.second;
			int dest_x = _destinationCoordinate.first;
			int dest_y = _destinationCoordinate.second;
			int curr_ori = _currentOrientation;
			int diff_x = curr_x - dest_x;
			int diff_y = curr_y - dest_y;

			if( curr_ori == 180)
			{
				if((diff_x == 0 && diff_y ==0)||(diff_x==1&&diff_y==0))
				{
					voiceMsg.message=front_back[0];
				} 
				else if(diff_x == 1 && diff_y == -1)
				{
					voiceMsg.message=ori[1]+" "+front_back[0];
				}
				else if(diff_x == 0 && diff_y == -1)
				{
					voiceMsg.message = ori[1];
				}
				else if(diff_x == -1 && diff_y == -1)
				{
					voiceMsg.message = ori[1]+" "+front_back[1];
				}
				else if(diff_x==-1 && diff_y==0)
				{
					voiceMsg.message = front_back[1];
				}
				else if(diff_x==-1 && diff_y==1)
				{
					voiceMsg.message = ori[0]+" "+front_back[1];
				}
				else if(diff_x == 0 && diff_y == 1)
				{
					voiceMsg.message = ori[0];
				}
				else if(diff_x == 1 && diff_y ==1)
				{
					voiceMsg.message = ori[0]+" "+front_back[0];
				}
			}
			else if(curr_ori==270)
			{
				if((diff_x == 0 && diff_y ==0)||(diff_x==1&&diff_y==0))
				{
					voiceMsg.message=ori[0];
				} 
				else if(diff_x == 1 && diff_y == -1)
				{
					voiceMsg.message=ori[0]+" "+front_back[0];
				}
				else if(diff_x == 0 && diff_y == -1)
				{
					voiceMsg.message = front_back[0];
				}
				else if(diff_x == -1 && diff_y == -1)
				{
					voiceMsg.message = ori[1]+" "+front_back[0];
				}
				else if(diff_x==-1 && diff_y==0)
				{
					voiceMsg.message = ori[1];
				}
				else if(diff_x==-1 && diff_y==1)
				{
					voiceMsg.message = ori[1]+" "+front_back[1];
				}
				else if(diff_x == 0 && diff_y == 1)
				{
					voiceMsg.message = front_back[1];
				}
				else if(diff_x == 1 && diff_y ==1)
				{
					voiceMsg.message = ori[0]+" "+front_back[1];
				}
			}
			else if(curr_ori==0)
			{
				if((diff_x == 0 && diff_y ==0)||(diff_x==1&&diff_y==0))
				{
					voiceMsg.message=front_back[1];
				} 
				else if(diff_x == 1 && diff_y == -1)
				{
					voiceMsg.message=ori[0]+" "+front_back[1];
				}
				else if(diff_x == 0 && diff_y == -1)
				{
					voiceMsg.message = ori[0];
				}
				else if(diff_x == -1 && diff_y == -1)
				{
					voiceMsg.message = ori[0]+" "+front_back[0];
				}
				else if(diff_x==-1 && diff_y==0)
				{
					voiceMsg.message = front_back[0];
				}
				else if(diff_x==-1 && diff_y==1)
				{
					voiceMsg.message = ori[1]+" "+front_back[0];
				}
				else if(diff_x == 0 && diff_y == 1)
				{
					voiceMsg.message = ori[1];
				}
				else if(diff_x == 1 && diff_y ==1)
				{
					voiceMsg.message = ori[1]+" "+front_back[1];
				}
			}
			else if(curr_ori==90)
			{
				if((diff_x == 0 && diff_y ==0)||(diff_x==1&&diff_y==0))
				{
					voiceMsg.message=ori[1];
				} 
				else if(diff_x == 1 && diff_y == -1)
				{
					voiceMsg.message=ori[1]+" "+front_back[1];
				}
				else if(diff_x == 0 && diff_y == -1)
				{
					voiceMsg.message = front_back[1];
				}
				else if(diff_x == -1 && diff_y == -1)
				{
					voiceMsg.message = ori[0]+" "+front_back[1];
				}
				else if(diff_x==-1 && diff_y==0)
				{
					voiceMsg.message =ori[0];
				}
				else if(diff_x==-1 && diff_y==1)
				{
					voiceMsg.message = ori[0]+" "+front_back[0];
				}
				else if(diff_x == 0 && diff_y == 1)
				{
					voiceMsg.message = front_back[0];
				}
				else if(diff_x == 1 && diff_y ==1)
				{
					voiceMsg.message = ori[1]+" "+front_back[0];
				}
			}
			_lcmInstance.publish(_voiceChannel, &voiceMsg);
		}
	}
	else 
	{
	
		string nextHapticCommand = generateHapticCommand();	
		string nextVoiceCommand = generateVoiceCommand(nextHapticCommand);

		if (_mode == 0)//changed for test
		{
			//Send the Signal to audio synthesizer	
			LCMDataType::voiceCommand voiceMsg;
			voiceMsg.command = nextVoiceCommand;
			_lcmInstance.publish(_voiceChannel, &voiceMsg);

			//Send the Signal to Haptic Channel;
			LCMDataType::haptic_belt_command_t hapticMsg;
			hapticMsg.command = nextHapticCommand;
		//	std::cout<<"Next haptic command is "<<hapticMsg.command<<std::endl;
			_lcmInstance.publish(_hapticChannel, &hapticMsg);
		}
		else
		{
			cout <<"Next Haptic Command: " << nextHapticCommand << endl;
			cout << "Next Voice Command: " << nextVoiceCommand << endl;
		}

		_positionInThePath++;
	}
}

void navigator::navigate()
{
	//Check the initialization status
	std::cout<<"Current position in the path is "<<_positionInThePath<<std::endl;
	if (!_initialized)
	{
		std::cout << "The navigator is not inialized properly" << std::endl;
		exit(1);
	}
	//Check the initialization of maps
	if (!_map->getInitStatus())
	{
		cout << "Map is not initialized" << endl;
	}
	
	cout << "====================Path====================" << endl;
	for (int i = 0; i < _path.size(); i++)
	{
			std::cout << _path[i].first << " " << _path[i].second << std::endl;	
	}
	cout << "============================================" << endl;
	

//	if (_currentNode == _destinationNode && _currentOrientation == _destinationOrientation)
//	{
//		string nextVoiceCommand = "reached";
//		if (_mode == 0) //changed for test
//		{
//			LCMDataType::voiceCommand voiceMsg;
//			voiceMsg.command = nextVoiceCommand;
//			_lcmInstance.publish(_voiceChannel, &voiceMsg);
//	//		cout << "Next Command:" << nextVoiceCommand << endl;
//		}
//		else
//		{
//			cout << "Next Command:" << nextVoiceCommand << endl;
//		}
//	}
	
	bool inDestinationList=0;
	if(std::find(_destinationList.cbegin(), _destinationList.cend(), _currentNode)!=_destinationList.cend())
	{
		inDestinationList=1;
	}

	if(inDestinationList == 1)
	{
		string nextVoiceCommand = "reached";
		if (_mode == 0)//chagned for test
		{
			std::string front_back[] = {"front", "behind"};
			std::string ori[] = {"left", "right"};
			LCMDataType::voiceCommand voiceMsg;
			voiceMsg.command = nextVoiceCommand;
			int curr_x = _currentCoordinate.first;
			int curr_y = _currentCoordinate.second;
			int dest_x = _destinationCoordinate.first;
			int dest_y = _destinationCoordinate.second;
			int curr_ori = _currentOrientation;
			int diff_x = curr_x - dest_x;
			int diff_y = curr_y - dest_y;

			if( curr_ori == 180)
			{
				if((diff_x == 0 && diff_y ==0)||(diff_x==1&&diff_y==0))
				{
					voiceMsg.message=front_back[0];
				} 
				else if(diff_x == 1 && diff_y == -1)
				{
					voiceMsg.message=ori[1]+" "+front_back[0];
				}
				else if(diff_x == 0 && diff_y == -1)
				{
					voiceMsg.message = ori[1];
				}
				else if(diff_x == -1 && diff_y == -1)
				{
					voiceMsg.message = ori[1]+" "+front_back[1];
				}
				else if(diff_x==-1 && diff_y==0)
				{
					voiceMsg.message = front_back[1];
				}
				else if(diff_x==-1 && diff_y==1)
				{
					voiceMsg.message = ori[0]+" "+front_back[1];
				}
				else if(diff_x == 0 && diff_y == 1)
				{
					voiceMsg.message = ori[0];
				}
				else if(diff_x == 1 && diff_y ==1)
				{
					voiceMsg.message = ori[0]+" "+front_back[0];
				}
			}
			else if(curr_ori==270)
			{
				if((diff_x == 0 && diff_y ==0)||(diff_x==1&&diff_y==0))
				{
					voiceMsg.message=ori[0];
				} 
				else if(diff_x == 1 && diff_y == -1)
				{
					voiceMsg.message=ori[0]+" "+front_back[0];
				}
				else if(diff_x == 0 && diff_y == -1)
				{
					voiceMsg.message = front_back[0];
				}
				else if(diff_x == -1 && diff_y == -1)
				{
					voiceMsg.message = ori[1]+" "+front_back[0];
				}
				else if(diff_x==-1 && diff_y==0)
				{
					voiceMsg.message = ori[1];
				}
				else if(diff_x==-1 && diff_y==1)
				{
					voiceMsg.message = ori[1]+" "+front_back[1];
				}
				else if(diff_x == 0 && diff_y == 1)
				{
					voiceMsg.message = front_back[1];
				}
				else if(diff_x == 1 && diff_y ==1)
				{
					voiceMsg.message = ori[0]+" "+front_back[1];
				}
			}
			else if(curr_ori==0)
			{
				if((diff_x == 0 && diff_y ==0)||(diff_x==1&&diff_y==0))
				{
					voiceMsg.message=front_back[1];
				} 
				else if(diff_x == 1 && diff_y == -1)
				{
					voiceMsg.message=ori[0]+" "+front_back[1];
				}
				else if(diff_x == 0 && diff_y == -1)
				{
					voiceMsg.message = ori[0];
				}
				else if(diff_x == -1 && diff_y == -1)
				{
					voiceMsg.message = ori[0]+" "+front_back[0];
				}
				else if(diff_x==-1 && diff_y==0)
				{
					voiceMsg.message = front_back[0];
				}
				else if(diff_x==-1 && diff_y==1)
				{
					voiceMsg.message = ori[1]+" "+front_back[0];
				}
				else if(diff_x == 0 && diff_y == 1)
				{
					voiceMsg.message = ori[1];
				}
				else if(diff_x == 1 && diff_y ==1)
				{
					voiceMsg.message = ori[1]+" "+front_back[1];
				}
			}
			else if(curr_ori==90)
			{
				if((diff_x == 0 && diff_y ==0)||(diff_x==1&&diff_y==0))
				{
					voiceMsg.message=ori[1];
				} 
				else if(diff_x == 1 && diff_y == -1)
				{
					voiceMsg.message=ori[1]+" "+front_back[1];
				}
				else if(diff_x == 0 && diff_y == -1)
				{
					voiceMsg.message = front_back[1];
				}
				else if(diff_x == -1 && diff_y == -1)
				{
					voiceMsg.message = ori[0]+" "+front_back[1];
				}
				else if(diff_x==-1 && diff_y==0)
				{
					voiceMsg.message =ori[0];
				}
				else if(diff_x==-1 && diff_y==1)
				{
					voiceMsg.message = ori[0]+" "+front_back[0];
				}
				else if(diff_x == 0 && diff_y == 1)
				{
					voiceMsg.message = front_back[0];
				}
				else if(diff_x == 1 && diff_y ==1)
				{
					voiceMsg.message = ori[1]+" "+front_back[0];
				}
			}
			_lcmInstance.publish(_voiceChannel, &voiceMsg);
		}
	}
	else
	{	
//		std::cout<<"Don't reach the destination"<<std::endl;
		//Check the current Position is in the _path list or not. 
		//First check the current posiition is the current step or not
		pair<int,int> predCurrent = _path[_positionInThePath]; 
		//这个path是为了我的这个destination和我现在所在的这个position专门生成的一条完整的path
		if (predCurrent.first == _currentNode && predCurrent.second == _currentOrientation)
		{
			//Generate all the commands;
			string nextHapticCommand = generateHapticCommand();	
//			cout << "The next haptic command is:" << nextHapticCommand << endl;
			string nextVoiceCommand = generateVoiceCommand(nextHapticCommand);
			cout << "The next voice command" << nextVoiceCommand << endl;

			if (_mode == 0)//changed for test
			{
				//Send the Signal to audio synthesizer	
				LCMDataType::voiceCommand voiceMsg;
				voiceMsg.command = nextVoiceCommand;
				_lcmInstance.publish(_voiceChannel, &voiceMsg);
				
				//Send the Signal to Haptic Channel;
				LCMDataType::haptic_belt_command_t hapticMsg;
				hapticMsg.command = nextHapticCommand;
//				std::cout<<"Next haptic command is "<<hapticMsg.command<<std::endl;
				_lcmInstance.publish(_hapticChannel, &hapticMsg);
			}
			else 
			{
				cout <<"Next Haptic Command: " << nextHapticCommand << endl;
				cout << "Next Voice Command: " << nextVoiceCommand << endl;
			}
	
			_positionInThePath++;
		}
		else
		{
			vector<pair<int,int> >::iterator it = find(_path.begin(), _path.end(), pair<int,int>(_currentNode, _currentOrientation));
			if (it != _path.end())
			{
				//If the current location is in the _path list, then continue navigate along this path. 
				int index = std::distance(_path.begin(), it);
				_positionInThePath = index;
				//Generate all the commands;
				string nextHapticCommand = generateHapticCommand();	
				cout<<"Still in the path, but not as predicted, current position in the path: "<<_positionInThePath<<endl;
				//cout << "The next command is:" << nextHapticCommand << endl;
				string nextVoiceCommand = generateVoiceCommand(nextHapticCommand);
				//cout << "The next voice command" << nextVoiceCommand << endl;
		
			
				if (_mode == 0) //changed for test
				{
					//Send the Signal to audio synthesizer	
					LCMDataType::voiceCommand voiceMsg;
					voiceMsg.command = nextVoiceCommand;
					_lcmInstance.publish(_voiceChannel, &voiceMsg);
			
					//Send the Signal to Haptic Channel;
					LCMDataType::haptic_belt_command_t hapticMsg;
					hapticMsg.command = nextHapticCommand;
					_lcmInstance.publish(_hapticChannel, &hapticMsg);
				}
				else
				{
					cout <<"Next Haptic Command: " << nextHapticCommand << endl;
					cout << "Next Voice Command: " << nextVoiceCommand << endl;
				}

			}
			else
			{
				//If the current location is not in the _path list, then relocate and then....em....regenerate the path (drifting) //说明偏移了？
				cout<<"Not in the path, will re-calculate the shortest path "<<std::endl;
				navigateInit();								
			}
		}
	}	
}

string navigator::generateHapticCommand()
{
	if(_positionInThePath == -1)
	{
		cout << "The navigator is not initialize" << endl;
		exit(1);
	}
	
	if (_path[_positionInThePath].first != _currentNode)
	{
		cout << "Didn't Match The Path" << endl;
		exit(1);
	}
	
	int xCurrent = _currentCoordinate.first;
	int yCurrent = _currentCoordinate.second; 
	int oCurrent = _currentOrientation;	
	pair<int,int> nextStep = _path[_positionInThePath + 1];

	int xNext = _map->getNodeCoordinateFromIndex(nextStep.first).first;
	int yNext = _map->getNodeCoordinateFromIndex(nextStep.first).second;
	int oNext = nextStep.second; //直接得到下一步朝哪个方向走。  
//	std::cout<<"Next coordinate: "<<xNext<<" "<<yNext<<" Next orientation: "<<nextStep.second<<std::endl;

	if (oCurrent == 0)
	{
		return generateCommand0(xCurrent, yCurrent, xNext, yNext, oNext);
	}
	else if(oCurrent == 90)
	{
		return generateCommand90(xCurrent, yCurrent, xNext, yNext, oNext);
	}
	else if(oCurrent == 180)
	{
		return generateCommand180(xCurrent, yCurrent, xNext, yNext, oNext);
	}
	else if(oCurrent == 270)
	{
		return generateCommand270(xCurrent, yCurrent, xNext, yNext, oNext);

	}
	else
	{
		cout << "Something is wrong, no this orientation" << endl;
		exit(1);
	}
}

string navigator::generateCommand0(int xCurrent, int yCurrent, int xNext, int yNext, int oNext)
{
	if (abs(xCurrent - xNext) > 1 || abs(yCurrent - yNext) > 1)
	{
		cout << "Criteria1:"  << (xCurrent - xNext) << endl;
		cout << "Criteria2:" << (yCurrent - yNext) << endl;
		cout << "The _path is not generated well" << endl;
		exit(1);		
	}
	else
	{
		string command;
		if (((xNext - xCurrent) == 1 && (yNext - yCurrent) == 0 && (oNext == 0)))
		{
			command = "w";
			return command;
		}
		else if(((xNext - xCurrent)  == 0 && (yNext - yCurrent) == 0 && (oNext == 270)))
		{
			command = "Y";
			return command;
		}
		else if(((xNext - xCurrent)  == 0 && (yNext - yCurrent) == 0 && (oNext == 90)))
		{
			command = "X";	
			return command;
		}
		else if(((xNext - xCurrent)  == 0 && (yNext - yCurrent) == 0 && (oNext == 180)))
		{
			command = "l";	
			return command; 
		}
		else
		{
			cout<<"From command 0 haptic belt command generator"<<std::endl;
			cout << "The _path is not generated well" << endl;
			exit(1);	
		}
	}
}

string navigator::generateCommand90(int xCurrent, int yCurrent, int xNext, int yNext, int oNext)
{
	if (abs(xCurrent - xNext) > 1 || abs(yCurrent - yNext) > 1)
	{
		cout << "Criteria1:"  << (xCurrent - xNext) << endl;
		cout << "Criteria2:" << (yCurrent - yNext) << endl;
		cout<<"From command 90"<<std::endl;
		cout << "The _path is not generated well" << endl;
		exit(1);		
	}
	else
	{
		string command;
		if (((xNext - xCurrent) == 0 && (yNext - yCurrent) == -1 && (oNext == 90)))
		{
			command = "w";
			return command;
		}
		else if(((xNext - xCurrent)  == 0 && (yNext - yCurrent) == 0 && (oNext == 0)))
		{
			command = "Y";
			return command;
		}
		else if(((xNext - xCurrent)  == 0 && (yNext - yCurrent) == 0 && (oNext == 180)))
		{
			command = "X";	
			return command; 
		}
		else if(((xNext - xCurrent)  == 0 && (yNext - yCurrent) == 0 && (oNext == 270)))
		{
			command = "l";	
			return command; 
		}
		else
		{
			cout<<"From command 90 haptic belt command generator"<<std::endl;
			cout << "The _path is not generated well" << endl;
			exit(1);	
		}
	}
}

string navigator::generateCommand180(int xCurrent, int yCurrent, int xNext, int yNext, int oNext)
{
	if (abs(xCurrent - xNext) > 1 || abs(yCurrent - yNext) > 1)
	{
		cout << "Criteria1:"  << (xCurrent - xNext) << endl;
		cout << "Criteria2:" << (yCurrent - yNext) << endl;
		cout<<"Generated by command 180"<<std::endl;
		cout << "The _path is not generated well" << endl;
		exit(1);		
	}
	else
	{
		string command;
		if ((xNext - xCurrent) == -1 && (yNext - yCurrent) == 0 && (oNext == 180))
		{
			command = "w";
			return command;
		}
		else if(((xNext - xCurrent)  == 0 && (yNext - yCurrent) == 0 && (oNext == 90)))
		{
			command = "Y";
			return command;
		}
		else if(((xNext - xCurrent)  == 0 && (yNext - yCurrent) == 0 && (oNext == 270)))
		{
			command = "X";	
			return command; 
		}
		else if(((xNext - xCurrent)  == 0 && (yNext - yCurrent) == 0 && (oNext == 0)))
		{
			command = "l";	
			return command; 
		}
		else
		{
			cout<<"From command 180 haptic belt command generator"<<std::endl;
			cout << "The _path is not generated well" << endl;
			exit(1);	
		}
	}
}

string navigator::generateCommand270(int xCurrent, int yCurrent, int xNext, int yNext, int oNext)
{
	if (abs(xCurrent - xNext) > 1 || abs(yCurrent - yNext) > 1)
	{
		cout << "Criteria1:"  << (xCurrent - xNext) << endl;
		cout << "Criteria2:" << (yCurrent - yNext) << endl;
		cout<<"Generated by command 270"<<std::endl;
		cout << "The _path is not generated well" << endl;
		exit(1);		
	}
	else
	{
		string command;
		if ((xNext - xCurrent) == 0 && (yNext - yCurrent) == 1 && (oNext == 270))
		{
			 command = "w";
			return command;
		}
		else if(((xNext - xCurrent)  == 0 && (yNext - yCurrent) == 0 && (oNext == 180)))
		{
			command = "Y";
			return command;
		}
		else if(((xNext - xCurrent)  == 0 && (yNext - yCurrent) == 0 && (oNext == 0)))
		{
			command = "X";	
			return command; 
		}
		else if(((xNext - xCurrent)  == 0 && (yNext - yCurrent) == 0 && (oNext == 90)))
		{
			command = "l";	
			return command; 
		}
		else
		{
			cout<<"From command 270"<<std::endl;
			cout << "The _path is not generated well" << endl;
			exit(1);	
		}
	}
}
void navigator::handleLCMRequests(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const LCMDataType::locationBundle* msg)
{
	int msgType;
	int imageIndex1;
	int imageIndex2;
	int destination;
	int destinationOrientation;
	int currentNode1;
	int currentNode2;
	int orientation1;
	int orientation2;
	int destinationNode;
	msgType = msg->msgType; 
	if(msgType == 1)
	{
		_initialized=0;
		return;
	}
	if (_mode == 0) 
	{
		msgType = msg->msgType; 
		
		if (msgType == 0)
		{	
			imageIndex1 = msg->imageIndex1;
			imageIndex2 = msg->imageIndex2;
			destination = msg->destination;
			_destinationList = _map->getDestinationList(destination);			
			destinationOrientation = _map->getDestinationOrientation(destination);
			currentNode1 = _map->getImageCoordinateViaIndex(imageIndex1);
			currentNode2 = _map->getImageCoordinateViaIndex(imageIndex2);
			orientation1 = _map->getImageOrientation(imageIndex1);
			orientation2 = _map->getImageOrientation(imageIndex2);
			destinationNode = _destinationList[0];
		}
		/*
		else
		{
			if (msgType == 1) 
			{
				imageIndex1 = msg->imageIndex1;
				imageIndex2 = msg->imageIndex2;
				currentNode1 = _map->getImageCoordinateViaIndex(imageIndex1);
				currentNode2 = _map->getImageCoordinateViaIndex(imageIndex2);
				orientation1 = _map->getImageOrientation(imageIndex1);
				orientation2 = _map->getImageOrientation(imageIndex2);
			}
			else
			{
				cout << "The message is not initialized well" << endl;
				exit(1);
			}
		}*/
	}
	/*
	else
	{
		msgType = msg->msgType;
		if (msgType != 4 && msgType!=5)
		{
			cout << "The message is not initialized well" << endl;
			exit(1);
		}
		int xPos = msg->xCurrent;
		int yPos = msg->yCurrent;
		std::cout<<"XCurrent from the message: "<<xPos<<" YCurrent from the message: "<<yPos<<std::endl;
		pair<int,int> currentCoordinate = pair<int, int>(xPos, yPos);
		currentNode1 = _map->getNodeIndexFromCoordinate(currentCoordinate);
		currentNode2 = _map->getNodeIndexFromCoordinate(currentCoordinate);
	
		orientation1 = msg->oCurrent;
		orientation2 = msg->oCurrent;
		if(msgType==4)
		{
			int destinationIndex = msg->destination;
			destinationOrientation = this->_map->getDestinationOrientation(destinationIndex);
			destinationNode = this->_map->getDestinationList(destinationIndex)[0];
		}
		std::cout<<"Current Node Id obtained is: "<<currentNode1<<std::endl;
	}*/
	
	bool flag = 0;
	bool flag_distance=0;
	bool flag_orientation=0;
	
	pair<int, int> currentCoordinate1 = _map->getNodeCoordinateFromIndex(currentNode1);
	pair<int, int> currentCoordinate2 = _map->getNodeCoordinateFromIndex(currentNode2);
	int distance = sqrt(pow((currentCoordinate1.first - currentCoordinate2.first),2) + pow((currentCoordinate1.second - currentCoordinate2.second),2));
	if (distance > 3) flag_distance = 0;
	else flag_distance = 1;
	if (orientation1 == orientation2) flag_orientation = 1;
	else flag_orientation = 0;
	if(flag_distance&&flag_orientation) flag=1;
	else flag=0;
	if (flag) 
	{
		if (msgType == 0) 
		{	
			_currentNode = currentNode1;
			_currentOrientation = orientation1;
			_destinationNode = destinationNode; 

			_destinationOrientation = destinationOrientation;
			_currentCoordinate = _map->getNodeCoordinateFromIndex(_currentNode);

			_destinationCoordinate = _map->getNodeCoordinateFromIndex(_destinationNode);

			cout << "current node:" << _currentNode << endl;
			cout << "current Orientation:" << _currentOrientation << endl;
	//		cout << "current Coordinate:" << _currentCoordinate.first << " " << _currentCoordinate.second << endl;

	//		cout << "Destination node:" << _destinationNode << endl;
	//		cout << "Destination Orientation:" << _destinationOrientation << endl;
	//		cout << "Destination Coordinate:" << _destinationCoordinate.first << " " << _destinationCoordinate.second << endl;

			if (_initialized == 1) 
			{
				navigate();
			}
			else
			{
				navigateInit(); 
			}
		}
		else
		{
			if (msgType == 1)
			{
				_currentNode = currentNode1;
				_currentOrientation = orientation1;
				_currentCoordinate = _map->getNodeCoordinateFromIndex(_currentNode);
				cout << "current node:" << _currentNode << endl;
				cout << "current Orientation:" << _currentOrientation << endl;
				cout << "current Coordinate:" << _currentCoordinate.first << " " << _currentCoordinate.second << endl;

				cout << "Destination node:" << _destinationNode << endl; //直接用的是刚刚初始化的_destinationNode.
				cout << "Destination Orientation:" << _destinationOrientation << endl;
				cout << "Destination Coordinate:" << _destinationCoordinate.first << " " << _destinationCoordinate.second << endl;

				if (_initialized == 1)
				{
					navigate();
				}
				else
				{
					navigateInit();
				}
			}
			else if(msgType == 4)
			{
				_currentNode = currentNode1;
				_currentOrientation = orientation1;
				_currentCoordinate = _map->getNodeCoordinateFromIndex(_currentNode);
				_destinationNode = destinationNode;
				_destinationOrientation = destinationOrientation;
				_destinationCoordinate = _map->getNodeCoordinateFromIndex(_destinationNode);
				cout << "current node:" << _currentNode << endl;
				cout << "current Orientation:" << _currentOrientation << endl;
				cout << "current Coordinate:" << _currentCoordinate.first << " " << _currentCoordinate.second << endl;

				cout << "Destination node:" << _destinationNode << endl;
				cout << "Destination Orientation:" << _destinationOrientation << endl;
				cout << "Destination Coordinate:" << _destinationCoordinate.first << " " << _destinationCoordinate.second << endl;

				if (_initialized == 1)
				{
					navigate();
				}
				else
				{
					navigateInit();
				}
			}
			else if(msgType == 5)
			{
				_currentNode = currentNode1;
				_currentOrientation = orientation1;
				_currentCoordinate = _map->getNodeCoordinateFromIndex(_currentNode);
			//	_destinationNode = destinationNode;
			//_destinationOrientation = destinationOrientation;
			//	_destinationCoordinate = _map->getNodeCoordinateFromIndex(_destinationNode);
				cout << "current node:" << _currentNode << endl;
				cout << "current Orientation:" << _currentOrientation << endl;
				cout << "current Coordinate:" << _currentCoordinate.first << " " << _currentCoordinate.second << endl;

				cout << "Destination node:" << _destinationNode << endl;
				cout << "Destination Orientation:" << _destinationOrientation << endl;
				cout << "Destination Coordinate:" << _destinationCoordinate.first << " " << _destinationCoordinate.second << endl;

				if (_initialized == 1)
				{
					navigate();
				}
				else
				{
					navigateInit();
				}
			}
		}
	}
	else
	{
			LCMDataType::voiceCommand voiceMsg;
			voiceMsg.command = "relocate";
			_lcmInstance.publish(_voiceChannel, &voiceMsg);
	}	
}
