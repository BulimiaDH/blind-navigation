/*****************************************
  Static Map Class Implementation
  Created by Hongyi Fan 
  Dec. 1 2017
  ****************************************/

#include "staticMap.h"

using namespace std;



void readMapFile(string filePath, vector<float>& xPos, vector<float>& yPos, vector<int>& ori)
{
	ifstream inStream;
	inStream.open(filePath);

	if(!inStream)
	{
		std::cout << "Unable to Open the File" << std::endl;
		exit(1);
	}
	string x;
	while (inStream >> x)
	{
		stringstream ss;
		ss << x;
		float xPosTemp;
		ss >> xPosTemp;
		xPos.push_back(xPosTemp);
		inStream >> x;
		stringstream mm;
		mm << x;
		
		float yPosTemp;
		mm >> yPosTemp;
		yPos.push_back(yPosTemp);

		inStream >> x;
		stringstream oo;
		oo << x;
		int oriTemp;
		oo >> oriTemp;
		ori.push_back(oriTemp);
	}
}

void readDestinationFile(string filePath, vector<pair<float,float> > &Coords, vector<int> &destinationList, vector<int> &destinationOrientation)
{
	ifstream inStream;
	inStream.open(filePath);
	
	if (!inStream)
	{
		std::cout << "Unable to open the destination file" << std::endl;
		exit(1);
	}

	string x;
	while(inStream >> x)
	{
		stringstream ss;
		ss << x;
		float xPosTemp;
		ss >> xPosTemp;

		stringstream yy;
		inStream >> x;
		yy << x;
		float yPosTemp;
		yy >> yPosTemp;

		stringstream dd;
		int destinationTemp;
		inStream >> x;
		dd << x;
		dd >> destinationTemp;

		stringstream oo;
		int orientationTemp;
		inStream >> x;
		oo << x;
		oo >> orientationTemp;

		destinationList.push_back(destinationTemp);
		Coords.push_back(std::pair<float,float> (xPosTemp, yPosTemp));
		destinationOrientation.push_back(orientationTemp);
	}
}

vector<int> staticMap::getDestinationList(int destinationIndex)
{
	return _destinationList[destinationIndex];	
}

int staticMap::getImageOrientation(int imageIndex)
{
	return _imageOrientationIndex[imageIndex];
}


staticMap::staticMap()
{
	_AdjacentMatrix = NULL;
	_initialized = 0;
}



staticMap::staticMap(string filePath, string destinationPath)
{
//	std::cout << "Static Map, destinationPat: " <<destinationPath<<std::endl;
	vector<float> xPos;
	vector<float> yPos;
	vector<int> ori;
	std::cout<<filePath<<" "<<destinationPath<<std::endl;
	readMapFile(filePath, xPos, yPos, ori);
	_xPos = xPos;
	_yPos = yPos;
	_ori = ori;
	

	for (int i = 0; i < _ori.size() ;i++)
	{
		_imageOrientationIndex.insert(pair<int, int>(i, _ori[i]));
	}
	//Get width and height
	auto itWidthMax = max_element(std::begin(_xPos),std::end(_xPos));
	auto itHeightMax = max_element(std::begin(_yPos), std::end(_yPos));
	auto itWidthMin = min_element(std::begin(_xPos),std::end(_xPos));
	auto itHeightMin = min_element(std::begin(_yPos), std::end(_yPos));
	float maxWidth = round(*itWidthMax);
	float maxHeight = round(*itHeightMax);
	_width = maxWidth;
	_height = maxHeight;
	//cout << "width: " << _width << endl;
	//cout << "height: " << _height << endl;

	//Initialize validMap	
	for (int i = 0; i < _width; i++)
	{
		for (int j = 0; j < _height; j++)
		{
			pair<int,int> gridElements(i,j);
			bool validation  = 0;
			_validMap.insert(pair< pair<int, int> , bool>(gridElements, validation));
		}
	}
//	std::cout<<"size of the valid map is "<<_validMap.size()<<std::endl<<_width<<_height<<std::endl;

	_numNodes = 0;
	vector<int> xNode;
	vector<int> yNode;
	
	for (int i = 0; i < _xPos.size(); i++)
	{
		int xTemp = round(_xPos[i]);
		int yTemp = round(_yPos[i]);
		//cout <<"grid X"<< xTemp << endl;
		//cout <<"grid Y"<< yTemp << endl;
		if (_validMap[pair<int, int>(xTemp,yTemp)] == 0)
		{
			_validMap[pair<int, int>(xTemp, yTemp)] = 1;
			_numNodes++; //There is one node only when they have coordinates.
			xNode.push_back(xTemp);
			yNode.push_back(yTemp);
		}	
	}
	_xNode = xNode;
	_yNode = yNode;
//	for (int i = 0; i < _xNode.size(); i++)
//	{
//		//std::cout <<"x:" << _xNode[i] << std::endl;
//		std::cout <<"y:" << _yNode[i] << std::endl;
//	}

	//Node Index to Coordinates
//	std::cout<<"Number of nodes is "<<_numNodes<<std::endl;
	for (int i = 0; i < _numNodes; i++)
	{

		_nodeCoordinatesIndex.insert(pair<int, pair<int, int> >(i, pair<int, int>(_xNode[i], _yNode[i])));
	}
//	// Node to Coordinates
//	int counter = 0;
//	for (int i = 0; i < _width; i++)
//	{
//		for (int j = 0; j < _height; j++)
//		{
//			if(_validMap[pair<int,int>(i,j)] == 0)
//			{
//				continue;
//			}
//			else
//			{
//				_nodeCoordinatesIndex.insert(pair<int, pair<int,int> >(counter, pair<int,int>(i, j)));
//				counter++;
//			}
//		}
//	}

//	for (int i = 0; i < _numNodes; i++)
//	{
//		//std::cout << "index: " << i << std::endl;
//		std::cout << _nodeCoordinatesIndex[i].first << " " << _nodeCoordinatesIndex[i].second << std::endl;
//	}

	//Node Coordinates to Index	
	for (int i = 0; i < _numNodes; i++)
	{
		_nodeCoordinatesIndex.insert(pair<int, pair<int,int> >(i, pair<int,int>(_xNode[i], _yNode[i])));
	}


	 //Initialize CoordNodeIndex;	
	for (int i = 0; i < _xPos.size(); i++)
	{
		pair<int,int> gridCoord = _nodeCoordinatesIndex[i];
		_coordNodeIndex.insert(pair<pair<int,int>, int>(gridCoord,i));
	}
	
	//Initialize imageNodeIndex
	for (int i = 0; i < _xPos.size(); i++)
	{
		int xTemp = round(_xPos[i]);
		int yTemp = round(_yPos[i]);
	
		int node = _coordNodeIndex[pair<int,int>(xTemp, yTemp)];

		_imageNodeIndex.insert(pair<int,int>(i, node));
	}

	//Initialize _AdjacentMatrix;
	_AdjacentMatrix = new int*[_numNodes];
	for (int i = 0; i < _numNodes; i++)
	{
		_AdjacentMatrix[i] = new int[_numNodes];
	}


	for (int i = 0; i < _numNodes; i++)
	{
		for (int j = 0; j < _numNodes; j++)
		{
			if (i == j)
			{
				_AdjacentMatrix[i][j] = 0;
				continue;
			}
			std::pair<int,int> row  = _nodeCoordinatesIndex[i];
			std::pair<int,int> column = _nodeCoordinatesIndex[j];
		
			//cout << row.first << " " << row.second << endl;
			//cout << column.first << " " << column.second << endl;
			
			bool criteria1 = (abs(row.first - column.first) == 1 && abs(row.second - column.second) == 0);
			bool criteria2 = (abs(row.first - column.first) == 0 && abs(row.second - column.second) == 1);	
			bool fullCriteria = (criteria1 | criteria2);

			//cout << "criteria1: " << criteria1 << endl;
			//cout << "criteria2: " << criteria2 << endl;
			//cout << "fellCriteria: " << fullCriteria << endl;

			if (fullCriteria)
			{
				_AdjacentMatrix[i][j] = 1;
			}
			else
			{
				_AdjacentMatrix[i][j] = 0;
			}
		}
	}
	//Initialize _destinationList;
	vector<pair<float, float> > Coords;
	vector<int> List;
	vector<int> destOri;
	readDestinationFile(destinationPath, Coords, List, destOri);
//	std::cout << List.size() << endl;
	
	auto itListMax = max_element(std::begin(List),std::end(List));

	int maxList = round(*itListMax);

	//Code for debugging
	/*
	for (int i = 0; i < List.size(); i++)
	{
		cout << List[i] << endl;
		cout << destOri[i] << endl;
		cout << endl;
	}
	*/

	cout << endl;

	for (int i = 0; i <= maxList; i++)
	{
		_destinationOrientation.push_back(0);
	}

	for (int i = 0; i < List.size(); i++)
	{
		int xPos = round(Coords[i].first);
		int yPos = round(Coords[i].second);
		int nodeIndex = _coordNodeIndex[std::pair<int, int>(xPos,yPos)];
		if (_destinationList.find(List[i]) == _destinationList.end())
		{
			vector<int> tempA;
			tempA.push_back(nodeIndex);
			_destinationList.insert(pair<int, vector<int> >(List[i], tempA));
			_destinationOrientation[List[i]] = destOri[i];
		}
		else
		{
			_destinationList[List[i]].push_back(nodeIndex);
		}
	}
	/*
	for (int i = 0; i < _destinationOrientation.size(); i++)
	{
		cout << "destOri:" << _destinationOrientation[i] << endl;
	}
	*/
	_initialized = 1;
}
