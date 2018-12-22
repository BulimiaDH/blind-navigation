/*
 *********************
  Static Map Class Header 
  Created by Hongyi Fan
  Dec. 1 2017
  *********************/

#ifndef __STATICMAP_h_
#define __STATICMAP_h_

#include <iostream>
#include <stdio.h>
#include <string>
#include <map>
#include <vector>
#include <utility>
#include <cmath>
#include <sstream>
#include <fstream>
#include <algorithm>
using namespace std;

class staticMap{
public:
	staticMap();
	staticMap(std::string filePath, std::string destinationPath);
	~staticMap()
	{
		for (int i = 0; i < _height; i++)
		{
			delete[] _AdjacentMatrix[i];
		}
		delete[] _AdjacentMatrix;
	};

	//Accessor
	std::map<pair<int,int>, bool> getValidMap(){return _validMap;};
	std::map<int, int> getImageNodeIndex(){return _imageNodeIndex;};
	std::vector<int> getDestinationOrientationList(){return _destinationOrientation;};
	std::pair<int, int> getNodeCoordinateFromIndex(int nodeIndex){return _nodeCoordinatesIndex[nodeIndex];}; 
	int getNodeIndexFromCoordinate(pair<int,int> nodeCoordinate){return _coordNodeIndex[nodeCoordinate];};

	int getImageCoordinateViaIndex(int imageIndex){return _imageNodeIndex[imageIndex];};
	bool getGridValidation(int x, int y){return _validMap[pair<int,int>(x,y)];};	
	int** getAdjacentMatrix(){return _AdjacentMatrix;};
	int getNumNodes(){return _numNodes;};
	int getWidth(){return _width;};
	int getHeight(){return _height;};
	int getInitStatus() {return _initialized;};
	vector<int> getDestinationList(int destinationIndex);
	int getDestinationOrientation(int destinationIndex){return _destinationOrientation[destinationIndex];};
	int getImageOrientation(int imageIndex);
private:
	bool _initialized;
	int _width;
	int _height;
	int _numNodes;
	std::vector<float> _xPos;
	std::vector<float> _yPos;
	std::vector<int> _ori;

	std::vector<int> _xNode;
	std::vector<int> _yNode;

	std::map<pair<int,int>,bool> _validMap;     
	std::map<int, int> _imageNodeIndex; 
	std::map<int, int> _imageOrientationIndex;
	std::map<int, vector<int> > _destinationList;
	std::vector<int> _destinationOrientation;
	std::map<int, pair<int,int>  > _nodeCoordinatesIndex;    
	std::map<pair<int,int>, int> _coordNodeIndex; 			
	int** _AdjacentMatrix;
};
#endif
