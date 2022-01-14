// maze_magic.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "opencv2/opencv.hpp"
#include <iostream>
#include <string>
#include <filesystem>

using namespace cv;
using namespace std;

Vec3b redColor(50, 50, 200);
Vec3b greenColor(50, 200, 50);
Vec3b blueColor(200, 50, 50);
Vec3b blackColor(0, 0, 0);
Vec3b pathColor = (0, 0, 255);


class mNode
{
public:
	mNode* up, * down, * left, * right;
	vector<mNode*> nodes;
	int x, y, tentDist;
	double localGoal, globalGoal;
	mNode* parent;
	bool start;
	bool end;
	bool visited;

	mNode()
	{
		up = NULL;
		down = NULL;
		left = NULL;
		right = NULL;
		x = -1;
		y = -1;
		start = false;
		end = false;
		visited = false;
		parent = NULL;
	}
	mNode(int _x, int _y)
	{
		up = NULL;
		down = NULL;
		left = NULL;
		right = NULL;
		x = _x;
		y = _y;
		start = false;
		end = false;
		visited = false;
		parent = NULL;
	}

	void addNodesToVec()
	{
		if (up != NULL)
			nodes.push_back(up);
		if (down != NULL)
			nodes.push_back(down);
		if (left != NULL)
			nodes.push_back(left);
		if (right != NULL)
			nodes.push_back(right);
	}
};

//100, 1000, 1000_braid, 10000 
double distance(mNode* a, mNode* b) //~1050nodes, 148226 nodes in 0.68s, 112041 nodes, 16.934.393 nodes in 45s
{
	return sqrt((a->x - b->x)*(a->x - b->x) + (a->y - b->y)*(a->y - b->y));
}

double distanceSquared(mNode* a, mNode* b) //~520 nodes, 3070 nodes in 0.11s, 40551 nodes, 9.102.543 nodes in 845s
{
	return (a->x - b->x)*(a->x - b->x) + (a->y - b->y)*(a->y - b->y);
}

double manhattanDistance(mNode* a, mNode* b) // ~1020 nodes, 133858 nodes in 0.68s, 109090 nodes, 16.922.031 in 46s
{
	return abs(a->x - b->x) + abs(a->y - b->y);
}

double distanceHalved(mNode* a, mNode* b) //~1050nodes, 148226 nodes in 0.68s, 112041 nodes, 16.934.393 nodes in 45s
{
	return sqrt((a->x - b->x)*(a->x - b->x) + (a->y - b->y)*(a->y - b->y))/2.0;
}

double heuristic(mNode* a, mNode* b)
{
	return distance(a, b);
}


void drawNode(Mat input, int x, int y)
{
	input.at<Vec3b>(Point(x, y)) = greenColor;
}

void drawPath(Mat input, int x, int y)
{
	input.at<Vec3b>(Point(x, y)) = pathColor;
}


int main()
{
	filesystem::path cwd = filesystem::current_path();
	char input[100] = {};
	cout << "Input target image with maze, must be .bmp format \n";
	cin.getline(input, 25);
	cout << input << "\n";
	string fileName = string(input);
	cout << "Loading file: " << fileName << "\n";
	Mat maze = imread(fileName, IMREAD_UNCHANGED);
	
	if (maze.empty())
	{
		cout << "Could not read that file, please try again.";
		exit(-1);
	}

	float scale = 1000.0 / float(maze.cols);
	
	cout << "scaling to " << scale << "\n";
	if (scale < .1)
		scale = .1;

	Mat nodeImg;
	maze.copyTo(nodeImg);
	cvtColor(nodeImg, nodeImg, COLOR_GRAY2RGB);

	int64 start = getTickCount();

	cout << "Finding start and end nodes" << endl;
	//find start and end node:
	for (int i = 0; i < maze.cols; i++)
	{
		if (maze.at<bool>(Point(i, 0)))
			nodeImg.at<Vec3b>(Point(i, 0)) = blueColor;
		if (maze.at<bool>(Point(i, maze.rows - 1)))
			nodeImg.at<Vec3b>(Point(i, maze.rows - 1)) = redColor;
	}
	cout << "Finding other nodes" << endl;
	//find all other nodes:
	for (int y = 1; y < maze.rows - 1; y++)
	{
		for (int x = 1; x < maze.cols - 1; x++)
		{
			if (maze.at<bool>(Point(x, y)))
			{
				bool topWall = !maze.at<bool>(Point(x, y - 1));
				bool leftWall = !maze.at<bool>(Point(x - 1, y));
				bool bottomWall = !maze.at<bool>(Point(x, y + 1));
				bool rightWall = !maze.at<bool>(Point(x + 1, y));

				int sum = (int)topWall + (int)leftWall + (int)bottomWall + (int)rightWall;
				//cout << "x: " << x << ", y: " << y << ", sum: " << sum << endl;
				//char a;
				//cin >> a;

				if (sum == 0 || sum == 1 || sum == 3) //4-way junc, t-section, dead end
				{
					//addNode;
					drawNode(nodeImg, x, y);
				}
				else //corridor or corner, if corridor, place node, otherwise skip
				{
					if ((topWall && bottomWall) || (leftWall && rightWall)) //corridor, skip
					{
						//skip
					}
					else //corner
					{
						//addNode;
						drawNode(nodeImg, x, y);
					}
				}
			}
		}
	}
	int64 now = getTickCount();
	cout << "Done processing images after " << (now - start) / getTickFrequency() << " seconds \n";

	//all nodes are found, and drawn in nodeImg, create nodetree:
	cout << "Readying node matrix" << endl;
	//vector matrix with all empty nodes for now
	vector<vector<mNode*>> nodeMatrix(nodeImg.cols, vector<mNode*>(nodeImg.cols));
	
	cout << "Adding start and end node to matrix" << endl;
	//add start and end node:
	for (int i = 0; i < nodeImg.cols; i++)
	{
		if (nodeImg.at<Vec3b>(Point(i, 0)) == blueColor)
		{
			mNode node(i, 0);
			node.start = true;
			nodeMatrix[0][i] = &node;
			cout << "added start node at x,y: " << i << "," << 0 << endl;
		}

		if (nodeImg.at<Vec3b>(Point(i, nodeImg.rows - 1)) == redColor)
		{
			mNode node(i, nodeImg.rows - 1);
			node.end = true;
			nodeMatrix[nodeImg.rows - 1][i] = &node;
			cout << "added end node at x,y: " << i << "," << nodeImg.rows - 1 << endl;
		}
	}


	cout << "Adding all other nodes to matrix" << endl;
	//add all other nodes:
	for (int y = 0; y < nodeImg.rows - 1; y++)
	{
		for (int x = 1; x < nodeImg.cols - 1; x++)
		{
			if (nodeImg.at<Vec3b>(Point(x, y)) == greenColor || nodeImg.at<Vec3b>(Point(x, y)) == blueColor)
			{
				mNode* node = nodeMatrix[y][x];
				if(node == NULL)
					node = new mNode(x, y);
				//add node connections:
				//traverse right:
				bool done = false;
				int _x = x;
				int _y = y;
				while (!done)
				{
					_x++;
					if (nodeImg.at<Vec3b>(Point(_x, _y)) == greenColor)
					{ //found right node
						if (nodeMatrix[_y][_x] == NULL)
							nodeMatrix[_y][_x] = new mNode(_x, _y);
						node->right = nodeMatrix[_y][_x];
						nodeMatrix[_y][_x]->left = node;
						done = true;
					}
					
					if (nodeImg.at<Vec3b>(Point(_x, _y)) == blackColor)
					{ // no right node
						done = true;
					}
				}
				//traverse down:
				done = false;
				_x = x;
				_y = y;
				while (!done)
				{
					_y++;
					if (nodeImg.at<Vec3b>(Point(_x, _y)) == greenColor)
					{ //found bottom node
						if (nodeMatrix[_y][_x] == NULL)
							nodeMatrix[_y][_x] = new mNode(_x, _y);
						node->down = nodeMatrix[_y][_x];
						nodeMatrix[_y][_x]->up = node;

						done = true;
					}
					else if (nodeImg.at<Vec3b>(Point(_x, _y)) == redColor)
					{
						node->down = nodeMatrix[_y][_x];
						nodeMatrix[_y][_x]->up = node;
						cout << "found end node below, end = " << node->down->end << "\n";
						done = true;
					}
					else if (nodeImg.at<Vec3b>(Point(_x, _y)) == blackColor)
					{
						done = true;
					}
				}

				nodeMatrix[y][x] = node;
				//cout << "Added node at x,y: " << x << "," << y << endl;
			}
		}
	}
	now = getTickCount();
	cout << "Done making node matrix after " << (now - start) / getTickFrequency() << " seconds \n";

	cout << "Making node list \n";
	vector<mNode*> nodeList;
	for (int i = 0; i < nodeMatrix.size(); i++)
	{
		for (int j = 0; j < nodeMatrix[i].size(); j++)
		{
			if (nodeMatrix[i][j] != NULL)
			{
				nodeList.push_back(nodeMatrix[i][j]);
			}
		}
		nodeMatrix[i].clear();
	}
	nodeMatrix.clear();
	now = getTickCount();
	cout << "Done with node list, size: "<< nodeList.size() << ", after " << (now - start) / getTickFrequency() << " seconds \n";


	//path find:
	mNode* nodeStart = nullptr;
	mNode* nodeEnd = nullptr;
	for (int i = 0; i < nodeList.size(); i++)
	{
		if (nodeList[i]->start)
			nodeStart = nodeList[i];
		if (nodeList[i]->end)
			nodeEnd = nodeList[i];

		nodeList[i]->globalGoal = INFINITY;
		nodeList[i]->localGoal = INFINITY;
		nodeList[i]->visited = false;
		nodeList[i]->addNodesToVec();
	}
	
	if(nodeStart == NULL)
		return 1;
	if (nodeEnd == NULL)
		return 2;


	mNode* curNode;
	curNode = nodeStart;
	curNode->localGoal = 0.0;
	curNode->globalGoal = heuristic(nodeStart, nodeEnd);


	list<mNode*> openList;
	openList.push_back(nodeStart);

	Mat pathCheckImg;
	Mat upscaledPathCheck;
	maze.copyTo(pathCheckImg);
	cvtColor(pathCheckImg, pathCheckImg, COLOR_GRAY2RGB);
	now = getTickCount();
	cout << "Starting on A* after " << (now - start) / getTickFrequency() << " seconds \n";
	int nodesVisited = 0;
	while (!openList.empty() && curNode != nodeEnd)
	{
		if (nodesVisited % 10 == 0)
		{
			now = getTickCount();
			cout << "Openlist size: " << openList.size() << ", processed " << nodesVisited << " out of " << nodeList.size() << " nodes after " << (now - start) / getTickFrequency() << " seconds \n";
			resize(pathCheckImg, upscaledPathCheck, Size(0, 0), scale, scale, INTER_NEAREST);
			imshow("path checked", upscaledPathCheck);
			waitKey(1);
		}
		//cout << "a star while, size: " << openList.size() << "\n";
		openList.sort([](const mNode* lhs, const mNode* rhs) { return lhs->globalGoal < rhs->globalGoal;  });

		while (!openList.empty() && openList.front()->visited)
			openList.pop_front();

		if (openList.empty())
			break;

		curNode = openList.front();
		curNode->visited = true;
		nodesVisited++;
		drawPath(pathCheckImg, curNode->x, curNode->y);
		//cout << "num neighbors: " << curNode->nodes.size() <<", x,y: " << curNode->x << "," << curNode->y << ", up, down, left, right == NULL: " << (curNode->up == NULL) << "," << (curNode->down == NULL) << "," << (curNode->left == NULL) << "," << (curNode->right == NULL) << "\n";
		for (auto neighbor : curNode->nodes)
		{
			if (!neighbor->visited)
				openList.push_back(neighbor);

			double possiblyLowerGoal = curNode->localGoal; + distance(curNode, neighbor);
			
			if (possiblyLowerGoal < neighbor->localGoal)
			{
				neighbor->parent = curNode;
				neighbor->localGoal = possiblyLowerGoal;
				neighbor->globalGoal = neighbor->localGoal; + heuristic(neighbor, nodeEnd);
			}
		}
	}
	now = getTickCount();
	cout << "done with A*, visited " << nodesVisited << " nodes, after " << (now - start) / getTickFrequency() << " seconds \n";
	cv::destroyWindow("path checked");
	//draw path:
	Mat pathImg;
	maze.copyTo(pathImg);
	cvtColor(pathImg, pathImg, COLOR_GRAY2RGB);

	mNode* path = curNode;
	while (path != NULL)
	{
		drawPath(pathImg, path->x, path->y);

		if (path->parent != NULL)
		{
			int xPos = path->x;
			while (xPos != path->parent->x)
			{
				if (path->parent->x < xPos)
					xPos--;
				else
					xPos++;
				drawPath(pathImg, xPos, path->y);
			}
			int yPos = path->y;
			while (yPos != path->parent->y)
			{
				if (path->parent->y < yPos)
					yPos--;
				else
					yPos++;
				drawPath(pathImg, path->x, yPos);
			}
		}
		
		path = path->parent;
	}
	now = getTickCount();
	cout << "drew a path after " << (now - start) / getTickFrequency() << " seconds \n";


	//scale images and show them:
	Mat upscaled;
	//resize(maze, upscaled, Size(0, 0), scale, scale, InterpolationFlags::INTER_NEAREST);
	//imshow("maze", upscaled);
	resize(nodeImg, nodeImg, Size(0, 0), scale, scale, INTER_NEAREST);
	imshow("All nodes in the maze", nodeImg);
	resize(pathImg, pathImg, Size(0, 0), scale, scale, INTER_NEAREST);
	imshow("Final found path", pathImg);
	resize(pathCheckImg, pathCheckImg, Size(0, 0), scale, scale, INTER_NEAREST);
	imshow("Nodes that has been visited", pathCheckImg);


	cout << "Done! " << endl;
	waitKey();
	
	
	return 0;
}