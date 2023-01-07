#ifndef KDTREE_H_
#define KDTREE_H_
/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

// Structure to represent node of kd tree
class Node
{
public:
	std::vector<float> point;
	int id;
	Node *left;
	Node *right;

	Node(std::vector<float> arr, int setId);
	~Node()
	{
		delete left;
		delete right;
	}
};

class KdTree
{
public:
	Node *root;

	KdTree();
	~KdTree()
	{
		delete root;
	}

	void insertHelper(Node **node, u_int depth, std::vector<float> point, int id);

	void insert(std::vector<float> point, int id);

	void searchHelper(std::vector<float> target, Node *node, int depth, float distanceTol, std::vector<int> &ids);
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol);
};

#endif
