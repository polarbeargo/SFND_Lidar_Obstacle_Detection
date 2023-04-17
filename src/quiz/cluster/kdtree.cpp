/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "kdtree.h"

void KdTree::insertHelper(Node **node, uint depth, std::vector<float> point, int id)
{
	// TODO: Fill in this function to insert a new point into the tree
	// the function should create a new node and place correctly with in the root
	if (*node == NULL)
	{
		*node = new Node(point, id);
	}
	else
	{
		uint cd = depth % 3;
		if (point[cd] < ((*node)->point[cd]))
			insertHelper(&((*node)->left), depth + 1, point, id);
		else
			insertHelper(&((*node)->right), depth + 1, point, id);
	}
}

void KdTree::insert(std::vector<float> point, int id)
{
	// TODO: Fill in this function to insert a new point into the tree
	// the function should create a new node and place correctly with in the root
	insertHelper(&root, 0, point, id);
}

void KdTree::searchHelper(std::vector<float> target, Node *node, int depth, float distanceTol, std::vector<int> &ids)
{
	if (node != NULL)
	{
		bool x = node->point[0] >= (target[0] - distanceTol) &&
				 node->point[0] <= (target[0] + distanceTol);
		bool y = node->point[1] >= (target[1] - distanceTol) &&
				 node->point[1] <= (target[1] + distanceTol);
		bool z = node->point[2] >= (target[2] - distanceTol) &&
				 node->point[2] <= (target[2] + distanceTol);
		if (x && y && z)
		{
			float distance = sqrt((node->point[0] - target[0]) * (node->point[0] - target[0]) + (node->point[1] - target[1]) * (node->point[1] - target[1]) + (node->point[2] - target[2]) * (node->point[2] - target[2]));
			if (distance <= distanceTol)
				ids.push_back(node->id);
		}

		if (target[depth % 3] - distanceTol < node->point[depth % 3])
			searchHelper(target, node->left, depth + 1, distanceTol, ids);
		if (target[depth % 3] + distanceTol > node->point[depth % 3])
			searchHelper(target, node->right, depth + 1, distanceTol, ids);
	}
}

// return a list of point ids in the tree that are within distance of target
std::vector<int> KdTree::search(std::vector<float> target, float distanceTol)
{
	std::vector<int> ids;
	searchHelper(target, root, 0, distanceTol, ids);
	return ids;
}
