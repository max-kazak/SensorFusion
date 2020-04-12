/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <math.h>


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		insertHelper(root, 0, point, id);
	}

	void insertHelper(Node* &node, uint curDepth, std::vector<float> point, int id)
	{
		if (node == NULL) {
			node = new Node(point, id);
		} else {
			uint dimIndex = curDepth % node->point.size();

			if (point[dimIndex] < node->point[dimIndex]) {
				// move to the left subtree
				insertHelper(node->left, curDepth+1, point, id);
			} else {
				// move to the right subtree
				insertHelper(node->right, curDepth+1, point, id);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		searchHelper(target, distanceTol, root, 0, ids);

		return ids;
	}
	
	void searchHelper(std::vector<float> target, float distanceTol, Node* node, int depth, std::vector<int> &ids)
	{
		if (node != NULL) {
			if ( (node->point[0] >= (target[0] - distanceTol)) &&
				 (node->point[0] <= (target[0] + distanceTol)) &&
				 (node->point[1] >= (target[1] - distanceTol)) &&
				 (node->point[1] <= (target[1] + distanceTol)) )
			{
				float dist = sqrt(pow(node->point[0] - target[0], 2) + pow(node->point[1] - target[1], 2));
				if (dist <= distanceTol)
					ids.push_back(node->id);
			}

			if ((target[depth%2]-distanceTol) < node->point[depth%2])
				searchHelper(target, distanceTol, node->left, depth+1, ids);
			if ((target[depth%2]+distanceTol) > node->point[depth%2])
				searchHelper(target, distanceTol, node->right, depth+1, ids);
		}
	}


};




