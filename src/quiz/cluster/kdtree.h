/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


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

	void insertHelper(Node *&node, unsigned int depth, std::vector<float> point, int id) 
	{
		// tree_dim --> 3- or 2-dimensional kdtree? 
		int tree_dim = point.size();
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		if (node == NULL)
		{
			node = new Node(point, id);
		}
		else
		{
			// dimension to compare depends on depth
			unsigned int dim = depth % tree_dim;

			if (point[dim] < node->point[dim])
			{
				insertHelper(node->left, depth + 1, point, id);
			}
			else
			{
				insertHelper(node->right, depth + 1, point, id);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		insertHelper(root, 0, point, id);
	}
	
	
	void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int> &ids)
	{
		// tree_dim --> 3- or 2-dimensional kdtree? 
		int tree_dim = target.size();

		if (node != NULL)
		{
			if (tree_dim == 2) {
				if ((node->point[0] >= target[0] - distanceTol) && (node->point[0] <= target[0] + distanceTol)
					&& (node->point[1] >= target[1] - distanceTol) && (node->point[1] <= target[1] + distanceTol))
				{
					float distance = sqrt((node->point[0] - target[0]) * (node->point[0] - target[0]) + (node->point[1] - target[1]) * (node->point[1] - target[1]));
					if (distance < distanceTol)
						ids.push_back(node->id);
				}
			}
			
			else
			{
				if ((node->point[0] >= target[0] - distanceTol) && (node->point[0] <= target[0] + distanceTol)
					&& (node->point[1] >= target[1] - distanceTol) && (node->point[1] <= target[1] + distanceTol)
					&& (node->point[2] >= target[2] - distanceTol) && (node->point[2] <= target[2] + distanceTol))
				{
					float distance = sqrt((node->point[0] - target[0]) * (node->point[0] - target[0]) + (node->point[1] - target[1]) * (node->point[1] - target[1]));
					if (distance < distanceTol)
						ids.push_back(node->id);
				}
			}
			// check across boundary
			if ((target[depth % tree_dim] - distanceTol) < node->point[depth % tree_dim])
				searchHelper(target, node->left, depth + 1, distanceTol, ids);
			if ((target[depth % tree_dim] + distanceTol) > node->point[depth % tree_dim])
				searchHelper(target, node->right, depth + 1, distanceTol, ids);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
	
	void clusterHelper(int indice, const std::vector<std::vector<float>> points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol) {
		processed[indice] = true;
		cluster.push_back(indice);

		// search nearest points in tree within tolerance
		std::vector<int> nearest = tree->search(points[indice], distanceTol);

		for (int id : nearest) {
			if (!processed[id])
				clusterHelper(id, points, cluster, processed, tree, distanceTol);
		}
	}
};




