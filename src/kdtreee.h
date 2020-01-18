#include "render/render.h"
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

// Structure to represent node of kd tree
template<typename PointT>
struct Node
{
	PointT point;
	Node<PointT>* left{NULL};
	Node<PointT>* right{NULL};
    int id;

	Node(PointT arr, int Id)
		: point(arr), id(Id){
	}
};

template<typename PointT>
struct KdTree
{
	Node<PointT> *root{NULL};


	KdTree(typename pcl::PointCloud<PointT>::Ptr cloud){
		for(int i = 0; i < cloud->points.size(); i++){
			insert(cloud->points[i], i);
		}

	}

	void insert(PointT point, int id)
	{
		insertHelper(&root, point, 0, id);
	}

	void insertHelper(Node<PointT> **node,  PointT point,  int level, int id)
	{


		if (*node == NULL)
		{
			*node = new Node<PointT>(point, id);
		}
		else
		{
			int cd = level % 3;

			if (point.data[cd] < (*node)->point.data[cd])
			{
				insertHelper(&((*node)->left), point,  level + 1, id);
			}
			else
			{
				insertHelper(&((*node)->right), point,  level + 1, id);
			}
		}

		return;
	}

	void searchHelper(PointT target,  Node<PointT> *node, int depth, float distanceTol, std::vector<int> &ids)
	{

		if (node != NULL)
		{
			if ((node->point.data[0] >= (target.data[0] - distanceTol) && node->point.data[0] <= (target.data[0] + distanceTol) && node->point.data[1] >= (target.data[1] - distanceTol) && node->point.data[1] <= (target.data[1] + distanceTol) && node->point.data[2] >= (target.data[2] - distanceTol) && node->point.data[2] <= (target.data[2] + distanceTol)  ))
			{
				//TODO:calculate 3d distance
				float distance = sqrt((node->point.data[0] - target.data[0]) * (node->point.data[0] - target.data[0]) + (node->point.data[1] - target.data[1]) * (node->point.data[1] - target.data[1]) +  (node->point.data[2] - target.data[2]) * (node->point.data[2] - target.data[2]));
				if (distance <= distanceTol)
				{
					ids.push_back(node->id);
				}
			}

			if ((target.data[depth % 3] - distanceTol) < node->point.data[depth % 3])
			{
				searchHelper(target, node->left, depth + 1, distanceTol, ids);
			}
			if ((target.data[depth % 3] + distanceTol) > node->point.data[depth % 3])
			{
				searchHelper(target, node->right, depth + 1, distanceTol, ids);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	
	std::vector<int> search( PointT target, float distanceTol)
	{	std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
};
