#ifndef MYCLUSTERING_H
#define MYCLUSTERING_H

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

#include <math.h>
#include <vector>


// Structure to represent node of kd tree
template<typename PointT>
struct Node
{
	PointT point;
	int id;
	Node<PointT>* left;
	Node<PointT>* right;

	Node(PointT arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

template<typename PointT>
class KdTree
{
	Node<PointT>* root;

public:

	KdTree()
	: root(NULL)
	{}

	void insert(PointT point, int id);

	void insertHelper(Node<PointT>* &node, uint curDepth, PointT point, int id);

	void insertCloud(typename pcl::PointCloud<PointT>::Ptr cloud);

	// return a list of point ids in the tree that are within distance of target
	typename pcl::PointIndices::Ptr search(PointT target, float distanceTol);

	void searchHelper(PointT target, float distanceTol, Node<PointT>* node, int depth, typename pcl::PointIndices::Ptr &ids);

};


template<typename PointT>
struct EuclideanClusterExtractor
{
	KdTree<PointT>* tree;
	typename pcl::PointCloud<PointT>::Ptr cloud;

	EuclideanClusterExtractor()
	{}

	EuclideanClusterExtractor(typename pcl::PointCloud<PointT>::Ptr _cloud, KdTree<PointT>* _tree)
	:	cloud(_cloud), tree(_tree)
	{}

	void setCloud(typename pcl::PointCloud<PointT>::Ptr _cloud)
	{
		cloud = _cloud;
	}

	void setTree(KdTree<PointT>* _tree)
	{
		tree = _tree;
	}

	void clusterHelper(int index, pcl::PointIndices &cluster, std::vector<bool> &processed, float distanceTol);

	std::vector<pcl::PointIndices> euclideanCluster(float distanceTol, int minSize, int maxSize);
};

#endif
