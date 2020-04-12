#include "myClustering.h"

template<typename PointT>
void KdTree<PointT>::insert(PointT point, int id)
{
	insertHelper(root, 0, point, id);
}

template<typename PointT>
void KdTree<PointT>::insertHelper(Node<PointT>* &node, uint curDepth, PointT point, int id)
{
	if (node == NULL) {
		node = new Node<PointT>(point, id);
	} else {
		uint dimIndex = curDepth % 3;
		float pointDimVal;
		float nodeDimVal;
		switch (dimIndex) {
			case 0:
				pointDimVal=point.x;
				nodeDimVal=node->point.x;
				break;
			case 1:
				pointDimVal=point.y;
				nodeDimVal=node->point.y;
				break;
			case 2:
				pointDimVal=point.z;
				nodeDimVal=node->point.z;
				break;
		}

		if (pointDimVal < nodeDimVal) {
			// move to the left subtree
			insertHelper(node->left, curDepth+1, point, id);
		} else {
			// move to the right subtree
			insertHelper(node->right, curDepth+1, point, id);
		}
	}
}

template<typename PointT>
void KdTree<PointT>::insertCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
{
	for (int i=0; i < cloud->points.size(); i++) {
		insert(cloud->points[i], i);
	}
}

// return a list of point ids in the tree that are within distance of target
template<typename PointT>
typename pcl::PointIndices::Ptr KdTree<PointT>::search(PointT target, float distanceTol)
{
	//std::vector<int> ids;
	pcl::PointIndices::Ptr ids (new pcl::PointIndices ());

	searchHelper(target, distanceTol, root, 0, ids);

	return ids;
}

template<typename PointT>
void KdTree<PointT>::searchHelper(PointT target, float distanceTol, Node<PointT>* node, int depth, typename pcl::PointIndices::Ptr &ids)
{
	if (node != NULL) {
		if ( (node->point.x >= (target.x - distanceTol)) &&
			 (node->point.x <= (target.x + distanceTol)) &&
			 (node->point.y >= (target.y - distanceTol)) &&
			 (node->point.y <= (target.y + distanceTol)) &&
			 (node->point.z >= (target.z - distanceTol)) &&
			 (node->point.z <= (target.z + distanceTol)) )
		{
			float dist = sqrt(pow(node->point.x - target.x, 2) + pow(node->point.y - target.y, 2) + pow(node->point.z - target.z, 2));
			if (dist <= distanceTol)
				ids->indices.push_back(node->id);
		}

		uint dimIndex = depth % 3;
		float targetDimVal;
		float nodeDimVal;
		switch (dimIndex) {
			case 0:
				targetDimVal=target.x;
				nodeDimVal=node->point.x;
				break;
			case 1:
				targetDimVal=target.y;
				nodeDimVal=node->point.y;
				break;
			case 2:
				targetDimVal=target.z;
				nodeDimVal=node->point.z;
				break;
		}

		if ((targetDimVal-distanceTol) < nodeDimVal)
			searchHelper(target, distanceTol, node->left, depth+1, ids);
		if ((targetDimVal+distanceTol) > nodeDimVal)
			searchHelper(target, distanceTol, node->right, depth+1, ids);
	}
}

template<typename PointT>
void EuclideanClusterExtractor<PointT>::clusterHelper(int index, pcl::PointIndices &cluster, std::vector<bool> &processed, float distanceTol)
{
	processed[index] = true;
	cluster.indices.push_back(index);

	pcl::PointIndices::Ptr near_points = tree->search(cloud->points[index], distanceTol);

	for(int id : near_points->indices){
		if(!processed[id])
			clusterHelper(id, cluster, processed, distanceTol);
	}
}

template<typename PointT>
std::vector<pcl::PointIndices> EuclideanClusterExtractor<PointT>::euclideanCluster(float distanceTol, int minSize, int maxSize)
{
	std::vector<pcl::PointIndices> clusters;

	std::vector<bool> processed(cloud->points.size(), false);

	for (int i=0; i < cloud->points.size(); i++) {
		if (processed[i])
			continue;

		pcl::PointIndices cluster;
		clusterHelper(i, cluster, processed, distanceTol);
		if (cluster.indices.size() >= minSize && cluster.indices.size() <= maxSize)
			clusters.push_back(cluster);
	}

	return clusters;

}

//template class KdTree<pcl::PointXYZI>;

