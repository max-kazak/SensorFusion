// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include "myClustering.h"
#include <unordered_set>
// using templates for myClustering so also include .cpp to help linker
#include "myClustering.cpp"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Create the filtering object
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT> ());
	pcl::VoxelGrid<PointT> vg;
	vg.setInputCloud (cloud);
	vg.setLeafSize (filterRes, filterRes, filterRes);
	vg.filter (*cloudFiltered);

	typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT> ());

	pcl::CropBox<PointT> region(true);
	region.setMin(minPoint);
	region.setMax(maxPoint);
	region.setInputCloud(cloudFiltered);
	region.filter(*cloudRegion);

	std::vector<int> indices;

	pcl::CropBox<PointT> roof(true);
	roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
	roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
	roof.setInputCloud(cloudRegion);
	roof.filter(indices);

	pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
	for (int point: indices)
		inliers->indices.push_back(point);

	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(cloudRegion);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // Create two new point clouds, one cloud with obstacles and other with segmented plane

	typename pcl::PointCloud<PointT>::Ptr inliersCloud (new pcl::PointCloud<PointT> ());
	typename pcl::PointCloud<PointT>::Ptr outliersCloud (new pcl::PointCloud<PointT> ());

	// Extract the inliers
//	for (int index: inliers->indices):
//			inliersCloud -> points.push_back(cloud->points[index]);

	// Create the filtering object
	pcl::ExtractIndices<PointT> extract;

	// Extract the inliers
	extract.setInputCloud (cloud);
	extract.setIndices (inliers);
	extract.setNegative (false);
	extract.filter (*inliersCloud);

	// Extract the outliers
	//extract.setInputCloud (cloud);
	//extract.setIndices (inliers);
	extract.setNegative (true);
	extract.filter (*outliersCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(outliersCloud, inliersCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	// Create the segmentation object
	pcl::SACSegmentation<PointT> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (maxIterations);
	seg.setDistanceThreshold (distanceThreshold);

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
    	std::cout << "Could not estimate planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::MySegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();


	pcl::PointIndices::Ptr inliersPointIndices (new pcl::PointIndices ());

	// RANSAC BEGIN
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	int nPoints = cloud -> points.size();

	while (maxIterations--) {
	// Randomly sample subset and fit line
		std::unordered_set<int> inliers;
		while(inliers.size()<3)
			inliers.insert(std::rand() % nPoints);

		auto itr = inliers.begin();
		PointT p1 = cloud -> points[*itr];
		itr++;
		PointT p2 = cloud -> points[*itr];
		itr++;
		PointT p3 = cloud -> points[*itr];

		// Ax+By+Cz+D=0
		float a = (p2.y-p1.y)*(p3.z-p1.z)-(p2.z-p1.z)*(p3.y-p1.y);
		float b = (p2.z-p1.z)*(p3.x-p1.x)-(p2.x-p1.x)*(p3.z-p1.z);
		float c = (p2.x-p1.x)*(p3.y-p1.y)-(p2.y-p1.y)*(p3.x-p1.x);
		float d = -(a*p1.x+b*p1.y+c*p1.z);

		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		for (int i=0; i<nPoints; i++) {

			if (inliers.count(i)>0)
				// skip point if already part of inliers
				continue;

			PointT p = cloud -> points[i];
			float dist = fabs(a*p.x + b*p.y + c*p.z + d) / sqrt(a*a + b*b + c*c);
			if (dist <= distanceTol)
				inliers.insert(i);
		}

		if (inliers.size() > inliersResult.size())
			inliersResult = inliers;

	}
	// RANSAC END

	for (int point: inliersResult)
		inliersPointIndices->indices.push_back(point);


    if (inliersPointIndices->indices.size() == 0) {
    	std::cout << "Could not estimate planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliersPointIndices, cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Creating the KdTree object for the search method of the extraction
	typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud (cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance (clusterTolerance); // 2cm
	ec.setMinClusterSize (minSize);
	ec.setMaxClusterSize (maxSize);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud);
	ec.extract (cluster_indices);

	for (pcl::PointIndices getIndices: cluster_indices) {
		typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

		for (int i: getIndices.indices)
			cloudCluster->points.push_back(cloud->points[i]);

		cloudCluster -> width = cloudCluster->points.size();
		cloudCluster -> height = 1;
		cloudCluster -> is_dense = true;

		clusters.push_back(cloudCluster);
	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::MyClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Creating the KdTree object for the search method of the extraction
    KdTree<PointT>* tree = new KdTree<PointT>();
    tree->insertCloud(cloud);

    // extract cluster_indices using euclidean extraction
    EuclideanClusterExtractor<PointT> ex;
    ex.setCloud(cloud);
    ex.setTree(tree);

    std::vector<pcl::PointIndices> cluster_indices;
    cluster_indices = ex.euclideanCluster(clusterTolerance, minSize, maxSize);

	for (pcl::PointIndices getIndices: cluster_indices) {
		typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

		for (int i: getIndices.indices)
			cloudCluster->points.push_back(cloud->points[i]);

		cloudCluster -> width = cloudCluster->points.size();
		cloudCluster -> height = 1;
		cloudCluster -> is_dense = true;

		clusters.push_back(cloudCluster);
	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
