// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>

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

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    // create filtered object: downssampling the dataset using a leaf size of 0.2 m
    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud <PointT> ());

    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);
    
    typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT> ());

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -.4, 1));
    roof.setInputCloud(cloudRegion);

    pcl::PointIndices::Ptr inliers{ new pcl::PointIndices };
    for (int point : indices)
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
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
    // extract inliers from cloud --> planeCloud        
    for (int index : inliers->indices) {
        planeCloud->push_back(cloud->points[index]);        
    }

    // extract other points not belonging to plane --> setNegative True
    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;    
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);    
    extract.filter(*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    std::unordered_set<int> inliersResult;
    //pcl::PointIndices::Ptr pt_inliers(new pcl::PointIndices());        
    
    srand(time(NULL));

    // TODO: Fill in this function

    // For max iterations 
    for (int i = 0; i < maxIterations; ++i) {
        // Randomly sample subset and fit line   
        std::unordered_set<int> inliers;
        // unordered_set --> do not enter same index twice

        // insert three random points as inliers
        while (inliers.size() < 3)
            inliers.insert(rand() % cloud->points.size());

        float x1, x2, x3, y1, y2, y3, z1, z2, z3;
        auto it = inliers.begin();
        x1 = cloud->points[*it].x;
        y1 = cloud->points[*it].y;
        z1 = cloud->points[*it].z;
        it++;
        x2 = cloud->points[*it].x;
        y2 = cloud->points[*it].y;
        z2 = cloud->points[*it].z;
        it++;
        x3 = cloud->points[*it].x;
        y3 = cloud->points[*it].y;
        z3 = cloud->points[*it].z;

        // calc cross-product v1 x v2 and A=i, B=j, C=k
        float A = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
        float B = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
        float C = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
        float D = -(A * x1 + B * y1 + C * z1);

        for (int j = 0; j < cloud->points.size(); ++j) {
            // if point was used as inlier, continue
            if (inliers.count(j) > 0)
                continue;

            // Measure distance between every point and fitted line
            PointT point = cloud->points[j];
            float x4 = point.x;
            float y4 = point.y;
            float z4 = point.z;

            // calc distance point to plane
            float d = std::fabs(A * x4 + B * y4 + C * z4 + D) / std::sqrt(A * A + B * B + C * C);

            // If distance is smaller than threshold count it as inlier
            if (d < distanceThreshold) {
                inliers.insert(j);                
            }
        }

        if (inliers.size() > inliersResult.size())
            inliersResult = inliers;
    }
    
    pcl::PointCloud<PointT>::Ptr cloudInliers{ new pcl::PointCloud<PointT>() };
    pcl::PointCloud<PointT>::Ptr cloudOutliers{ new pcl::PointCloud<PointT>() };

    if (!inliersResult.empty()) {
        for (int i = 0; i < cloud->points.size(); ++i) {                     
            if (inliersResult.count(i)) 
                cloudInliers->points.push_back(cloud->points[i]);            
            else 
                cloudOutliers->points.push_back(cloud->points[i]);    
            
        }
    }

    return std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>(
        cloudOutliers, cloudInliers);
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	//pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // segment the largest planar component from input cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
        std::cout << "Could not estimate a planar model for given dataset" << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance); // 2cm
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    int j = 0;
    for (pcl::PointIndices getIndices : clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
                
        for (int index : getIndices.indices)            
            cloudCluster->points.push_back(cloud->points[index]); 
        
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering_KdTree(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    
    // create tree and initialize with points
    KdTree* tree = new KdTree;    
    std::vector<std::vector<float>> points;
    for (int i = 0; i < cloud->points.size(); ++i) {
        std::vector<float> p{ cloud->points[i].x, cloud->points[i].y, cloud->points[i].z };
        tree->insert(p, i);
        points.push_back(p);
    }
    
    // TODO: Fill out this function to return list of indices for each cluster
    std::vector<std::vector<int>> clusters;
    std::vector<bool> processed(points.size(), false);

    int i = 0;
    while (i < points.size()) {
        if (processed[i])
        {
            i++;
            continue;
        }

        std::vector<int> cluster;
        tree->clusterHelper(i, points, cluster, processed, tree, clusterTolerance);
        clusters.push_back(cluster);

        i++;
    }


    //return clusters;

    std::vector<pcl::PointCloud<PointT>::Ptr> pcl_clusters;
    
    for (auto indices : clusters) {
        // check for cluster_size between min and max
        if (indices.size() < minSize || indices.size() > maxSize)
            continue;

        pcl::PointCloud<PointT>::Ptr cloudCluster{ new pcl::PointCloud<PointT> };

        for (auto i : indices) 
            cloudCluster->points.push_back(cloud->points[i]);

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;        
        pcl_clusters.push_back(cloudCluster);
    }


    return pcl_clusters;
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