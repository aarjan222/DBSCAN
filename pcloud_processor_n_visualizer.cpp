#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <thread>
#include <map>
#include "dbscan.h"

struct Color
{
    float r, g, b;
    Color(float setR, float setG, float setB)
        : r(setR), g(setG), b(setB)
    {
    }
};

pcl::PointCloud<pcl::PointXYZI>::Ptr CreateData3D(std::string file)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());

    if (pcl::io::loadPCDFile<pcl::PointXYZI>(file, *cloud) == -1)
    {
        PCL_ERROR("Couldn't Read File\n");
    }

    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;
    return cloud;
}

// Function to convert pcl::PointCloud<pcl::PointXYZI>::Ptr to std::vector<LidarPoint>
std::vector<Point> convertToLidarPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
{
    std::vector<Point> lidarPoints;
    lidarPoints.reserve(cloud->points.size());

    for (const auto &pclPoint : cloud->points)
    {
        Point lidarPoint;
        lidarPoint.x = static_cast<double>(pclPoint.x);
        lidarPoint.y = static_cast<double>(pclPoint.y);
        lidarPoint.z = static_cast<double>(pclPoint.z);
        // lidarPoint.r = static_cast<double>(pclPoint.intensity); // Intensity as reflectivity

        lidarPoints.push_back(lidarPoint);
    }

    return lidarPoints;
}

// Function to convert std::vector<Point> to pcl::PointCloud<pcl::PointXYZI>::Ptr
void saveClustersToPCD(const std::vector<Point> &points, const std::string &output_dir)
{
    // this maps stores 1 PointCloud for each unique clusterID.
    std::map<int, pcl::PointCloud<pcl::PointXYZI>::Ptr> clusterClouds;

    for (const auto &p : points)
    {
        int clusterID = p.clusterID;

        // Create new point
        pcl::PointXYZI point;
        point.x = p.x;
        point.y = p.y;
        point.z = p.z;
        point.intensity = 1.0f; // default intensity (could encode clusterID here too)

        // If clusterID is not present, create a new cloud
        // if clusterID seen for the first time, create a new empty point cloud for it.3
        if (clusterClouds.find(clusterID) == clusterClouds.end())
        {
            clusterClouds[clusterID] = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
        }

        // Add the point to the corresponding point cloud
        clusterClouds[clusterID]->push_back(point);
    }

    // Save each cluster as a separate .pcd file
    for (const auto &pair : clusterClouds)
    {
        int clusterID = pair.first;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = pair.second;

        // Skip empty clouds
        if (cloud->empty())
            continue;

        std::stringstream ss;
        if (clusterID == -1)
            ss << output_dir << "/noise.pcd";
        else
            ss << output_dir << "/cluster_" << clusterID << ".pcd";

        pcl::io::savePCDFileBinary(ss.str(), *cloud);
    }
}