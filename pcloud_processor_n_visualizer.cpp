#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
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

pcl::visualization::PCLVisualizer::Ptr initScene()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Clustering"));
    viewer->setBackgroundColor(0, 0, 0); // rgb 000= black
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
    viewer->addCoordinateSystem(1.0); // scale=1.0
    return viewer;
}

void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr &viewer, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, std::string name)
{

    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, name);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, name);
}

void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr &viewer, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, std::string name, Color color)
{

    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, name);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, name);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
}

// Function to convert std::vector<LidarPoint> to pcl::PointCloud<pcl::PointXYZI>::Ptr
pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertToPCLCloud(const std::vector<Point> &points)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
   
    // Assign a unique color for each cluster
    std::map<int, std::tuple<uint8_t, uint8_t, uint8_t>> clusterColors;
    int nextColor = 0;

    for (const auto &p : points)
    {
        pcl::PointXYZRGB point;
        point.x = p.x;
        point.y = p.y;
        point.z = p.z;

        int clusterID = p.clusterID;

        if (clusterID == -1)
        {
            // Noise points in gray
            point.r = 128;
            point.g = 128;
            point.b = 128;
        }
        else
        {
            if (clusterColors.find(clusterID) == clusterColors.end())
            {
                // Generate a unique color
                uint8_t r = 50 + (nextColor * 50) % 255;
                uint8_t g = 80 + (nextColor * 80) % 255;
                uint8_t b = 100 + (nextColor * 120) % 255;
                clusterColors[clusterID] = std::make_tuple(r, g, b);
                nextColor++;
            }
            auto [r, g, b] = clusterColors[clusterID];
            point.r = r;
            point.g = g;
            point.b = b;
        }

        cloud->push_back(point);
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    return cloud;
}