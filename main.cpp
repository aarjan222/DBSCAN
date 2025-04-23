#include <stdio.h>
#include <iostream>
#include "dbscan.h"
#include "pcloud_processor_n_visualizer.cpp"

#define MINIMUM_POINTS 10    // minimum number of cluster
#define EPSILON (0.45*0.45) // distance for clustering, metre^2

void printResults(vector<Point> &points, int num_points)
{
    int i = 0;
    printf("Number of points: %u\n"
           " x     y     z     cluster_id size\n"
           "-----------------------------\n",
           num_points);
    while (i < num_points)
    {
        printf("%5.2lf %5.2lf %5.2lf: %d %ld\n",
               points[i].x,
               points[i].y, points[i].z,
               points[i].clusterID, points.size());
        ++i;
    }
}

int main()
{
    vector<Point> points;

    // std::string file = "../data/kitti_data_filtered.pcd";
    std::string file = "../data/logictronix_env_2_persons_2_filtered.pcd";

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = CreateData3D(file);
    points = convertToLidarPoints(cloud);

    for(size_t i = 0; i< points.size();i++){
        points[i].clusterID = UNCLASSIFIED;
    }

    // constructor
    DBSCAN ds(MINIMUM_POINTS, EPSILON, points);

    // main loop
    ds.run();

    // result of DBSCAN algorithm
    // printResults(ds.m_points, ds.getTotalPointSize());

    // saveClustersToPCD(ds.m_points, "../kitti_data_clustered_output/");
    saveClustersToPCD(ds.m_points, "../logictronix_env_clustered_output/");

    return 0;
}

/*
kitti
4 0.75 0.75 299 clusters took 20mins DBSCAN took 1213559 milliseconds for 123397 points.
4 0.75 0.75 299 clusters took 11mins DBSCAN took 708773 milliseconds for 123397 points.
50 0.75 0.75 54 clusters took 4mins DBSCAN took 208254 milliseconds for 67109 points.
20 0.2 0.2 66 clusters took 6.75mins DBSCAN took 405243 milliseconds
10 0.45 0.45 312 clusters took 6.83mins DBSCAN took 410177 milliseconds


logictronix_env
4 0.75 0.75 42 clusters took 7456ms



*/