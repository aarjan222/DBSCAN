#include "dbscan.h"

// Iterates through all points. If a point is unclassified, it tries to expand a 
// cluster from that point.
int DBSCAN::run()
{
    int clusterID = 1;
    vector<Point>::iterator iter;
    for(iter = m_points.begin(); iter != m_points.end(); ++iter)
    {
        if ( iter->clusterID == UNCLASSIFIED )
        {
            if ( expandCluster(*iter, clusterID) != FAILURE )
            {
                clusterID += 1;
            }
        }
    }

    return 0;
}

// Calculates neighbors (within epsilon radius).
// If enough neighbors are found (≥ minPoints), a new cluster is formed.
// Then it tries to expand the cluster by checking neighbors of neighbors.
/*
This is a recursive-like cluster expansion loop:
    Start from the initial core point.
    Add its neighbors.
    Then, for any neighbor that is itself a core point, repeat the process.
    This builds a full cluster by reaching all density-connected points.
*/
int DBSCAN::expandCluster(Point point, int clusterID)
{    
    vector<int> clusterSeeds = calculateCluster(point);

    if ( clusterSeeds.size() < m_minPoints )
    {
        point.clusterID = NOISE;
        return FAILURE;
    }
    else
    {
        // this can be either core point or border point but not noise points
        int index = 0, indexCorePoint = 0;
        vector<int>::iterator iterSeeds;
        /*
        gothrough all indices in clusterSeeds(these are neighbor of the core points)
        for each point in m_points, you assign it the current clusterID
        also detect which is the original core point(by comparing coordinates)
        indexCorePoint holds the pos of that core point in the clusterSeeds vector.
        */
        for( iterSeeds = clusterSeeds.begin(); iterSeeds != clusterSeeds.end(); ++iterSeeds)
        {
            m_points.at(*iterSeeds).clusterID = clusterID;
            if (m_points.at(*iterSeeds).x == point.x && m_points.at(*iterSeeds).y == point.y && m_points.at(*iterSeeds).z == point.z )
            {
                indexCorePoint = index;
            }
            ++index;
        }
        /*
        remove the core point from clusterSeeds
        core point does not need to be processed again in the next loop
        only it neighbors are processed for further cluster expansion.
        */
        clusterSeeds.erase(clusterSeeds.begin()+indexCorePoint);
        
        /*
        iterate over the rest of the seeds (neighbors)
        */
       
       // for each of the neighbor clusterSeeds[i] do the following:
       for( vector<int>::size_type i = 0, n = clusterSeeds.size(); i < n; ++i )
       {
            // get neighbors of this neighbor,
            // you get all points within epsilon distance of this neighbor -- i.e its neighbors.
            vector<int> clusterNeighors = calculateCluster(m_points.at(clusterSeeds[i]));

            // this means that this is also a core point.
            // so it can also grow the cluster.
            if ( clusterNeighors.size() >= m_minPoints )
            {
                vector<int>::iterator iterNeighors;
                // check its neighbor and assign clusteriD.
                for ( iterNeighors = clusterNeighors.begin(); iterNeighors != clusterNeighors.end(); ++iterNeighors )
                {
                    // If it's not yet classified or marked as noise, we consider it as part of the cluster.
                    if ( m_points.at(*iterNeighors).clusterID == UNCLASSIFIED || m_points.at(*iterNeighors).clusterID == NOISE )
                    {
                        // If it's UNCLASSIFIED, we add it to clusterSeeds so that it too can be 
                        // expanded later.
                        if ( m_points.at(*iterNeighors).clusterID == UNCLASSIFIED )
                        {
                            clusterSeeds.push_back(*iterNeighors);
                            n = clusterSeeds.size();
                        }
                        m_points.at(*iterNeighors).clusterID = clusterID;
                    }
                }
            }
        }

        return SUCCESS;
    }
}

// Returns indices of all points within epsilon distance from a given point
vector<int> DBSCAN::calculateCluster(Point point)
{
    int index = 0;
    vector<Point>::iterator iter;
    vector<int> clusterIndex;
    for( iter = m_points.begin(); iter != m_points.end(); ++iter)
    {
        if ( calculateDistance(point, *iter) <= m_epsilon )
        {
            clusterIndex.push_back(index);
        }
        index++;
    }
    return clusterIndex;
}

// Returns squared Euclidean distance between two 3D points (saves computation of sqrt).
inline double DBSCAN::calculateDistance(const Point& pointCore, const Point& pointTarget )
{
    return pow(pointCore.x - pointTarget.x,2)+pow(pointCore.y - pointTarget.y,2)+pow(pointCore.z - pointTarget.z,2);
}


