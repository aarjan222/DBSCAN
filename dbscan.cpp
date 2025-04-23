#include "dbscan.h"
#include <chrono>
#include <iostream>

// Iterates through all points. If a point is unclassified, it tries to expand a 
// cluster from that point.
int DBSCAN::run()
{
    auto startTime = std::chrono::steady_clock::now();

    int clusterID = 1;
    vector<Point>::iterator iter;
    for(iter = m_points.begin(); iter != m_points.end(); ++iter)
    {
        if ( iter->clusterID == UNCLASSIFIED )//for all of the unclustered core points
        {
            if ( expandCluster(*iter, clusterID) != FAILURE )//identify all points as either core, border or noise
            {
                clusterID += 1;
            }
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime-startTime);
    std::cout << "DBSCAN took " << elapsedTime.count() << " milliseconds" << std::endl;

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
    // clusterSeeds is the vector which contains all the border points for that point within the eps
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
        for each point in m_points, you assign it the current clusterID``
        also detect which is the original core point(by comparing coordinates)
        indexCorePoint holds the pos of that core point in the clusterSeeds vector.
        */
        for( iterSeeds = clusterSeeds.begin(); iterSeeds != clusterSeeds.end(); ++iterSeeds)
        {
            // assign all the border points the same current clusterID
            m_points.at(*iterSeeds).clusterID = clusterID;
            if (m_points.at(*iterSeeds).x == point.x && m_points.at(*iterSeeds).y == point.y && m_points.at(*iterSeeds).z == point.z )
            {
                indexCorePoint = index;
            }
            ++index;// find the index of the corePoint
        }
        /*
        remove the corePoint from clusterSeeds
        core point does not need to be processed again in the next loop
        only its border points are processed for further cluster expansion.
        */
        clusterSeeds.erase(clusterSeeds.begin()+indexCorePoint);
        
        /*
        iterate over the rest of the seeds (neighbors) border points
        */
       
       // for each of the neighbor(border points) clusterSeeds[i] do the following:
       for( vector<int>::size_type i = 0, n = clusterSeeds.size(); i < n; ++i )
       {
            // get neighbors of this neighbor,
            // you get all points within epsilon distance of this neighbor -- i.e its neighbors.
            vector<int> clusterNeighors = calculateCluster(m_points.at(clusterSeeds[i]));
            // this is the list of border points of the previous border points

            // this means that this is also a core point.
            // so it can also grow the cluster.
            // check if this border points has enough minPts to become a core point.
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


/* 
* TimeComplexity

run () iterates over all points: O(N)
for each unclassified point, it calls expandCluster()

expandCluster()
calls calculateCluster()
    for each point, it computes distance to every other point O(N)
then iterates over neighbors and if any core points, 
    recursively find their neighbors.

worst-case: every point is a neighbor to every other point(dense dataset)
so in the worst-case: expandCluster() could do O(N^2) total work(each point checking all others.)

Even though you do nested loops, you're not doing N * N * N work always.
at worst youre doing sth like O(N) expandCluster calls, each of which may do upto O(N) neighbor
checks and each neighbor check is O(N) -- but in practice, you dont reprocess already labeled.
you avoid recomputing for points already added to a cluster.

calculateCluster()
O(N) as it loops over all N points.

Hence
WORST TIME COMPLEXITY:
O(N^2)

*/