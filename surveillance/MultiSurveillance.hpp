#ifndef MULTISURVEILLANCE_HPP
#define MULTISURVEILLANCE_HPP

#include "GraphVertex.hpp"

class MultiSurveillance
{
public:

    MultiSurveillance(int M, int N, vector<pair<int,int>> starts, vector<pair<int,int>> goals);
    ~MultiSurveillance();

    vector<GraphVertex*> GetSuccessorsTop(GraphVertex* vertex);
    GraphVertex* TopSearch();

    vector<GraphVertex*> GetSuccessorsMid(GraphVertex* vertex, GraphVertex* end);
    GraphVertex* GetMidVertex(GraphVertex* goal, int robot);
    double MidSearch(GraphVertex* goal, int robot);

    vector<GraphVertex*> GetSuccessorsLow(GraphVertex* vertex);
    double LowSearch(GraphVertex* start, GraphVertex* goal, int robot);

    int m_numWayPts;
    int m_numRobots;
    vector<pair<int,int>> m_starts;
    vector<pair<int,int>> m_goals;


};

#endif