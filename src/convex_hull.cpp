// Implementation of Andrew's monotone chain 2D convex hull algorithm.
// Asymptotic complexity: O(n log n).
// Practical performance: 0.5-1.0 seconds for n=1000000 on a 1GHz machine.
#include <algorithm>
#include <vector>
#include<iostream>
#include<cmath>
#include <kdl/frames.hpp>

#include "convex_hull.h"

using namespace std;
using namespace planner;

Point::Point(double x, double y)
{
    this->x = x;
    this->y = y;
}

KDL::Vector Point::to_KDL_vector()
{
    KDL::Vector vec;
    vec.x(x);
    vec.y(y);
    vec.z(0);
    return vec;
}

double Point::CrossProduct(const Point &a, const Point &b)
{
    return a.x * b.y - a.y * b.x;
}

double Point::cross(const Point &O, const Point &A, const Point &B)
{
    return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}

Point Point::operator + (const Point &other) const
{
    return Point(this->x + other.x, this->y + other.y);
}

Point Point::operator - (const Point &other) const
{
    return Point(this->x - other.x, this->y - other.y);
}

bool Point::operator <(const Point &p) const {
    return x < p.x || (x == p.x && y < p.y);
}

struct Zgreater
{
    bool operator()( const KDL::Vector& lx, const KDL::Vector& rx ) const {
        return lx.z() < rx.z();
    }
};

vector< KDL::Vector > convex_hull::projection(vector< KDL::Vector > IMULink_points3D, KDL::Frame IMULink_GravityWorld)
{
    double err=0;
    std::vector<KDL::Vector> result;
    for (KDL::Vector& IMULink_point:IMULink_points3D)
    {
        auto temp=IMULink_GravityWorld.Inverse()*IMULink_point;
//         KDL::Vector p;
//         p.x=temp.x();
//         p.y=temp.y();
//         p.z=temp.z();
        err+=temp.z();
        result.push_back(temp);
    }
    std::cout<<"global error with respect to the world plane: "<<err<<std::endl;
    return result;
}

std::vector<KDL::Vector> convex_hull::smartZfilter(vector<KDL::Vector> GravityWorld_points3D, double tolerance)
{
    assert(GravityWorld_points3D.size());
    vector<KDL::Vector> results;
    std::sort(GravityWorld_points3D.begin(),GravityWorld_points3D.end(),Zgreater());
    double startingZ=GravityWorld_points3D.front().z();
    for (auto point:GravityWorld_points3D)
    {
        if (point.z()<startingZ+tolerance)
            results.push_back(point);
        else
            break;
    }
    return results;
}

vector<Point> convex_hull::compute(vector<KDL::Vector> P)
{
    std::vector<Point> temp;
    for (auto point:P)
    {
        Point a;
        a.x=point.x();
        a.y=point.y();
        temp.push_back(a);
    }
    return compute(temp);
}

vector<Point> convex_hull::compute(vector<Point> P)
{
    int n = P.size(), k = 0;
    vector<Point> H(2*n);

    // Sort points lexicographically
    sort(P.begin(), P.end());

    // Build lower hull
    for (int i = 0; i < n; ++i) {
        while (k >= 2 && Point::cross(H[k-2], H[k-1], P[i]) <= 0) k--;
        H[k++] = P[i];
    }

    // Build upper hull
    for (int i = n-2, t = k+1; i >= 0; i--) {
        while (k >= t && Point::cross(H[k-2], H[k-1], P[i]) <= 0) k--;
        H[k++] = P[i];
    }

    H.resize(k);
    return H;
}

bool convex_hull::is_point_inside(std::vector< KDL::Vector > CH, Point P)
{
    std::vector<Point> temp;
    for (auto point:CH)
    {
        Point a;
        a.x=point.x();
        a.y=point.y();
        temp.push_back(a);
    }
    return is_point_inside(temp,P);
}

bool convex_hull::is_point_inside(std::vector< Point > CH, Point P)
{
    double minX,minY;
    double maxX,maxY;
    maxX=maxY=-100000;
    minX=minY=100000;
    for(auto p:CH)
    {
	minX = std::min(minX,p.x);
	maxX = std::max(maxX,p.x);
	minY = std::min(minY,p.y);
	maxY = std::max(maxY,p.y);
    }

    if (P.x < minX || P.x > maxX || P.y < minY || P.y > maxY) return false;
    
    // Simple code
//     int i, j;
//     bool result = false;
//     for (i = 0, j = CH.size()-1; i < CH.size(); j = i++) //PNPoly
//     {
// 	if ( ((CH.at(i).y>P.y) != (CH.at(j).y>P.y)) && (P.x < (CH.at(j).x-CH.at(i).x) * (P.y-CH.at(i).y) / (CH.at(j).y-CH.at(i).y) + CH.at(i).x) ) result = !result;
//     }
    
    // Implementation by Lascha Lagidse
    int i=0;
    int j=CH.size()-1;
    bool  result=false;
    
    for (i=0; i<CH.size(); i++)
    {
	if ((CH.at(i).y< P.y && CH.at(j).y>=P.y || CH.at(j).y< P.y && CH.at(i).y>=P.y) &&  (CH.at(i).x<=P.x || CH.at(j).x<=P.x))
	{
	    result^=(CH.at(i).x+(P.y-CH.at(i).y)/(CH.at(j).y-CH.at(i).y)*(CH.at(j).x-CH.at(i).x)<P.x);
	}
	j=i; 
    }
    
    return result;
}