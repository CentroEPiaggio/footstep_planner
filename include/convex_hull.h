#ifndef FS_CONVEX_HULL_H
#define FS_CONVEX_HULL_H

#include <vector>
#include <kdl/frames.hpp>

namespace planner
{
    
struct Point {
    double x, y;
    
    Point(double x = 0, double y = 0);

    KDL::Vector to_KDL_vector();
    
    static double CrossProduct(const Point &a, const Point &b);
    
    // 2D cross product of OA and OB vectors, i.e. z-component of their 3D cross product.
    // Returns a positive value, if OAB makes a counter-clockwise turn,
    // negative for clockwise turn, and zero if the points are collinear.
    static double cross(const Point &O, const Point &A, const Point &B);
    
    Point operator + (const Point &other) const;
    
    Point operator - (const Point &other) const;
    
    bool operator <(const Point &p) const;
};

class convex_hull
{

public:
    // Returns a list of points on the convex hull in counter-clockwise order.
    // Note: the last point in the returned list is the same as the first one.
    std::vector<KDL::Vector> smartZfilter(std::vector<KDL::Vector> GravityWorld_points3D, double tolerance=0.05);
    std::vector<Point> compute(std::vector<Point> P);
    std::vector<Point> compute(std::vector<KDL::Vector> P);
    std::vector<KDL::Vector> projection(std::vector< KDL::Vector > IMULink_points3D, KDL::Frame IMULink_GravityWorld);
    bool is_point_inside(std::vector<Point> CH,Point P);
    bool is_point_inside(std::vector<KDL::Vector> CH,Point P);

private:



};

}

#endif //FS_CONVEX_HULL_H