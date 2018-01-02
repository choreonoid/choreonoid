
#ifndef CNOID_AIST_COLLISION_DETECTOR_DIST_FUNCS_H_INCLUDED
#define CNOID_AIST_COLLISION_DETECTOR_DIST_FUNCS_H_INCLUDED

#include "Opcode/Opcode.h"

namespace Opcode {

/**
 * @brief compute distance between a point and a line segment
 * @param P the point
 * @param u0 one of end points of the line segment
 * @param u1 the other end point of the line segment
 * @return distance between the point and the line segment
 */
float PointSegDist(const Point& P, const Point& u0, const Point& u1);

/**
 * @brief compute distance between line segments
 * @brief u0 one of end points of the first line segment
 * @brief u1 the other end point of the first line segment
 * @brief v0 one of end points of the second line segment
 * @brief v1 the other end point of the second line segment
 * @return distance between line segments
 */
float SegSegDist(const Point& u0, const Point& u1, const Point& v0, const Point& v1);

/**
 * @brief compute the minimum distance and the closest points between two triangles
 * @param U0 the first vertex of the first triangle
 * @param U1 the second vertex of the first triangle
 * @param U2 the third vertex of the first triangle
 * @param V0 the first vertex of the second triangle
 * @param V1 the second vertex of the second triangle
 * @param V2 the third vertex of the second triangle
 * @param cp0 the closest point on the first triangle
 * @param cp1 the closest point on the second triangle
 * @return the minimum distance
 */
float TriTriDist(const Point& U0, const Point& U1, const Point& U2,
                 const Point& V0, const Point& V1, const Point& V2,
                 Point& cp0, Point& cp1);
}

#endif
