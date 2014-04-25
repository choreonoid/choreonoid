#ifndef __SSV_TREE_COLLIDER_H__
#define __SSV_TREE_COLLIDER_H__

#include "Opcode/Opcode.h"

namespace Opcode {

/**
 * @brief collision detector based on SSV(Sphere Swept Volume)
 */
class SSVTreeCollider : public AABBTreeCollider {
public:
    /**
     * @brief constructor
     */
    SSVTreeCollider();

    /**
     * @brief destructor
     */
    ~SSVTreeCollider(){};

    /**
     * @brief compute the minimum distance and the closest points
     * @param cache
     * @param minD the minimum distance
     * @param point0 the closest point on the first link
     * @param point1 the closest point on the second link
     * @param world0 transformation of the first link
     * @param world1 transformation of the second link
     * @return true if computed successfully, false otherwise
     */
    bool Distance(BVTCache& cache, float& minD, Point &point0, Point&point1,
                  const Matrix4x4* world0=null, const Matrix4x4* world1=null);

    /**
     * @brief detect collision between links. 
     * @param cache 
     * @param tolerance If distance between links is smaller than this value, it is regarded as collision
     * @param world0 transformation of the first link
     * @param world1 transformation of the second link
     * @return true if collision is detected, false otherwise
     */
    bool Collide(BVTCache& cache, double tolerance,
                 const Matrix4x4* world0=null, const Matrix4x4* world1=null);

protected:
    /**
     * @brief compute distance between SSV(Swept Sphere Volume)s
     * @param b0 collision node from the left tree
     * @param b1 collision node from the right tree 
     * @param return distance
     */
    float SsvSsvDist(const AABBCollisionNode* b0, const AABBCollisionNode *b1);

    /**
     * @brief compute distance between primitives(triangles)
     * @param id0 index of the first primitive
     * @param id1 index of the second primitive
     * @param point0 the closest point on the first primitive
     * @param point1 the closest point on the second primitive
     * @return the minimum distance
     */
    float PrimDist(udword id0, udword id1, Point& point0, Point& point1);

private:
    void Distance(const AABBCollisionTree* tree0, 
                  const AABBCollisionTree* tree1, 
                  const Matrix4x4* world0, const Matrix4x4* world1, 
                  Pair* cache, float& minD,  Point &point0, Point&point1);

    void _Distance(const AABBCollisionNode* b0, const AABBCollisionNode* b1,
                   float& minD, Point& point0, Point& point1);
    bool Collide(const AABBCollisionTree* tree0, 
                 const AABBCollisionTree* tree1, 
                 const Matrix4x4* world0, const Matrix4x4* world1, 
                 Pair* cache, double tolerance);
    
    bool _Collide(const AABBCollisionNode* b0, const AABBCollisionNode* b1,
                  double tolerance);
    /**
     * @brief compute distance between PSS(Point Swept Sphere)
     * @param r0 radius of the first sphere
     * @param center0 center of the first sphere
     * @param r1 radius of the first sphere
     * @param center1 center of the first sphere
     * @return distance
     */
    float PssPssDist(float r0, const Point& center0, float r1, const Point& center1);

    /**
     * @brief compute distance between PSS(Point Swept Sphere) and LSS(Line Swept Sphere)
     * @param r0 radius of the PSS
     * @param center0 center of the PSS
     * @param r1 radius of the LSS
     * @param point0 one of end points of the line segment
     * @param point1 the other end points of the line segment
     * @return distance
     */
    float PssLssDist(float r0, const Point& center0, 
                     float r1, const Point& point0, const Point& point1);

    /**
     * @brief compute distance between PSS(Point Swept Sphere) and LSS(Line Swept Sphere)
     * @param r0 radius of the PSS
     * @param point0 one of end points of the line segment
     * @param point1 the other end points of the line segment
     * @param r1 radius of the LSS
     * @param center0 center of the PSS
     * @return distance
     */
    float LssPssDist(float r0, const Point& point0, const Point& point1,
                     float r1, const Point& center0);

    /**
     * @brief compute distance between LSS(Line Swept Sphere)s
     * @param r0 radius of the first LSS
     * @param point0 one of end points of the first line segment
     * @param point1 the other end points of the first line segment
     * @param r1 radius of the second LSS
     * @param point2 one of end points of the second line segment
     * @param point3 the other end points of the second line segment
     * @return distance
     */
    float LssLssDist(float r0, const Point& point0, const Point& point1,
                     float r1, const Point& point2, const Point& point3);
};
}

#endif
