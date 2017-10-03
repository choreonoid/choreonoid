#include <iostream>
#include "SSVTreeCollider.h"
#include "DistFuncs.h"

static bool debug = false;

namespace Opcode {

SSVTreeCollider::SSVTreeCollider()
{
        
}
    
bool SSVTreeCollider::Distance(BVTCache& cache, 
                               float& minD, Point &point0, Point&point1,
                               const Matrix4x4* world0, const Matrix4x4* world1)
{
    // Checkings
    if(!cache.Model0 || !cache.Model1)                             return false;
    if(cache.Model0->HasLeafNodes()!=cache.Model1->HasLeafNodes()) return false;
    if(cache.Model0->IsQuantized()!=cache.Model1->IsQuantized())   return false;
        
    // Checkings
    if(!Setup(cache.Model0->GetMeshInterface(), cache.Model1->GetMeshInterface())) return false;
        
    // Simple double-dispatch
    const AABBCollisionTree* T0 = (const AABBCollisionTree*)cache.Model0->GetTree();
    const AABBCollisionTree* T1 = (const AABBCollisionTree*)cache.Model1->GetTree();
    Distance(T0, T1, world0, world1, &cache, minD, point0, point1);
    return true;
}

void SSVTreeCollider::Distance(const AABBCollisionTree* tree0, 
                               const AABBCollisionTree* tree1, 
                               const Matrix4x4* world0, const Matrix4x4* world1, 
                               Pair* cache, float& minD, Point &point0, Point&point1)
{
    if (debug) std::cout << "Distance()" << std::endl;
    // Init collision query
    InitQuery(world0, world1);
    
    // Compute initial value using temporal coherency
    // todo : cache should be used 

    const AABBCollisionNode *n;
    for (unsigned int i=0; i<tree0->GetNbNodes(); i++){
        n = tree0->GetNodes()+i;
        if (n->IsLeaf()){
            mId0 = n->GetPrimitive();
            break;
        }
    } 
    for (unsigned int i=0; i<tree1->GetNbNodes(); i++){
        n = tree1->GetNodes()+i;
        if (n->IsLeaf()){
            mId1 = n->GetPrimitive();
            break;
        }
    } 
    Point p0, p1;
    minD = PrimDist(mId0, mId1, p0, p1);
    
    // Perform distance computation
    _Distance(tree0->GetNodes(), tree1->GetNodes(), minD, p0, p1);

    // transform points
    TransformPoint4x3(point0, p0, *world1);
    TransformPoint4x3(point1, p1, *world1);

    // update cache
    cache->id0 = mId0;
    cache->id1 = mId1;
}

bool SSVTreeCollider::Collide(BVTCache& cache, double tolerance,
                              const Matrix4x4* world0, const Matrix4x4* world1)
{
    // Checkings
    if(!cache.Model0 || !cache.Model1)								return false;
    if(cache.Model0->HasLeafNodes()!=cache.Model1->HasLeafNodes())	return false;
    if(cache.Model0->IsQuantized()!=cache.Model1->IsQuantized())	return false;

    // Checkings
    if(!Setup(cache.Model0->GetMeshInterface(), cache.Model1->GetMeshInterface()))	return false;
    
    // Simple double-dispatch
    const AABBCollisionTree* T0 = (const AABBCollisionTree*)cache.Model0->GetTree();
    const AABBCollisionTree* T1 = (const AABBCollisionTree*)cache.Model1->GetTree();
    return Collide(T0, T1, world0, world1, &cache, tolerance);
}

bool SSVTreeCollider::Collide(const AABBCollisionTree* tree0, 
                              const AABBCollisionTree* tree1, 
                              const Matrix4x4* world0, const Matrix4x4* world1, 
                              Pair* cache, double tolerance)
{
    // Init collision query
    InitQuery(world0, world1);
    
    // todo : cache should be used 

    // Perform collision detection
    if (_Collide(tree0->GetNodes(), tree1->GetNodes(), tolerance)){
        // update cache
        cache->id0 = mId0;
        cache->id1 = mId1;
        return true;
    }

    return false;
}

float SSVTreeCollider::SsvSsvDist(const AABBCollisionNode *b0, 
                                  const AABBCollisionNode *b1)
{
    CollisionAABB::ssv_type t1, t2;
    t1 = b0->mAABB.mType;
    t2 = b1->mAABB.mType;
    if (t1 == CollisionAABB::SSV_PSS && t2 == CollisionAABB::SSV_PSS){
        return PssPssDist(b0->mAABB.mRadius, b0->mAABB.mCenter, 
                          b1->mAABB.mRadius, b1->mAABB.mCenter);
    }else if (t1 == CollisionAABB::SSV_PSS && t2 == CollisionAABB::SSV_LSS){
        return PssLssDist(b0->mAABB.mRadius, b0->mAABB.mCenter,
                          b1->mAABB.mRadius, b1->mAABB.mPoint0, b1->mAABB.mPoint1);
    }else if (t1 == CollisionAABB::SSV_LSS && t2 == CollisionAABB::SSV_PSS){
        return LssPssDist(b0->mAABB.mRadius, b0->mAABB.mPoint0, b0->mAABB.mPoint1,
                          b1->mAABB.mRadius, b1->mAABB.mCenter);
    }else if (t1 == CollisionAABB::SSV_LSS && t2 == CollisionAABB::SSV_LSS){
        return PssPssDist(sqrtf(b0->GetSize()), b0->mAABB.mCenter, 
                          sqrtf(b1->GetSize()), b1->mAABB.mCenter);
    }else{
        std::cerr << "this ssv combination is not supported" << std::endl;
    }
    return 0.0f;
}

float SSVTreeCollider::PssPssDist(float r0, const Point& center0, float r1, const Point& center1)
{
    Point c0;
    TransformPoint(c0, center0, mR0to1, mT0to1);
    return (center1-c0).Magnitude() - r0 - r1;
}

float SSVTreeCollider::PssLssDist(float r0, const Point& center0, float r1, const Point& point0, const Point& point1)
{
    Point c0;
    TransformPoint(c0, center0, mR0to1, mT0to1);
    float d = PointSegDist(c0, point0, point1);
    return d - r0 - r1;
}

float SSVTreeCollider::LssPssDist(float r0, const Point& point0, const Point& point1, float r1, const Point& center0)
{
    Point p0, p1;
    TransformPoint(p0, point0, mR0to1, mT0to1);
    TransformPoint(p1, point1, mR0to1, mT0to1);
    float d = PointSegDist(center0, p0, p1);
    return d - r0 - r1;
}

float SSVTreeCollider::LssLssDist(float r0, const Point& point0, const Point& point1, float r1, const Point& point2, const Point& point3)
{
    Point p0, p1;
    TransformPoint(p0, point0, mR0to1, mT0to1);
    TransformPoint(p1, point1, mR0to1, mT0to1);
    float d = SegSegDist(p0, p1, point2, point3);
    return d - r0 - r1;
}

void SSVTreeCollider::_Distance(const AABBCollisionNode* b0, const AABBCollisionNode* b1,
                                float& minD, Point& point0, Point& point1)
{
    if (debug) std::cout << "_Distance()" << std::endl;

    mNowNode0 = b0;
    mNowNode1 = b1;
    float d;

    // Perform BV-BV distance test
    d = SsvSsvDist(b0, b1);

    if(d > minD) return;
    
    if(b0->IsLeaf() && b1->IsLeaf()) { 
        Point p0, p1;
        d = PrimDist(b0->GetPrimitive(), b1->GetPrimitive(), p0, p1);
        if (d < minD){
            minD = d;
            point0 = p0;
            point1 = p1;
            mId0 = b0->GetPrimitive();
            mId1 = b1->GetPrimitive(); 
        }
        return;
    }
    
    if(b1->IsLeaf() || (!b0->IsLeaf() && (b0->GetSize() > b1->GetSize())))
        {
            _Distance(b0->GetNeg(), b1, minD, point0, point1);
            _Distance(b0->GetPos(), b1, minD, point0, point1);
        }
    else
        {
            _Distance(b0, b1->GetNeg(), minD, point0, point1);
            _Distance(b0, b1->GetPos(), minD, point0, point1);
        }
}

bool SSVTreeCollider::_Collide(const AABBCollisionNode* b0, const AABBCollisionNode* b1,
                               double tolerance)
{
    mNowNode0 = b0;
    mNowNode1 = b1;
    float d;

    // Perform BV-BV distance test
    d = SsvSsvDist(b0, b1);

    if(d > tolerance) return false;
    
    if(b0->IsLeaf() && b1->IsLeaf()) { 
        Point p0, p1;
        d = PrimDist(b0->GetPrimitive(), b1->GetPrimitive(), p0, p1);
        if (d <= tolerance){
            mId0 = b0->GetPrimitive();
            mId1 = b1->GetPrimitive(); 
            return true;
        }else{
            return false;
        }
    }
    
    if(b1->IsLeaf() || (!b0->IsLeaf() && (b0->GetSize() > b1->GetSize())))
        {
            if (_Collide(b0->GetNeg(), b1, tolerance)) return true;
            if (_Collide(b0->GetPos(), b1, tolerance)) return true;
        }
    else
        {
            if (_Collide(b0, b1->GetNeg(), tolerance)) return true;
            if (_Collide(b0, b1->GetPos(), tolerance)) return true;
        }
    return false;
}

float SSVTreeCollider::PrimDist(udword id0, udword id1, Point& point0, Point& point1)
{
    // Request vertices from the app
    VertexPointers VP0;
    VertexPointers VP1;
    mIMesh0->GetTriangle(VP0, id0);
    mIMesh1->GetTriangle(VP1, id1);
    
    // Modified by S-cubed, Inc.
    // Transform from space 0 (old : 1) to space 1 (old : 0)
    // In CD, the conversion is opposite, so it is adjusted accordingly.
    Point u0,u1,u2;
    TransformPoint(u0, *VP0.Vertex[0], mR0to1, mT0to1);
    TransformPoint(u1, *VP0.Vertex[1], mR0to1, mT0to1);
    TransformPoint(u2, *VP0.Vertex[2], mR0to1, mT0to1);

    // Perform triangle-triangle distance test
    return TriTriDist(u0, u1, u2,
                      *VP1.Vertex[0], *VP1.Vertex[1], *VP1.Vertex[2],
                      point0, point1);
}

}
