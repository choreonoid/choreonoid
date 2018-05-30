/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_AIST_COLLIDION_DETECTOR_COLDET_MODEL_PAIR_H
#define CNOID_AIST_COLLIDION_DETECTOR_COLDET_MODEL_PAIR_H

#include "CollisionData.h"
#include "ColdetModel.h"
#include "CollisionPairInserter.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ColdetModelPair : public Referenced
{
public:
    ColdetModelPair();
    ColdetModelPair(ColdetModel* model0, ColdetModel* model1, double tolerance = 0.0);
    ColdetModelPair(const ColdetModelPair& org);

    virtual ~ColdetModelPair();

    void set(ColdetModel* model0, ColdetModel* model1);

    ColdetModel* model(int index) { return models[index]; }

    std::vector<collision_data>& detectCollisions() {
        return detectCollisionsSub(true);
    }

    std::vector<collision_data>& collisions() {
        return collisionPairInserter->cdContact;
    }

    void clearCollisions(){
        collisionPairInserter->cdContact.clear();
    }

    bool checkCollision() {
        return !detectCollisionsSub(false).empty();
    }

    static double computeDistance(ColdetModel* model0, ColdetModel* model1, double* point0, double* point1);
    double computeDistance(double* point0, double* point1);

    /**
       @param out_triangle0, out_triangle1 Indices of the triangle pair that are originally registered by ColdeModel::setTraiangle().
       @param out_point0, out_point1 The closest points 
    */
    double computeDistance(int& out_triangle0, double* out_point0, int& out_triangle1, double* out_point1);

    bool detectIntersection();

    double tolerance() const { return tolerance_; }

    void setTolerance(double tolerance){
        tolerance_ = tolerance;
    }

    void setCollisionPairInserter(Opcode::CollisionPairInserter *inserter); 

    int calculateCentroidIntersection(float &cx, float &cy, float &A, float radius, std::vector<float> vx, std::vector<float> vy);
		
    int makeCCW(std::vector<float> &vx, std::vector<float> &vy);
		
    float calculatePolygonArea(const std::vector<float> &vx, const std::vector<float> &vy);
    void calculateSectorCentroid(float &cx, float &cy, float radius, float th1, float th2);

    inline bool isInsideCircle(float r, float x, float y) {
        return sqrt(pow(x, 2) + pow(y, 2)) <= r;
    }
    bool isInsideTriangle(float x, float y, const std::vector<float> &vx, const std::vector<float> &vy);

    int calculateIntersection(std::vector<float> &x, std::vector<float> &y, float radius, float x1, float y1, float x2, float y2);

private:
        
    std::vector<collision_data>& detectCollisionsSub(bool detectAllContacts);
    bool detectMeshMeshCollisions(bool detectAllContacts);
    bool detectSphereSphereCollisions(bool detectAllContacts);
    bool detectSphereMeshCollisions(bool detectAllContacts);
    bool detectPlaneCylinderCollisions(bool detectAllContacts);
    bool detectPlaneMeshCollisions(bool detectAllContacts);

    ColdetModelPtr models[2];
    double tolerance_;
    Opcode::CollisionPairInserter* collisionPairInserter;
    int boxTestsCount;
    int triTestsCount;
};

typedef ref_ptr<ColdetModelPair> ColdetModelPairPtr;

}

#endif
