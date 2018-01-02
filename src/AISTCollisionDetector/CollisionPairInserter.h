
#ifndef CNOID_AIST_COLLISION_DETECTOR_COLLISION_PAIR_INSERTER_H
#define CNOID_AIST_COLLISION_DETECTOR_COLLISION_PAIR_INSERTER_H

#include "CollisionData.h"
#include <vector>

namespace cnoid {
class ColdetModelInternalModel;
}

namespace Opcode {

class AABBCollisionNode;
class MeshInterface;

class CollisionPairInserter
{
public:
    virtual ~CollisionPairInserter(){}
    /**
       @brief clear collision information
    */
    void clear(){
        cdContact.clear();
    }

    /**
       @brief detect collsiion between triangles
       @param P1 the first vertex of the first triangle
       @param P2 the second vertex of the first triangle
       @param P3 the third vertex of the first triangle
       @param Q1 the first vertex of the second triangle
       @param Q2 the second vertex of the second triangle
       @param Q3 the third vertex of the second triangle
       @param col_p collision information
       @return 1 if collision is detected, 0 otherwise
       @note all vertices must be represented in the same coordinates
    */
    virtual int detectTriTriOverlap(
        const cnoid::Vector3& P1,
        const cnoid::Vector3& P2,
        const cnoid::Vector3& P3,
        const cnoid::Vector3& Q1,
        const cnoid::Vector3& Q2,
        const cnoid::Vector3& Q3,
        cnoid::collision_data* col_p)=0;

    /**
       @brief refine collision information using neighboring triangls
       @param b1 node of the first colliding triangle
       @param b2 node of the second colliding triangle
       @param id1 id of the first colliding triangle
       @param id2 id of the second colliding triangle
       @param num_of_i_points the number of intersecting points
       @param i_points intersecting points
       @param n_vector normal vector of collision
       @param depth penetration depth
       @param n1 normal vector of the first triangle
       @param m1 normal vector of the second triangle
       @param ctype collision type
       @param mesh1 mesh which includes the first triangle
       @param mesh2 mesh which includes the second triangle
       @return CD_OK if refined successfully
       @note collision information is expressed in the second mesh coordinates
    */
    virtual int apply(const Opcode::AABBCollisionNode* b1,
                      const Opcode::AABBCollisionNode* b2,
                      int id1, int id2,
                      int num_of_i_points,
                      cnoid::Vector3 i_points[4],
                      cnoid::Vector3& n_vector,
                      double depth,
                      cnoid::Vector3& n1,
                      cnoid::Vector3& m1,
                      int ctype,
                      Opcode::MeshInterface* mesh1,
                      Opcode::MeshInterface* mesh2)=0;

    /**
       @brief get collision information
       @return collision information
    */
    std::vector<cnoid::collision_data>& collisions() {
        return cdContact;
    }

    void set(cnoid::ColdetModelInternalModel* model0, 
             cnoid::ColdetModelInternalModel* model1){
        models[0] = model0;
        models[1] = model1;
    }

    cnoid::Matrix3 CD_Rot1;	///< rotation of the first mesh
    cnoid::Vector3 CD_Trans1;	///< translation of the first mesh
    double CD_s1;		///< scale of the first mesh

    cnoid::Matrix3 CD_Rot2;	///< rotation of the second mesh
    cnoid::Vector3 CD_Trans2;	///< translation of the second mesh
    double CD_s2;		///< scale of the second mesh

    std::vector<cnoid::collision_data> cdContact; ///< collision information
        
    cnoid::ColdetModelInternalModel* models[2];
};

}

#endif
