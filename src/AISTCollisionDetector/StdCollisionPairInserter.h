
#ifndef CNOID_AIST_COLLISION_DETECTOR_STD_COLLISION_PAIR_INSERTER_H
#define CNOID_AIST_COLLISION_DETECTOR_STD_COLLISION_PAIR_INSERTER_H

#include "CollisionPairInserter.h"

namespace Opcode {

class StdCollisionPairInserter : public CollisionPairInserter
{
public:
    StdCollisionPairInserter();
    virtual ~StdCollisionPairInserter();
    virtual int detectTriTriOverlap(
        const cnoid::Vector3& P1,
        const cnoid::Vector3& P2,
        const cnoid::Vector3& P3,
        const cnoid::Vector3& Q1,
        const cnoid::Vector3& Q2,
        const cnoid::Vector3& Q3,
        cnoid::collision_data* col_p);

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
                      Opcode::MeshInterface* mesh2);

private:

    class tri
    {
    public:
        int id;
        cnoid::Vector3 p1, p2, p3;
    };
        
    class col_tri
    {
    public:
        int status; // 0: unvisited, 1: visited, 2: included in the convex neighbor 
        cnoid::Vector3 p1, p2, p3;
        cnoid::Vector3 n;
    };

    static void copy_tri(col_tri* t1, tri* t2);
        
    static void copy_tri(col_tri* t1, col_tri* t2);
        
    static void calc_normal_vector(col_tri* t);
        
    static int is_convex_neighbor(col_tri* t1, col_tri* t2);
        
    void triangleIndexToPoint(cnoid::ColdetModelInternalModel* model, int id, col_tri& tri);
        
    int get_triangles_in_convex_neighbor(cnoid::ColdetModelInternalModel* model, int id, col_tri* tri_convex_neighbor, int max_num);
        
    void get_triangles_in_convex_neighbor(cnoid::ColdetModelInternalModel* model, int id, col_tri* tri_convex_neighbor, std::vector<int>& map, int& count);
        
    void examine_normal_vector(int id1, int id2, int ctype);

    void check_separability(int id1, int id2, int ctype);

    void find_signed_distance(
        cnoid::Vector3 &signed_distance, col_tri *trp, int nth, int ctype, int obj);

    void find_signed_distance(
        cnoid::Vector3& signed_distance, const cnoid::Vector3& vert, int nth, int ctype, int obj);

    void find_signed_distance(cnoid::Vector3& signed_distance1, cnoid::ColdetModelInternalModel* model0, int id1, int contactIndex, int ctype, int obj);

    int new_point_test(int k);
};

}

#endif
