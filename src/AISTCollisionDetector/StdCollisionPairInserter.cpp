
#include "StdCollisionPairInserter.h"
#include "ColdetModelInternalModel.h"
#include "Opcode/Opcode.h"
#include <cstdio>
#include <iostream>

using namespace std;
using namespace Opcode;
using namespace cnoid;

namespace Opcode {

int tri_tri_overlap(
    const Vector3& P1,
    const Vector3& P2,
    const Vector3& P3,
    const Vector3& Q1,
    const Vector3& Q2,
    const Vector3& Q3,
    collision_data* col_p,
    CollisionPairInserter* collisionPairInserter);
}

namespace {
const bool COLLIDE_DEBUG = false;
// if DEPTH_CHECK is defined in the compile, contact point selection using depth value is enabled
#ifdef  DEPTH_CHECK
const double MAX_DEPTH = 0.1;
#endif
const int CD_OK = 0;
const int CD_ALL_CONTACTS = 1;
const int CD_FIRST_CONTACT = 2;
const int CD_ERR_COLLIDE_OUT_OF_MEMORY = 2;
    
enum {
    FV = 1,
    VF,
    EE
};
}


StdCollisionPairInserter::StdCollisionPairInserter()
{

}


StdCollisionPairInserter::~StdCollisionPairInserter()
{

}


void StdCollisionPairInserter::copy_tri(col_tri* t1, tri* t2)
{
    t1->p1 = t2->p1;
    t1->p2 = t2->p2;
    t1->p3 = t2->p3;
}


void StdCollisionPairInserter::copy_tri(col_tri* t1, col_tri* t2)
{
    t1->p1 = t2->p1;
    t1->p2 = t2->p2;
    t1->p3 = t2->p3;

    if(t2->n[0] && t2->n[1] && t2->n[2]){
        t1->n = t2->n;
    }
}


void StdCollisionPairInserter::calc_normal_vector(col_tri* t)
{
    if(t->status == 0){
        const Vector3 e1(t->p2 - t->p1);
        const Vector3 e2(t->p3 - t->p2);
        t->n = e1.cross(e2).normalized();
        t->status = 1;
    }
}


int StdCollisionPairInserter::is_convex_neighbor(col_tri* t1, col_tri* t2)
{
    const double EPS = 1.0e-12; // a small number
        
    calc_normal_vector(t2);
        
    // printf("normal vector1 = %f %f %f\n", t1->n[0], t1->n[1], t1->n[2]);
    // printf("normal vector2 = %f %f %f\n", t2->n[0], t2->n[1], t2->n[2]);
        
    const Vector3 vec1(t1->p1 - t2->p1);
    const Vector3 vec2(t1->p2 - t2->p2);
    const Vector3 vec3(t1->p3 - t2->p3);
        
    // printf("is_convex_neighbor = %f %f %f\n",innerProd(t1->n,vec1),innerProd(t1->n,vec2),innerProd(t1->n,vec3));
        
    if(t2->n.dot(vec1) < EPS && t2->n.dot(vec2) < EPS && t2->n.dot(vec3) < EPS){
        return 1;
    } else {
        return 0;
    }
}

void StdCollisionPairInserter::triangleIndexToPoint(ColdetModelInternalModel* model, int id, col_tri& tri){
    IceMaths::IndexedTriangle indextriangle = model->triangles[id];
    IceMaths::Point point0 = model->vertices[indextriangle.mVRef[0]];
    IceMaths::Point point1 = model->vertices[indextriangle.mVRef[1]];
    IceMaths::Point point2 = model->vertices[indextriangle.mVRef[2]];
    tri.p1[0] = point0.x; tri.p1[1] = point0.y; tri.p1[2] = point0.z; 
    tri.p2[0] = point1.x; tri.p2[1] = point1.y; tri.p2[2] = point1.z; 
    tri.p3[0] = point2.x; tri.p3[1] = point2.y; tri.p3[2] = point2.z; 
}


void StdCollisionPairInserter::get_triangles_in_convex_neighbor
(ColdetModelInternalModel* model, int id, col_tri* tri_convex_neighbor, std::vector<int>& foundTriangles, int& count)
{
    int k;
    for(int i=0; i<foundTriangles.size(); i++)
        if(foundTriangles[i] == id){
            k = i;
            break;
        }

    for(int i=0; i<3; i++){
        int nei = model->neighbors[id][i];
        if(nei < 0)
            continue;
        int j=0;
        for(; j<foundTriangles.size(); j++)
            if(foundTriangles[j] == nei)
                break;
        if(j<foundTriangles.size())
            continue;

        col_tri tri_nei;
        triangleIndexToPoint(model, nei, tri_nei); 

        if(is_convex_neighbor( &tri_nei, &tri_convex_neighbor[k])){
            if(k!=0){
                Vector3 p1 = tri_nei.p1 - tri_convex_neighbor[0].p1;
                if(p1.dot(tri_convex_neighbor[0].n) > 0)
                    continue;
                Vector3 p2 = tri_nei.p2 - tri_convex_neighbor[0].p1;
                if(p2.dot(tri_convex_neighbor[0].n) > 0)
                    continue;
                Vector3 p3 = tri_nei.p3 - tri_convex_neighbor[0].p1;
                if(p3.dot(tri_convex_neighbor[0].n) > 0)
                    continue;
            }
            foundTriangles.push_back(nei);
            tri_convex_neighbor[count].status = 0;
            copy_tri(&tri_convex_neighbor[count++], &tri_nei);
        }
    }

    if(COLLIDE_DEBUG) {
        cout << "id= " << id;
        for(int i=0; i<foundTriangles.size(); i++ )
            cout << " " << foundTriangles[i];
        cout << endl;
    }
}

int StdCollisionPairInserter::get_triangles_in_convex_neighbor
(ColdetModelInternalModel* model, int id, col_tri* tri_convex_neighbor, int min_num)
{
    std::vector<int> foundTriangles;
    int count=0;
    triangleIndexToPoint(model, id, tri_convex_neighbor[count++]);
    tri_convex_neighbor[0].status = 0;
    foundTriangles.push_back(id);
    
    int start = 0;
    int end = 1;

    int j=0;
    while(count < min_num && j<2){
        for(int i=start; i< end; i++)
            get_triangles_in_convex_neighbor(model, foundTriangles[i], tri_convex_neighbor, foundTriangles, count);
        start = end;
        end  = count;
        j++;
    }

    return count;
}


void StdCollisionPairInserter::examine_normal_vector(int id1, int id2, int ctype)
{
    check_separability(id1, id2, ctype);
}


void StdCollisionPairInserter::check_separability(int id1, int id2, int ctype)
{
    int contactIndex = cdContact.size() - 1;
    Vector3 signed_distance;
    Vector3 signed_distance1(99999999.0,99999999.0,99999999.0);
    Vector3 signed_distance2(-99999999.0,-99999999.0,-99999999.0);

    find_signed_distance(signed_distance1, models[0], id1, contactIndex, ctype, 1);
    find_signed_distance(signed_distance2, models[1], id2, contactIndex, ctype, 2);

    int max = (2 < ctype) ? ctype : 2;
    
    for(int i=0; i < max; ++i){
        signed_distance[i] = signed_distance1[i] - signed_distance2[i];
        if(COLLIDE_DEBUG) printf("signed distance %d = %f\n", i, signed_distance[i]);
    }

    if(COLLIDE_DEBUG){
        printf("origin normal = %f %f %f\n", cdContact[contactIndex].n_vector[0],
                cdContact[contactIndex].n_vector[1], cdContact[contactIndex].n_vector[2]);
        cout << "origin depth = " << cdContact[contactIndex].depth << endl;
    }

    switch(ctype){

    case FV:
        if(signed_distance[0] < signed_distance[1]){
            cdContact[contactIndex].n_vector = cdContact[contactIndex].m;
            cdContact[contactIndex].depth = fabs(signed_distance[1]);
            if(COLLIDE_DEBUG) printf("normal replaced\n");
        } else {
            cdContact[contactIndex].depth = fabs(signed_distance[0]);
        }
        break;
        
    case VF:
        if(signed_distance[0] < signed_distance[1]){
            cdContact[contactIndex].n_vector = - cdContact[contactIndex].n;
            cdContact[contactIndex].depth = fabs(signed_distance[1]);
            if(COLLIDE_DEBUG) printf("normal replaced\n");
        } else{
            cdContact[contactIndex].depth = fabs(signed_distance[0]);
        }
        break;
        
    case EE:
        cdContact[contactIndex].num_of_i_points = 1;
        if(signed_distance[0] < signed_distance[1] && signed_distance[2] <= signed_distance[1]){
            cdContact[contactIndex].n_vector = cdContact[contactIndex].m;
            cdContact[contactIndex].depth = fabs(signed_distance[1]);
            if(COLLIDE_DEBUG) printf("normal replaced\n");
        } else if(signed_distance[0] < signed_distance[2] && signed_distance[1] < signed_distance[2]){
            cdContact[contactIndex].n_vector = - cdContact[contactIndex].n;
            cdContact[contactIndex].depth = fabs(signed_distance[2]);
            if(COLLIDE_DEBUG) printf("normal replaced\n");
        } else {
            cdContact[contactIndex].depth = fabs(signed_distance[0]);
            // cout << "depth in InsertCollisionPair.cpp = " << signed_distance[0] << endl;
        }
        cdContact[contactIndex].i_points[0] += cdContact[contactIndex].i_points[1];
        cdContact[contactIndex].i_points[0] *= 0.5;
        break;
    }
    
    if(COLLIDE_DEBUG){
        printf("final normal = %f %f %f\n", cdContact[contactIndex].n_vector[0],
               cdContact[contactIndex].n_vector[1], cdContact[contactIndex].n_vector[2]);
    }
    if(COLLIDE_DEBUG){
        for(int i=0; i < cdContact[contactIndex].num_of_i_points; ++i){
            cout << "final depth = " << cdContact[contactIndex].depth << endl;
            cout << "final i_point = " << cdContact[contactIndex].i_points[i][0] << " "
                 << cdContact[contactIndex].i_points[i][1] << " " << cdContact[contactIndex].i_points[i][2]
                 << endl;
        }
    }
    
    if(COLLIDE_DEBUG) cout << endl;
}


void StdCollisionPairInserter::find_signed_distance(
    Vector3& signed_distance, col_tri* trp, int nth, int ctype, int obj)
{
    find_signed_distance(signed_distance, trp->p1, nth, ctype, obj);
    find_signed_distance(signed_distance, trp->p2, nth, ctype, obj);
    find_signed_distance(signed_distance, trp->p3, nth, ctype, obj);
}


void StdCollisionPairInserter::find_signed_distance(
    Vector3& signed_distance, const Vector3& vert, int nth, int ctype, int obj)
{
    Vector3 vert_w;
    if(obj==1){
        vert_w = CD_s1 * (CD_Rot1 * vert + CD_Trans1);
    } else {
        vert_w = CD_s2 * (CD_Rot2 * vert + CD_Trans2);
    }
        
    if(COLLIDE_DEBUG) printf("vertex = %f %f %f\n", vert_w[0], vert_w[1], vert_w[2]);
        
    // use the first intersecting point to find the distance
    const Vector3 vec(vert_w - cdContact[nth].i_points[0]);
    //vecNormalize(cdContact[nth].n_vector);
    cdContact[nth].n_vector.normalize();
        
    double dis0 = cdContact[nth].n_vector.dot(vec);
    if(COLLIDE_DEBUG){
        cout << "vec = " << vec[0] << " " << vec[1] << " " << vec[2] << endl;
        cout << "n_vec = " << cdContact[nth].n_vector[0] << " " << cdContact[nth].n_vector[1] << " " << cdContact[nth].n_vector[2] << endl;
     }
        
#if 0
    switch(ctype){
    case FV:
        if(dot(cdContact[nth].n_vector, cdContact[nth].n) > 0.0) dis0 = - dis0;
        break;
    case VF:
        if(dot(cdContact[nth].n_vector, cdContact[nth].m) < 0.0) dis0 = - dis0;
        break;
    case EE:
        if(dot(cdContact[nth].n_vector, cdContact[nth].n) > 0.0 ||
           dot(cdContact[nth].n_vector, cdContact[nth].m) < 0.0)
            dis0 = - dis0;
    }
#endif
        
    if(COLLIDE_DEBUG) printf("dis0 = %f\n", dis0);
        
    double dis1 = dis0;
    double dis2 = dis0;
        
    switch(ctype){
    case FV:
        dis1 =   cdContact[nth].m.dot(vec);
        if(COLLIDE_DEBUG){
            cout << "m = " << cdContact[nth].m[0] << " " << cdContact[nth].m[1] << " " << cdContact[nth].m[2] << endl;
        }
        if(COLLIDE_DEBUG) printf("dis1 = %f\n", dis1);
        break;
    case VF:
        dis1 = - cdContact[nth].n.dot(vec);
        if(COLLIDE_DEBUG){
            cout << "n = " << cdContact[nth].n[0] << " " << cdContact[nth].n[1] << " " << cdContact[nth].n[2] << endl;
        }
        if(COLLIDE_DEBUG) printf("dis1 = %f\n", dis1);
        break;
    case EE:
        dis1 =   cdContact[nth].m.dot(vec);
        dis2 = - cdContact[nth].n.dot(vec);
        if(COLLIDE_DEBUG){
            printf("dis1 = %f\n", dis1);
            printf("dis2 = %f\n", dis2);
        }
    }

    if(COLLIDE_DEBUG) printf("obj = %d\n", obj);
    if(obj == 1){
        if(dis0 < signed_distance[0]) signed_distance[0] = dis0;
        if(dis1 < signed_distance[1]) signed_distance[1] = dis1;
        if(ctype==EE)
            if(dis2 < signed_distance[2]) signed_distance[2] = dis2;
    }
    else{
        if(signed_distance[0] < dis0) signed_distance[0] = dis0;
        if(signed_distance[1] < dis1) signed_distance[1] = dis1;
        if(ctype==EE)
            if(signed_distance[2] < dis2) signed_distance[2] = dis2;
    }
}


void StdCollisionPairInserter::find_signed_distance(
    Vector3& signed_distance,
    ColdetModelInternalModel* model,
    int id, 
    int contactIndex, 
    int ctype, 
    int obj)
{
    const int MIN_NUM_NEIGHBOR = 10;
    col_tri* tri_convex_neighbor = new col_tri[22];
    int num = get_triangles_in_convex_neighbor(model, id, tri_convex_neighbor, MIN_NUM_NEIGHBOR);

    for(int i=0; i<num; ++i){
        find_signed_distance(signed_distance, &tri_convex_neighbor[i], contactIndex, ctype, obj);
    }

    delete [] tri_convex_neighbor;
}


int StdCollisionPairInserter::new_point_test(int k)
{
    const double eps = 1.0e-12; // 1 micro meter to judge two contact points are identical
    
    int last = cdContact.size()-1;
    
    for(int i=0; i < last; ++i){
        for(int j=0; j < cdContact[i].num_of_i_points; ++j){
            Vector3 dv(cdContact[i].i_points[j] - cdContact[last].i_points[k]);
            double d = cdContact[i].depth - cdContact[last].depth;
            if(dv.dot(dv) < eps && d*d < eps) return 0;
        }
    }
    return 1;
}


//
// obsolute signatures
//
int StdCollisionPairInserter::apply(
    const Opcode::AABBCollisionNode* b1,
    const Opcode::AABBCollisionNode* b2,
    int id1, int id2,
    int num_of_i_points,
    Vector3 i_points[4],
    Vector3& n_vector,
    double depth,
    Vector3& n1,
    Vector3& m1,
    int ctype,
    Opcode::MeshInterface* mesh1,
    Opcode::MeshInterface* mesh2)
{
    cdContact.push_back(collision_data());
    collision_data& contact = cdContact.back();
    contact.id1 = id1;
    contact.id2 = id2;
    contact.depth = depth;
    contact.num_of_i_points = num_of_i_points;

    if(COLLIDE_DEBUG) printf("num_of_i_points = %d\n", num_of_i_points);

    for(int i=0; i < num_of_i_points; ++i){
        contact.i_points[i].noalias() = CD_s2 * ((CD_Rot2 * i_points[i]) + CD_Trans2);
    }

    contact.n_vector.noalias() = CD_Rot2 * n_vector;
    contact.n.noalias() = CD_Rot2 * n1;
    contact.m.noalias() = CD_Rot2 * m1;
    examine_normal_vector(id1, id2, ctype);

#ifdef DEPTH_CHECK
    // analyze_neighborhood_of_i_point(b1, b2, cdContactsCount, ctype);
    // remove the intersecting point if depth is deeper than MAX_DEPTH meter
    if(fabs(contact.depth) < MAX_DEPTH){
        for(int i=0; i < num_of_i_points; ++i){
            contact.i_point_new[i] = new_point_test(i);
        }
    } else {
        for(int i=0; i < num_of_i_points; ++i){
            contact.i_point_new[i] = 0;
        }
    }
#else
    for(int i=0; i < num_of_i_points; ++i){
        contact.i_point_new[i] = 1;
    }
#endif

    return CD_OK;
}


int StdCollisionPairInserter::detectTriTriOverlap(
    const Vector3& P1,
    const Vector3& P2,
    const Vector3& P3,
    const Vector3& Q1,
    const Vector3& Q2,
    const Vector3& Q3,
    collision_data* col_p)
{
    return tri_tri_overlap(P1, P2, P3, Q1, Q2, Q3, col_p, this);
}
