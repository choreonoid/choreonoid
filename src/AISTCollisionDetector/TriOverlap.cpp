//
// TriOverlap.cpp
//

#include "CollisionPairInserter.h"
#include <cmath>
#include <cstdio>
#include <iostream>

using namespace std;
using namespace cnoid;

namespace {

const bool HIRUKAWA_DEBUG = false;

/* used in normal_test */
enum { NOT_INTERSECT = 0,
       EDGE1_NOT_INTERSECT = 1,
       EDGE2_NOT_INTERSECT = 2,
       EDGE3_NOT_INTERSECT = 3 };

/* used in cross_test */
const int INTERSECT = 1;
}


/**********************************************************	
  separability test by the supporting plane of a triangle
   return value 
   0   : not intersect
   1   : f1 or e1 doesn't intersect
   2   : f2 or e2 doesn't intersect
   3   : f3 or e3 doesn't intersect
**********************************************************/	
static int separability_test_by_face(const Vector3& nm)
{
    if( (nm[0] < 0.0 && nm[1] < 0.0 && nm[2] < 0.0) ||
        (nm[0] > 0.0 && nm[1] > 0.0 && nm[2] > 0.0) ){
        return NOT_INTERSECT;
    }
    if( (nm[0] < 0.0 && nm[1] < 0.0 && nm[2] > 0.0) ||
        (nm[0] > 0.0 && nm[1] > 0.0 && nm[2] < 0.0) ){
        return EDGE1_NOT_INTERSECT;
    }
    if( (nm[0] < 0.0 && nm[1] > 0.0 && nm[2] > 0.0) ||
        (nm[0] > 0.0 && nm[1] < 0.0 && nm[2] < 0.0) ){
        return EDGE2_NOT_INTERSECT;
    }
    if( (nm[0] > 0.0 && nm[1] < 0.0 && nm[2] > 0.0) ||
        (nm[0] < 0.0 && nm[1] > 0.0 && nm[2] < 0.0) ){
        return EDGE3_NOT_INTERSECT;
    }
    return 0;
}


/**********************************************************
   triangle inside test:
   normal vector is cross product of ei*fj
***********************************************************/
static int triangle_inside_test(
    const Vector3& ef1,
    const Vector3& ef2,
    const Vector3& ef3,
    const Vector3& P3,
    const Vector3& P1,
    const Vector3& P2,
    const Vector3& Q)
{	
    double ef1P1 = ef1.dot(P1); /*project P1 on ef1*/
    double ef1P3 = ef1.dot(P3); /*project P3 on ef1*/
    double ef1Q  = ef1.dot(Q);  /*project Q on ef1*/

    double ef2P2 = ef2.dot(P2); /*project P2 on ef2*/
    double ef2P1 = ef2.dot(P1); /*project P1 on ef2*/	
    double ef2Q  = ef2.dot(Q);   /*project Q on ef2*/

    double ef3P3 = ef3.dot(P3);  /*project P3 on ef3*/	
    double ef3P2 = ef3.dot(P2);  /*project P2 on ef3*/		
    double ef3Q  = ef3.dot(Q);   /*project Q on ef3*/		

    if(( (ef1P3 > ef1P1 && ef1Q > ef1P1) 	||
         (ef1P3 < ef1P1 && ef1Q < ef1P1)     )
       &&
       ( (ef2P1 > ef2P2 && ef2Q > ef2P2) 	||
         (ef2P1 < ef2P2 && ef2Q < ef2P2)     )
       &&
       ( (ef3P2 > ef3P3 && ef3Q > ef3P3) 	||
         (ef3P2 < ef3P3 && ef3Q < ef3P3)     )) {
        return INTERSECT;
    }

    return NOT_INTERSECT;
}


static void find_intersection_pt(
    Vector3& ipt,
    const Vector3& x1,
    const Vector3& x2,
    double mn1, 
    double mn2)
{
    if(mn1 == mn2) /*exit(0);*/ return;

    if(mn1 >0 && mn2 < 0){
        ipt = (-(mn2*x1) + mn1*x2)/(mn1-mn2);
    }else if(mn1 < 0 && mn2 > 0){
        ipt = (mn2*x1 - mn1*x2)/(-mn1+mn2);
    }
}


static inline void find_intersection_pt(
    Vector3& ipt,
    const Vector3& x1,
    const Vector3& x2,
    double p)
{
    ipt = (1.0 - p) * x1 + p * x2;

    if(HIRUKAWA_DEBUG){
        cout << "v1 = " << x1[0] << ", " << x1[1] << ", " << x1[2] << endl; 
        cout << "v2 = " << x2[0] << ", " << x2[1] << ", " << x2[2] << endl;
        cout << "edge = " << x2[0]-x1[0] << ", " << x2[1]-x1[1] << ", "
             << x2[2]-x1[2] << endl;
        cout << "common pt = " << ipt[0] << " " << ipt[1] << " " << ipt[2] << endl;
    }
}


//
// Calculate the depth of the intersection between two triangles
//
static inline double calc_depth(
    const Vector3& ip1,
    const Vector3& ip2,
    const Vector3& n)
{
    // vecNormalize(n);

    if(HIRUKAWA_DEBUG){
        cout << "calc_depth 1 = " << (ip1 - ip2).dot(n) << endl;
    }
    
    return fabs((ip1 - ip2).dot(n));
}


static double calc_depth(
    const Vector3& ip,
    const Vector3& pt1,
    const Vector3& pt2,
    const Vector3& n)
{
    double d1 = fabs((ip - pt1).dot(n));
    double d2 = fabs((ip - pt2).dot(n));
    double depth = (d1 < d2) ? d1 : d2;

    if(HIRUKAWA_DEBUG){
        cout << "calc_depth 2 = " << depth << endl;
    }
    
    return depth;
} 


static double calc_depth(
    const Vector3& ip1,
    const Vector3& ip2,
    const Vector3& pt1,
    const Vector3& pt2,
    const Vector3& pt3,
    const Vector3& n)
{
    // when a triangle penetrates another triangle at two intersection points
    // and the separating plane is the supporting plane of the penetrated triangle
    
    // vecNormalize(n);
    
    double d1 = fabs((ip1 - pt1).dot(n));
    double d2 = fabs((ip2 - pt2).dot(n));
    double d3 = fabs((ip1 - pt3).dot(n)); // ip1 can be either ip1 or ip2
    
    double depth = (d1 < d2) ? d2 : d1;
    if(d3 < depth){
        depth = d3;
    }
    
    if(HIRUKAWA_DEBUG){
        cout << "calc_depth 3 = " << depth << endl;
    }
    
    return depth;
}


static void find_foot(
    const Vector3& ip,
    const Vector3& pt1,
    const Vector3& pt2,
    Vector3& f)
{
    /*
      double u, v, w, p;

      u = pt2[0] - pt1[0]; v = pt2[1] - pt1[1]; w = pt2[2] - pt1[2];
    
      p = u * (ip[0] - pt1[0]) + v * (ip[1] - pt1[1]) + w * (ip[2] - pt1[2]);
      p /= u * u + v * v + w * w;
    
      f[0] = pt1[0] + u * p; f[1] = pt1[1] + v * p; f[2] = pt1[2] + w * p;
    */

    const Vector3 pt(pt2 - pt1);
    const double p = pt.dot(ip - pt1) / pt.dot(pt);
    f = pt1 + p * pt;
}


static double calc_depth(
    const Vector3& ip,
    const Vector3& pt1,
    const Vector3& pt2,
    const Vector3& pt3,
    const Vector3& n)
{
    Vector3 f12, f23, f31;
    
    find_foot(ip, pt1, pt2, f12);
    find_foot(ip, pt2, pt3, f23);
    find_foot(ip, pt3, pt1, f31);
    
    if(HIRUKAWA_DEBUG){
        cout << "ip = " << ip[0] << " " << ip[1] << " " << ip[2] << endl;
        cout << "f12 = " << f12[0] << " " << f12[1] << " " << f12[2] << endl;
        cout << "f23 = " << f23[0] << " " << f23[1] << " " << f23[2] << endl;
        cout << "f31 = " << f31[0] << " " << f31[1] << " " << f31[2] << endl;
    }
    
    // fabs() is taken to cope with numerical error of find_foot()
    const double d1 = fabs((f12 - ip).dot(n));
    const double d2 = fabs((f23 - ip).dot(n));
    const double d3 = fabs((f31 - ip).dot(n));
    
    // cout << "d1 d2 d3 = " << d1 << " " << d2 << " " << d3 << endl;
    // dsum = fabs(d1)+fabs(d2)+fabs(d3);
    // if(d1<0.0) d1=dsum; if(d2<0.0) d2=dsum; if(d3<0.0) d3=dsum;
    
    double depth = (d1 < d2) ? d1 : d2;
    if(d3 < depth){
        depth = d3;
    }
    
    if(HIRUKAWA_DEBUG){
        cout << "calc_depth 4 = " << depth << endl;
    }
    
    return depth;
}


static double calc_depth2(
    const Vector3& ip1,
    const Vector3& ip2,
    const Vector3& pt1,
    const Vector3& pt2,
    const Vector3& pt3,
    const Vector3& n)
{
    // when a triangle penetrates another triangle at two intersecting points
    // and the separating plane is the supporting plane of the penetrating triangle
    
    const Vector3 nn(n); // vecNormalize(nn);
    
    const double depth1 = calc_depth(ip1, pt1, pt2, pt3, nn);
    const double depth2 = calc_depth(ip2, pt1, pt2, pt3, nn);
    
    // cout << "depth1 depth2 = " << depth1 << " " << depth2 << endl;
    const double depth = (depth1 < depth2) ? depth2 : depth1;
    
    if(HIRUKAWA_DEBUG){
        cout << "calc_depth 5 = " << depth << endl;
    }
    
    return depth;
}


static void calcNormal(
    Vector3& vec,
    const Vector3& v1,
    const Vector3& v2,
    const Vector3& v3,
    double sgn)
{
    // find the vector from v1 to the mid point of v2 and v3 when 0<sgn
    
    if(sgn < 0){
        vec = -(v1 - 0.5 * (v2 + v3)).normalized();
    } else {
        vec =  (v1 - 0.5 * (v2 + v3)).normalized();
    }
}


static int find_common_perpendicular(
    const Vector3& p1,
    const Vector3& p2,
    const Vector3& q1,
    const Vector3& q2,
    const Vector3& ip1,
    const Vector3& ip2,
    const Vector3& n1,
    const Vector3& m1,
    const Vector3& n_vector,
    double& dp)
{
    const double eps = 1.0e-3; // threshold to consider two edges are parallel
    const double vn = 1.0e-2;  // threshold to judge an intersecting point is near a vertex

    const Vector3 e(p2 - p1);
    const Vector3 f(q2 - q1);

    const double c11 = e.dot(e);
    const double c12 = - e.dot(f);
    const double c21 = - c12;
    const double c22 = - f.dot(f);

    const double det = c11 * c22 - c12 * c21;
    // cout << "det = " << det << endl;

    if(fabs(det) < eps * c11 * c22){
        return 0;
    } else {
        const Vector3 g(q1 - p1);
        const double a = e.dot(g);
        const double b = f.dot(g);
        const double t1 = ( c22 * a - c12 * b) / det;
        const double t2 = (-c21 * a + c11 * b) / det;

        // quit if the foot of the common perpendicular is not on an edge
        if(t1<0 || 1<t1 || t2<0 || 1<t2) return 0;

        // when two edges are in contact near a vertex of an edge
        // if(t1<vn || 1.0-vn<t1 || t2<vn || 1.0-vn<t2) return 0;

        // find_intersection_pt(v1, p1, p2, t1);
        // find_intersection_pt(v2, q1, q2, t2);
   
        dp = calc_depth(ip1, ip2, n_vector); 

        return 1;
    }
}
    

// for vertex-face contact
static inline int get_normal_vector_test(
    const Vector3& ip1,
    const Vector3& v1,
    const Vector3& ip2,
    const Vector3& v2,
    const Vector3& n1,
    const Vector3& m1)
{
    // ip1 and ip2 are the intersecting points
    // v1 and v2 are the vertices of the penetrating triangle
    // note that (v1-ip1) (v2-ip2) lies on the penetrating triangle
    
    // eps_applicability = 0.965926; // Cos(15) threshold to consider two triangles face
    const double eps_applicability = 0.5; // Cos(60) threshold to consider two triangles face
    
    // This condition should be checked mildly because the whole sole of a foot
    // may sink inside the floor and then no collision is detected.
    return (eps_applicability < n1.dot(m1)) ? 0 : 1;
}


// for edge-edge contact
static int get_normal_vector_test(
    Vector3& n_vector,
    const Vector3& ef0,
    const Vector3& ip,
    const Vector3& iq,
    const Vector3& v1,
    const Vector3& v2,
    const Vector3& n1,
    const Vector3& m1,
    const Vector3& va1,
    const Vector3& va2,
    const Vector3& vb1,
    const Vector3& vb2)
{
    // ip is the intersecting point on triangle p1p2p3
    // iq is the intersecting point on triangle q1q2q3
    // v1 is the vertex of p1p2p3 which is not on the intersecting edge
    // v2 is the vertex of q1q2q3 which is not on the intersecting edge
    // note that (v1-ip) lies on triangle p1p2p3 and (v2-iq) on q1q2q3

    const double eps_applicability = 0.0; // 1.52e-2; // threshold to consider two triangles face
    const double eps_length = 1.0e-3; // 1mm: thereshold to consider the intersecting edge is short
    const double eps_theta = 1.0e-1;   // threshold to consider cos(theta) is too small 

    // quit if two triangles does not satifsy the applicability condition
    // i.e. two triangles do not face each other
    if(- eps_applicability < n1.dot(m1)) return 0;
    
    const double ea_length = (va1 - va2).norm();
    const double eb_length = (vb1 - vb2).norm();
    
    // return the normal vector of a triangle if an intersecting edge is too short
    if(ea_length < eps_length || eb_length < eps_length){
        // cout << "edge is too short" << endl;
        if(ea_length < eb_length) {
            n_vector = m1;
        } else {
            n_vector = -n1;
        }
        return 1;
    }

    const Vector3 sv1(v1 - ip);
    const double sv1_norm = sv1.norm();
    const Vector3 sv2(v2 - iq);
    const double sv2_norm = sv2.norm();

    if(eps_length < sv1_norm && eps_length < sv2_norm){
        // quit if two triangles do not satisfy the applicability conditions
        if(- eps_applicability < sv1.dot(sv2) / (sv1_norm * sv2_norm)){
            return 0;
        }
    }

    // now neither of two edges is not too short
    Vector3 ef(ef0.normalized());

    // Triangle p1p2p3
    const double theta1 = ef.dot(n1) / n1.norm();
    const double theta1_abs = fabs(theta1);
    
    double theta2;
    double theta2_abs;
    if(eps_length < sv1_norm){
        theta2 = ef.dot(sv1) / sv1_norm;
        theta2_abs = fabs(theta2);
    } else {
        theta2 = 0.0;
        theta2_abs = 0.0;
    }
    
    // triangle q1q2q3
    const double theta3 = ef.dot(m1) / m1.norm();
    const double theta3_abs = fabs(theta3);
    
    double theta4;
    double theta4_abs;
    if(eps_length < sv2_norm){
        theta4 = ef.dot(sv2) / sv2_norm;
        theta4_abs = fabs(theta4);
    } else {
        theta4 = 0.0;
        theta4_abs = 0.0;
    }

    if(sv1_norm < eps_length || sv2_norm < eps_length){
        // when sv1 or sv2 is too short
        // cout << "sv is too short" << endl;
        if(theta1_abs < theta3_abs){
            n_vector = m1;
        } else {
            n_vector = -n1;
        }
        return 1;    
    }

    if(theta2_abs < eps_theta && theta4_abs < eps_theta){
        // when two triangles are coplanar
        // proof.
        //  ef = (va2-va1)x(vb2-vb1) (1)
        //  sv1 * ef = 0             (2)
        //  sv2 * ef = 0             
        //  substituting (1) to (2),
        //    sv1 * (va2-va1) x (vb2-vb1) = 0
        //    (vb2 - vb1) * sv1 x (va2 - va1) = 0
        //  considering sv1 x (va2 - va1) = n,
        //    (vb2 - vb1) * n = 0
        //  in the same way
        //    (va2 - va1) * m = 0
        // q.e.d.

        if(theta1_abs < theta3_abs){
            n_vector = m1;
            return 1;
        } else{
            n_vector = -n1;
            return 1;
        }
    }

    // return 0 if the plane which passes through ip with normal vector ef
    // does not separate v1 and v2
    if(-eps_applicability < theta2 * theta4) return 0;

    //
    // regular case
    //
    double theta12;
    if(theta1_abs < theta2_abs){
        theta12 = theta2;
    } else {
        theta12 = -1.0 * theta1;
    }

    double theta34;
    if(theta3_abs < theta4_abs){
        theta34 = -1.0 * theta4;
    } else {
        theta34 = theta3;
    }

    double theta;
    if(fabs(theta12) < fabs(theta34)){
        theta = theta34;
    } else {
        theta = theta12;
    }

    if(0 < theta){
        n_vector = ef;
    } else {
        n_vector = -ef;
    }

    if(HIRUKAWA_DEBUG){
        cout << "va1=" << va1[0] << " " << va1[1] << " " << va1[2] << endl;
        cout << "va2=" << va2[0] << " " << va2[1] << " " << va2[2] << endl;
        cout << "va3=" << v1[0] << " " << v1[1] << " " << v1[2] << endl;
        cout << "vb1=" << vb1[0] << " " << vb1[1] << " " << vb1[2] << endl;
        cout << "vb2=" << vb2[0] << " " << vb2[1] << " " << vb2[2] << endl;
        cout << "vb3=" << v2[0] << " " << v2[1] << " " << v2[2] << endl;
        cout << "n1=" << n1[0] << " " << n1[1] << " " << n1[2] << endl;
        cout << "m1=" << m1[0] << " " << m1[1] << " " << m1[2] << endl;
        cout << "ef=" << ef[0] << " " << ef[1] << " " << ef[2] << endl;
        cout << "sv1=" << sv1[0] << " " << sv1[1] << " " << sv1[2] << endl;
        cout << "sv2=" << sv2[0] << " " << sv2[1] << " " << sv2[2] << endl;
        cout << endl;
    }

    if(n_vector.dot(sv1) < eps_applicability || - eps_applicability < n_vector.dot(sv2)){
        // when the separating plane separates the outsides of the triangles
        return 0;
    } else {
        return 1;
    }
}


//
// find the collision info when a vertex penetrates a face
//
static int find_collision_info(
    const Vector3& p1,
    const Vector3& p2,
    const Vector3& p3,
    double mp0,
    double mp1,
    double mp2,
    const Vector3& q1,
    const Vector3& q2,
    const Vector3& q3,
    const Vector3& f1,
    const Vector3& f2,
    const Vector3& f3,
    const Vector3& n1,
    const Vector3& m1,
    Vector3& ip3,
    Vector3& ip4,
    Vector3& ip5, /* unused ? */
    Vector3& ip6, /* unused ? */
    collision_data* col_p, double pq)
{
    Vector3 ip1;
    find_intersection_pt(ip1, p1, p2, mp0, mp1);
    Vector3 ip2;
    find_intersection_pt(ip2, p3, p1, mp2, mp0);
    
    if(get_normal_vector_test(ip1, p2, ip2, p3, m1, n1)){

        Vector3 vec;
        calcNormal(vec, p1, p2, p3, mp0);
        
        col_p->n_vector = m1 * pq;

        //
        // The depth is estimated in InsertCollisionPair.cpp
        // The following depth calculation is done only for debugging purpose
        //
        col_p->depth = calc_depth(ip1, ip2, p2, p3, p1, col_p->n_vector);
        const Vector3 nv(-n1 * pq);
        const double dp = calc_depth2(ip1, ip2, q1, q2, q3, nv);
        if(dp < col_p->depth){
            col_p->depth = dp;
        }

        ip3 = ip1; ip4 = ip2;
        col_p->num_of_i_points = 2;

        return 1;
    }

    return 0;
}


//
// find the collision info when an edges penetrate a face each other 
//
static int find_collision_info(
    const Vector3& p1,
    const Vector3& p2,
    const Vector3& p3,
    double mp0,
    double mp1,
    const Vector3& q1,
    const Vector3& q2,
    const Vector3& q3,
    double nq0,
    double nq1,
    const Vector3& ef11,
    const Vector3& n1,
    const Vector3& m1,
    Vector3& ip3,
    Vector3& ip4,
    collision_data *col_p)
{
    Vector3 ip1;
    find_intersection_pt(ip1, q1, q2, nq0, nq1);
    Vector3 ip2;
    find_intersection_pt(ip2, p1, p2, mp0, mp1);

    double dp;
    if(get_normal_vector_test(col_p->n_vector, ef11, ip2, ip1, p3, q3, n1, m1, p1, p2, q1, q2) &&
       find_common_perpendicular(p1, p2, q1, q2, ip1, ip2, n1, m1, col_p->n_vector, dp)){

        ip3 = ip1; ip4 = ip2;
        col_p->num_of_i_points = 2;
        col_p->depth = dp;
        return 1;
    }

    return 0;
}


namespace Opcode {

// very robust triangle intersection test
// uses no divisions
// works on coplanar triangles

int tri_tri_overlap(
    const Vector3& P1,
    const Vector3& P2,
    const Vector3& P3,
    const Vector3& Q1,
    const Vector3& Q2,
    const Vector3& Q3,
    collision_data* col_p,
    CollisionPairInserter* collisionPairInserter) 
{
    /*
      One triangle is (p1,p2,p3).  Other is (q1,q2,q3).
      Edges are (e1,e2,e3) and (f1,f2,f3).
      Normals are n1 and m1
      Outwards are (g1,g2,g3) and (h1,h2,h3).
     
      We assume that the triangle vertices are in the same coordinate system.

      First thing we do is establish a new c.s. so that p1 is at (0,0,0).

    */
    Vector3 p1, p2, p3;
    Vector3 q1, q2, q3;
    Vector3 e1, e2, e3;
    Vector3 f1, f2, f3;
    // Vector3 g1, g2, g3;
    // Vector3 h1, h2, h3;
    Vector3 n1, m1;
    Vector3 z;
    Vector3 nq, mp;

    int triP,triQ;
    int edf1, edf2, edf3, ede1, ede2, ede3;

    Vector3 ef11, ef12, ef13;
    Vector3 ef21, ef22, ef23;
    Vector3 ef31, ef32, ef33;

    /* intersection point   R is a flag which tri P & Q correspond or not  */
    Vector3 ip,ip3,ip4,ip5,ip6;
    Vector3 i_pts_w[4];
  
    const int FV = 1; // face-vertex contact type
    const int VF = 2; // vertex-face contact type
    const int EE = 3; // edge-edge contact type

    z << 0.0,0.0,0.0;
  
    p1 =  P1 - P1;
    p2 =  P2 - P1;
    p3 =  P3 - P1;
  
    q1 =  Q1 - P1;
    q2 =  Q2 - P1;
    q3 =  Q3 - P1;
  
    e1 =  p2 - p1;
    e2 =  p3 - p2;
    e3 =  p1 - p3;

    f1 =  q2 - q1;
    f2 =  q3 - q2;
    f3 =  q1 - q3;

    n1 = e1.cross(e2);
    m1 = f1.cross(f2);

    // now begin the series of tests

    /*************************************
        separability test by face
    ************************************/

    nq[0] = n1.dot(q1);
    nq[1] = n1.dot(q2);
    nq[2] = n1.dot(q3);
    triQ = separability_test_by_face(nq);

    if(triQ == NOT_INTERSECT) return 0;

    double mq = m1.dot(q1);
    mp[0] = m1.dot(p1) - mq;
    mp[1] = m1.dot(p2) - mq;
    mp[2] = m1.dot(p3) - mq;
    triP = separability_test_by_face(mp);
    if(triP == NOT_INTERSECT) return 0;

    ef11 = e1.cross(f1);
    ef12 = e1.cross(f2);
    ef13 = e1.cross(f3);
    ef21 = e2.cross(f1);
    ef22 = e2.cross(f2);
    ef23 = e2.cross(f3);
    ef31 = e3.cross(f1);
    ef32 = e3.cross(f2);
    ef33 = e3.cross(f3);

    edf1 = 0; edf2 = 0; edf3 = 0; ede1 = 0; ede2 = 0; ede3 = 0;

    /********************************
	 triangle inside test
    *********************************/	
    switch(triQ)
        {
        case NOT_INTERSECT:
            return 0;

        case EDGE1_NOT_INTERSECT:    
            edf2 = triangle_inside_test(ef12,ef22,ef32,p3,p1,p2,q2);
            edf3 = triangle_inside_test(ef13,ef23,ef33,p3,p1,p2,q3);
            break;

        case EDGE2_NOT_INTERSECT:	  
            edf1 = triangle_inside_test(ef11,ef21,ef31,p3,p1,p2,q1);	
            edf3 = triangle_inside_test(ef13,ef23,ef33,p3,p1,p2,q3);
            break;

        case EDGE3_NOT_INTERSECT:	
            edf1 = triangle_inside_test(ef11,ef21,ef31,p3,p1,p2,q1);	
            edf2 = triangle_inside_test(ef12,ef22,ef32,p3,p1,p2,q2);
            break;
        }

    int num_of_edges = edf1 + edf2 + edf3;
    if(num_of_edges == 3){
        //exit(1);
        return 0;
    }
 
    if(num_of_edges < 2){
        switch(triP)
            {
            case EDGE1_NOT_INTERSECT:
                ede2 = triangle_inside_test(ef21,ef22,ef23,q3,q1,q2,p2);
                ede3 = triangle_inside_test(ef31,ef32,ef33,q3,q1,q2,p3);
                if(ede2+ede3==2){
                    edf1= NOT_INTERSECT;
                    edf2= NOT_INTERSECT;
                    edf3= NOT_INTERSECT;
                }
                break;
	
            case EDGE2_NOT_INTERSECT:
                ede1 = triangle_inside_test(ef11,ef12,ef13,q3,q1,q2,p1);
                ede3 = triangle_inside_test(ef31,ef32,ef33,q3,q1,q2,p3);
                if(ede1+ede3==2){
                    edf1= NOT_INTERSECT;
                    edf2= NOT_INTERSECT;
                    edf3= NOT_INTERSECT;
                }
                break;    
	
            case EDGE3_NOT_INTERSECT:
                ede1 = triangle_inside_test(ef11,ef12,ef13,q3,q1,q2,p1);
                ede2 = triangle_inside_test(ef21,ef22,ef23,q3,q1,q2,p2);
                if(ede1+ede2 == 2){
                    edf1= NOT_INTERSECT;
                    edf2= NOT_INTERSECT;
                    edf3= NOT_INTERSECT;
                }
                break;
            }
        if(num_of_edges == 0 && ede1+ede2+ede3 == 3){
            //exit(1);
            return 0;
        }
    }
  
    int num = edf1+edf2+edf3+ede1+ede2+ede3;
    if(num == 0){
        // cout << "no edge intersect" << endl;
        return 0;
    }
    else if(num > 2){
        printf("err of edge detection....");
        //exit(1);
        return 0;
    }

    n1.normalize();
    m1.normalize();

    /*********************************
    find intersection points
    **********************************/
    if(num==1){
        if(edf1==INTERSECT){
            find_intersection_pt(ip,q1,q2,nq[0],nq[1]);
            ip3 = ip;
            col_p->n_vector = -n1;
            col_p->depth = 0.0;
            col_p->c_type = FV;
        }
        else if(edf2==INTERSECT){
            find_intersection_pt(ip,q2,q3,nq[1],nq[2]);
            ip3 = ip;
            col_p->n_vector = -n1;
            col_p->depth = 0.0;
            col_p->c_type = FV;
        }
        else if(edf3==INTERSECT){
            find_intersection_pt(ip,q3,q1,nq[2],nq[0]);
            ip3 = ip;
            col_p->n_vector = -n1;
            col_p->depth = 0.0;
            col_p->c_type = FV;
        }
        else if(ede1==INTERSECT){
            find_intersection_pt(ip,p1,p2,mp[0],mp[1]);
            ip3 =  ip;
            col_p->n_vector = m1;
            col_p->depth = 0.0;
            col_p->c_type = VF;
        }
        else if(ede2==INTERSECT){
            find_intersection_pt(ip,p2,p3,mp[1],mp[2]);
            ip3 =  ip;
            col_p->n_vector = m1;
            col_p->depth = 0.0;
            col_p->c_type = VF;
        }
        else if(ede3==INTERSECT){
            find_intersection_pt(ip,p3,p1,mp[2],mp[0]);
            ip3 =  ip;
            col_p->n_vector = m1;
            col_p->depth = 0.0;
            col_p->c_type = VF;
        }
        col_p->num_of_i_points = 1;
    }
    else if(num==2)
        {
            if(edf1==INTERSECT && edf2==INTERSECT){
                if(HIRUKAWA_DEBUG) cout << "f1 f2" << endl;
                col_p->c_type = FV;
                if(!find_collision_info(q2,q1,q3,nq[1],nq[0],nq[2],p1,p2,p3,e1,e2,e3,
                                        m1,n1,ip3,ip4,ip5,ip6,col_p,-1.0))
                    return 0;
            }
            else if(edf1==INTERSECT && edf3==INTERSECT){
                if(HIRUKAWA_DEBUG) cout << "f1 f3" << endl;
                col_p->c_type = FV;
                if(!find_collision_info(q1,q2,q3,nq[0],nq[1],nq[2],p1,p2,p3,e1,e2,e3,
                                        m1,n1,ip3,ip4,ip5,ip6,col_p,-1.0))
                    return 0;
            }
            else if(ede1==INTERSECT && edf1==INTERSECT){
                if(HIRUKAWA_DEBUG) cout << "e1 f1" << endl;
                col_p->c_type = EE;
                if(!find_collision_info(p1,p2,p3,mp[0],mp[1],q1,q2,q3,nq[0],nq[1],ef11,
                                        n1,m1,ip3,ip4,col_p))
                    return 0;
            }
            else if(ede2==INTERSECT && edf1==INTERSECT){
                if(HIRUKAWA_DEBUG) cout << "e2 f1" << endl;
                col_p->c_type = EE;
                if(!find_collision_info(p2,p3,p1,mp[1],mp[2],q1,q2,q3,nq[0],nq[1],ef21,
                                        n1,m1,ip3,ip4,col_p))
                    return 0;
            }
            else if(ede3==INTERSECT && edf1==INTERSECT){
                if(HIRUKAWA_DEBUG) cout << "e3 f1" << endl;
                col_p->c_type = EE;
                if(!find_collision_info(p3,p1,p2,mp[2],mp[0],q1,q2,q3,nq[0],nq[1],ef31,
                                        n1,m1,ip3,ip4,col_p))
                    return 0;
            }
            else if(edf2==INTERSECT && edf3==INTERSECT){
                if(HIRUKAWA_DEBUG) cout << "f2 f3" << endl;
                col_p->c_type = FV;
                if(!find_collision_info(q3,q2,q1,nq[2],nq[1],nq[0],p1,p2,p3,e1,e2,e3,
                                        m1,n1,ip3,ip4,ip5,ip6,col_p,-1.0))
                    return 0;
            }
            else if(ede1==INTERSECT && edf2==INTERSECT){
                if(HIRUKAWA_DEBUG) cout << "e1 f2" << endl;
                col_p->c_type = EE;
                if(!find_collision_info(p1,p2,p3,mp[0],mp[1],q2,q3,q1,nq[1],nq[2],ef12,
                                        n1,m1,ip3,ip4,col_p))
                    return 0;
            }
            else if(ede2==INTERSECT && edf2==INTERSECT){
                if(HIRUKAWA_DEBUG) cout << "e2 f2" << endl;
                col_p->c_type = EE;
                if(!find_collision_info(p2,p3,p1,mp[1],mp[2],q2,q3,q1,nq[1],nq[2],ef22,
                                        n1,m1,ip3,ip4,col_p))
                    return 0;
            }
            else if(ede3==INTERSECT && edf2==INTERSECT){
                if(HIRUKAWA_DEBUG) cout << "e3 f2" << endl;
                col_p->c_type = EE;
                if(!find_collision_info(p3,p1,p2,mp[2],mp[0],q2,q3,q1,nq[1],nq[2],ef32,
                                        n1,m1,ip3,ip4,col_p))
                    return 0;
            }
            else if(ede1==INTERSECT && edf3==INTERSECT){
                if(HIRUKAWA_DEBUG) cout << "e1 f3" << endl;
                col_p->c_type = EE;
                if(!find_collision_info(p1,p2,p3,mp[0],mp[1],q3,q1,q2,nq[2],nq[0],ef13,
                                        n1,m1,ip3,ip4,col_p))
                    return 0;
            }
            else if(ede2==INTERSECT && edf3==INTERSECT){
                if(HIRUKAWA_DEBUG) cout << "e2 f3" << endl;
                col_p->c_type = EE;
                if(!find_collision_info(p2,p3,p1,mp[1],mp[2],q3,q1,q2,nq[2],nq[0],ef23,
                                        n1,m1,ip3,ip4,col_p))
                    return 0;
            }
            else if(ede3==INTERSECT && edf3==INTERSECT){
                if(HIRUKAWA_DEBUG) cout << "e3 f3" << endl;
                col_p->c_type = EE;
                if(!find_collision_info(p3,p1,p2,mp[2],mp[0],q3,q1,q2,nq[2],nq[0],ef33,
                                        n1,m1,ip3,ip4,col_p))
                    return 0;
            }
            else if(ede1==INTERSECT && ede2==INTERSECT){
                if(HIRUKAWA_DEBUG) cout << "e1 e2" << endl;
                col_p->c_type = VF;
                if(!find_collision_info(p2,p1,p3,mp[1],mp[0],mp[2],q1,q2,q3,f1,f2,f3,
                                        n1,m1,ip3,ip4,ip5,ip6,col_p,1.0))
                    return 0;
            }
            else if(ede1==INTERSECT && ede3==INTERSECT){
                if(HIRUKAWA_DEBUG) cout << "e1 e3" << endl;
                col_p->c_type = VF;
                if(!find_collision_info(p1,p2,p3,mp[0],mp[1],mp[2],q1,q2,q3,f1,f2,f3,
                                        n1,m1,ip3,ip4,ip5,ip6,col_p,1.0))
                    return 0;
            }
            else if(ede2==INTERSECT && ede3==INTERSECT){
                if(HIRUKAWA_DEBUG) cout << "e2 e3" << endl;
                col_p->c_type = VF;
                if(!find_collision_info(p3,p2,p1,mp[2],mp[1],mp[0],q1,q2,q3,f1,f2,f3,
                                        n1,m1,ip3,ip4,ip5,ip6,col_p,1.0))
                    return 0;
            }
        }    

    if(col_p->num_of_i_points == 1){
        col_p->i_points[0] = ip3 + P1;
    }
    else if(col_p->num_of_i_points == 2){
        col_p->i_points[0] = ip3 + P1;
        col_p->i_points[1] = ip4 + P1;
    }
    else if(col_p->num_of_i_points == 3){
        col_p->i_points[0] = ip3 + P1;
        col_p->i_points[1] = ip4 + P1;
        col_p->i_points[2] = ip5 + P1;
    }
    else if(col_p->num_of_i_points == 4){
        col_p->i_points[0] = ip3 + P1;
        col_p->i_points[1] = ip4 + P1;
        col_p->i_points[2] = ip5 + P1;
        col_p->i_points[3] = ip5 + P1;
    }

    col_p->n = n1;
    col_p->m = m1;
    
    if(HIRUKAWA_DEBUG){

        CollisionPairInserter& c = *collisionPairInserter;
    
        Vector3 p1w(c.CD_s2 * (c.CD_Rot2 * P1 + c.CD_Trans2));
        Vector3 p2w(c.CD_s2 * (c.CD_Rot2 * P2 + c.CD_Trans2));
        Vector3 p3w(c.CD_s2 * (c.CD_Rot2 * P3 + c.CD_Trans2));
        Vector3 q1w(c.CD_s2 * (c.CD_Rot2 * Q1 + c.CD_Trans2));
        Vector3 q2w(c.CD_s2 * (c.CD_Rot2 * Q2 + c.CD_Trans2));
        Vector3 q3w(c.CD_s2 * (c.CD_Rot2 * Q3 + c.CD_Trans2));
        cout << "P1 = " << p1w[0] << " " << p1w[1] << " " << p1w[2] << endl;
        cout << "P2 = " << p2w[0] << " " << p2w[1] << " " << p2w[2] << endl;
        cout << "P3 = " << p3w[0] << " " << p3w[1] << " " << p3w[2] << endl;
        cout << "Q1 = " << q1w[0] << " " << q1w[1] << " " << q1w[2] << endl;
        cout << "Q2 = " << q2w[0] << " " << q2w[1] << " " << q2w[2] << endl;
        cout << "Q3 = " << q3w[0] << " " << q3w[1] << " " << q3w[2] << endl;

        for(int i=0; i<col_p->num_of_i_points; i++){
            i_pts_w[i] = c.CD_s2 * ((c.CD_Rot2 * col_p->i_points[i]) + c.CD_Trans2);
            cout << i << "-th intersecting point = ";
            cout << i_pts_w[i][0] << " " << i_pts_w[i][1] << " " << i_pts_w[i][2] << endl;
        }

        cout << "n1 = " << n1[0] << " " << n1[1] << " " << n1[2] << endl;
        cout << "m1 = " << m1[0] << " " << m1[1] << " " << m1[2] << endl;
        cout << "mp[0] mp[1] mp[2] = " << mp[0] << " " << mp[1] << " " << mp[2] << endl;
        cout << "nq[0] nq[1] nq[2] = " << nq[0] << " " << nq[1] << " " << nq[2] << endl;
        cout << "n_vector = " << col_p->n_vector[0] << " " << col_p->n_vector[1]
             << " " << col_p->n_vector[2] << endl;
        cout << "depth = " << col_p->depth << endl << endl;;

    }

#if TRACE1
    printf("intersect point...in tri_contact..\n");
    printf("    ip1x = %f ip1y = %f ip1z = %f\n    ip2x = %f ip2y = %f ip2z = %f\n",
           col_p->i_points[0][0],col_p->i_points[0][1],col_p->i_points[0][2],
           col_p->i_points[1][0],col_p->i_points[1][1],col_p->i_points[1][2]);

    printf("normal vector....it tri_conctact..\n");
    printf("N[0] = %f,N[1] = %f,N[2] = %f\n",col_p->n_vector[0],col_p->n_vector[1],col_p->n_vector[2]);
#endif

    return 1;
}
}

