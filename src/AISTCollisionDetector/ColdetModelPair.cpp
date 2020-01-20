/**
   @author Shin'ichiro Nakaoka
   @author Rafael Cisneros
*/

#include "ColdetModelPair.h"
#include "ColdetModelInternalModel.h"
#include "StdCollisionPairInserter.h"
#include "Opcode/Opcode.h"
#include "SSVTreeCollider.h"
#include <iostream>

using namespace std;
using namespace cnoid;

namespace {
const float LOCAL_EPSILON = 0.0001f;

enum pointType {vertex, inter};
enum figType {tri, sector};

struct pointStruct {
    float x, y, angle;
    pointType type;
    int code;
};
    
struct figStruct {
    figType type;
    int p1, p2;
    float area;
    float cx, cy;
};
}


ColdetModelPair::ColdetModelPair()
{
    collisionPairInserter = new Opcode::StdCollisionPairInserter;
}


ColdetModelPair::ColdetModelPair(ColdetModel* model0, ColdetModel* model1, double tolerance)
{
    collisionPairInserter = new Opcode::StdCollisionPairInserter;
    set(model0, model1);
    tolerance_ = tolerance;
}


ColdetModelPair::ColdetModelPair(const ColdetModelPair& org)
{
    collisionPairInserter = new Opcode::StdCollisionPairInserter;
    set(org.models[0], org.models[1]);
    tolerance_ = org.tolerance_;
}


ColdetModelPair::~ColdetModelPair()
{
    delete collisionPairInserter;
}


void ColdetModelPair::set(ColdetModel* model0, ColdetModel* model1)
{
    models[0] = model0;
    models[1] = model1;
    // inverse order because of historical background
    // this should be fixed.(note that the direction of normal is inversed when the order inversed 
    if(model0 && model1){
        collisionPairInserter->set(model1->internalModel, model0->internalModel);
    }
}


std::vector<collision_data>& ColdetModelPair::detectCollisionsSub(bool detectAllContacts)
{
    collisionPairInserter->clear();

    int pt0 = models[0]->getPrimitiveType();
    int pt1 = models[1]->getPrimitiveType();
    bool detected;
    bool detectPlaneSphereCollisions(bool detectAllContacts);
    
    if (( pt0 == ColdetModel::SP_PLANE && pt1 == ColdetModel::SP_CYLINDER)
        || (pt1 == ColdetModel::SP_PLANE && pt0 == ColdetModel::SP_CYLINDER)){
        detected = detectPlaneCylinderCollisions(detectAllContacts);
    }
    else if (pt0 == ColdetModel::SP_PLANE || pt1 == ColdetModel::SP_PLANE){
        detected = detectPlaneMeshCollisions(detectAllContacts);
    }
    else if (pt0 == ColdetModel::SP_SPHERE && pt1 == ColdetModel::SP_SPHERE) {
        detected = detectSphereSphereCollisions(detectAllContacts);
    }
	
    else if (pt0 == ColdetModel::SP_SPHERE || pt1 == ColdetModel::SP_SPHERE) {
        detected = detectSphereMeshCollisions(detectAllContacts);
    }
    else {
        detected = detectMeshMeshCollisions(detectAllContacts);
    }

    if(!detected){
        collisionPairInserter->clear();
    }

    return collisionPairInserter->collisions();
}


bool ColdetModelPair::detectPlaneMeshCollisions(bool detectAllContacts)
{
    bool result = false;

    ColdetModel* plane = nullptr;
    ColdetModel* mesh = nullptr;
    
    bool reversed=false;
    if(models[0]->getPrimitiveType() == ColdetModel::SP_PLANE){
        plane = models[0];
        mesh = models[1];
    }
    if(models[1]->getPrimitiveType() == ColdetModel::SP_PLANE){
        plane = models[1];
        mesh = models[0];
        reversed = true;
    }
    if(!plane || !mesh || !mesh->internalModel->model.GetMeshInterface()){
        return false;
    }

    Opcode::PlanesCollider PC;
    if(!detectAllContacts){
        PC.SetFirstContact(true);
    }
    PC.setCollisionPairInserter(collisionPairInserter);
    IceMaths::Matrix4x4 mTrans = *(mesh->transform);
    for(udword i=0; i<3; i++){
        for(udword j=0; j<3; j++){
            collisionPairInserter->CD_Rot1(i,j) = mTrans[j][i];
        }
        collisionPairInserter->CD_Trans1[i] = mTrans[3][i];
    }
    collisionPairInserter->CD_s1 = 1.0;

    Opcode::PlanesCache Cache;
    IceMaths::Matrix4x4 pTrans = (*(plane->pTransform)) * (*(plane->transform));
    IceMaths::Point p, nLocal(0,0,1), n;
    IceMaths::TransformPoint3x3(n, nLocal, pTrans);
    pTrans.GetTrans(p);
    Plane Planes[] = {Plane(p, n)};
    bool IsOk = PC.Collide(Cache, Planes, 1, mesh->internalModel->model, 
                           mesh->transform);
    if (!IsOk){
        std::cerr << "PlanesCollider::Collide() failed" << std::endl;
    } else {
        result = PC.GetContactStatus();
    }
    if(reversed){
        std::vector<collision_data>& cdata = collisionPairInserter->collisions();
        for(size_t i=0; i < cdata.size(); i++){
            cdata[i].n_vector *= -1;
        }
    }

    return result;
}

bool ColdetModelPair::detectMeshMeshCollisions(bool detectAllContacts)
{
    bool result = false;
    
    if(models[0]->isValid() && models[1]->isValid()){

        Opcode::BVTCache colCache;

        // inverse order because of historical background
        // this should be fixed.(note that the direction of normal is inversed when the order inversed 
        colCache.Model0 = &models[1]->internalModel->model;
        colCache.Model1 = &models[0]->internalModel->model;
        
        if(colCache.Model0->HasSingleNode() || colCache.Model1->HasSingleNode())
            return result;

        Opcode::AABBTreeCollider collider;
        collider.setCollisionPairInserter(collisionPairInserter);
        
        if(!detectAllContacts){
            collider.SetFirstContact(true);
        }
        
        bool isOk = collider.Collide(colCache, models[1]->transform, models[0]->transform);
		
        if (!isOk)
            std::cerr << "AABBTreeCollider::Collide() failed" << std::endl;
		
        result = collider.GetContactStatus();
        
        boxTestsCount = collider.GetNbBVBVTests();
        triTestsCount = collider.GetNbPrimPrimTests();
    }

    return result;
}

bool ColdetModelPair::detectSphereSphereCollisions(bool detectAllContacts) {
	
    bool result = false;
    int sign = 1;
	
    if (models[0]->isValid() && models[1]->isValid()) {
		
        ColdetModel* sphereA = models[0];
        ColdetModel* sphereB = models[1];
		
        IceMaths::Matrix4x4 sATrans = (*(sphereA->pTransform)) * (*(sphereA->transform));
        IceMaths::Matrix4x4 sBTrans = (*(sphereB->pTransform)) * (*(sphereB->transform));

        float radiusA, radiusB;		
        sphereA->getPrimitiveParam(0, radiusA);
        sphereB->getPrimitiveParam(0, radiusB);

        IceMaths::Point centerA = sATrans.GetTrans();
        IceMaths::Point centerB = sBTrans.GetTrans();
		
        IceMaths::Point D = centerB - centerA;
		
        float depth = radiusA + radiusB - D.Magnitude();
		
        if (D.Magnitude() <= (radiusA + radiusB)) {

            result = true;

            float x = (pow(D.Magnitude(), 2) + pow(radiusA, 2) - pow(radiusB, 2)) / (2 * D.Magnitude());
            float R = sqrt(pow(radiusA, 2) - pow(x, 2));
			
            IceMaths::Point n = D / D.Magnitude();
			
            IceMaths::Point q = centerA + n * x;

            std::vector<collision_data>& cdata = collisionPairInserter->collisions();
            cdata.clear();			
			
            collision_data col;
            col.depth = depth;
            col.num_of_i_points = 1;
            col.i_point_new[0] = 1;
            col.i_point_new[1] = 0;
            col.i_point_new[2] = 0;
            col.i_point_new[3] = 0;
            col.n_vector[0] = sign * n.x;
            col.n_vector[1] = sign * n.y;
            col.n_vector[2] = sign * n.z;
            col.i_points[0][0] = q.x;
            col.i_points[0][1] = q.y;
            col.i_points[0][2] = q.z;
            cdata.push_back(col);
        }
    }

    return result;
}

bool ColdetModelPair::detectSphereMeshCollisions(bool detectAllContacts) {
	
    bool result = false;
    int sign = 1;

    if (models[0]->isValid() && models[1]->isValid()) {
		
        ColdetModel* sphere = nullptr;
        ColdetModel* mesh = nullptr;

        if(models[0]->getPrimitiveType() == ColdetModel::SP_SPHERE){
            sphere = models[0];
            mesh = models[1];
            sign = -1;
        }
        else if(models[1]->getPrimitiveType() == ColdetModel::SP_SPHERE){
            sphere = models[1];
            mesh = models[0];
        }

        if(!sphere || !mesh){
            return false;
        }

        IceMaths::Matrix4x4 sTrans = (*(sphere->pTransform)) * (*(sphere->transform));
		
        float radius;
        sphere->getPrimitiveParam(0, radius);

        IceMaths::Sphere sphere_def(IceMaths::Point(0, 0, 0), radius);
		
        Opcode::SphereCache colCache;

        Opcode::SphereCollider collider;
		
        if (!detectAllContacts) {
            collider.SetFirstContact(true);
        }
		
        bool isOk = collider.Collide(colCache, sphere_def, mesh->internalModel->model, &sTrans, mesh->transform);

        if (isOk) {

            if (collider.GetContactStatus()) {
				
                int TouchedPrimCount = collider.GetNbTouchedPrimitives();
                const udword* TouchedPrim = collider.GetTouchedPrimitives();
				
                if (TouchedPrimCount) {
				
                    result = true;
					
                    std::vector< std::vector<IceMaths::Point> > triangle(TouchedPrimCount);		// Triangle of each face in world's coordinates
                    std::vector<IceMaths::Plane> face(TouchedPrimCount);				// Plane of each face in world's coordinates 
					
                    std::vector<float> depth(TouchedPrimCount);
					
                    std::vector<IceMaths::Point> q(TouchedPrimCount);
                    std::vector<float> A(TouchedPrimCount);
					
                    IceMaths::Matrix4x4 sTransInv;
                    IceMaths::InvertPRMatrix(sTransInv, sTrans);
					
                    std::vector<collision_data>& cdata = collisionPairInserter->collisions();
                    cdata.clear();

                    for (int i = 0; i < TouchedPrimCount; i++) {

                        int vertex_index[3];						
                        std::vector<IceMaths::Point> vertex(3);

                        float x, y, z;
                        float R;
						
                        mesh->getTriangle(TouchedPrim[i], vertex_index[0], vertex_index[1], vertex_index[2]);

                        for (int j = 0; j < 3; j++) {
                            mesh->getVertex(vertex_index[j], x, y, z);
                            TransformPoint4x3(vertex[j], IceMaths::Point(x, y, z), *(mesh->transform));
                        }
					
                        triangle[i] = std::vector<IceMaths::Point> (vertex);
						
                        face[i] = IceMaths::Plane(vertex[0], vertex[1], vertex[2]);
                        face[i].Normalize();
						
                        IceMaths::Plane face_s;		// Plane of each face in sphere's coordinates
                        IceMaths::TransformPlane(face_s, face[i], sTransInv);
                        face_s.Normalize();
						
                        if (abs(face_s.d) > radius)
                            cout << "No intersection";
                        else {

                            R = sqrt(pow(radius, 2) - pow(face_s.d, 2));
                            depth[i] = radius - abs(face_s.d);

                            IceMaths::Point U, V;

                            TransformPoint3x3(U, vertex[1] - vertex[0], sTransInv);
                            U.Normalize();
                            V = face_s.n ^ U;
                            V.Normalize();
							
                            IceMaths::Matrix4x4 scTrans;							
                            scTrans.SetRow(0, U);
                            scTrans.SetRow(1, V);
                            scTrans.SetRow(2, face_s.n);
                            scTrans.SetRow(3, face_s.n * -face_s.d);
							
                            IceMaths::Matrix4x4 scTransInv;
                            IceMaths::InvertPRMatrix(scTransInv, scTrans);

                            IceMaths::Point vertex_c[3];
                            std::vector<float> vx, vy;
							
                            for (int j = 0; j < 3; j++) {
                                TransformPoint4x3(vertex_c[j], vertex[j], sTransInv * scTransInv);
                                vx.push_back(vertex_c[j].x);
                                vy.push_back(vertex_c[j].y);
                            }
							
                            float cx, cy;
                            calculateCentroidIntersection(cx, cy, A[i], R, vx, vy);
							
                            TransformPoint4x3(q[i], IceMaths::Point (cx, cy, 0), scTrans * sTrans);
                        }
                    }

                    std::vector<bool> considered_checklist(TouchedPrimCount, false);
                    std::vector<int> sameplane;
					
                    std::vector<IceMaths::Point> new_q;
                    std::vector<IceMaths::Point> new_n;
                    std::vector<float> new_depth;
					
                    // The following procedure is needed to merge components from the same plane (but different triangles)

                    for (int i = 0; i < TouchedPrimCount; i++) {

                        if (!considered_checklist[i]) {

                            for (int j = i + 1; j < TouchedPrimCount; j++) {
                                IceMaths::Point normdiff(face[i].n - face[j].n);
                                if (normdiff.Magnitude() < LOCAL_EPSILON && (face[i].d - face[j].d) < LOCAL_EPSILON) {
                                    if (!sameplane.size()) sameplane.push_back(i);	// In order to consider it just once
                                    sameplane.push_back(j);
                                }
                            }
							
                            if (!sameplane.size()) {
                                new_q.push_back(q[i]);
                                new_n.push_back(face[i].n);
                                new_depth.push_back(depth[i]);
                                considered_checklist[i] = true;
                            }
                            else {

                                float sum_xA, sum_yA, sum_zA, sum_A;
                                sum_xA = sum_yA = sum_zA = sum_A = 0;
		
                                for (int k = 0; k < sameplane.size(); k++) {
                                    sum_xA += q[sameplane[k]].x * A[sameplane[k]];
                                    sum_yA += q[sameplane[k]].y * A[sameplane[k]];
                                    sum_zA += q[sameplane[k]].z * A[sameplane[k]];
                                    sum_A  += A[sameplane[k]];
                                    considered_checklist[sameplane[k]] = true;
                                }
							
                                IceMaths::Point q_temp;
                                q_temp.x = sum_xA / sum_A;
                                q_temp.y = sum_yA / sum_A;
                                q_temp.z = sum_zA / sum_A;
                                new_q.push_back(q_temp);
                                new_n.push_back(face[i].n);
                                new_depth.push_back(depth[i]);
							
                                sameplane.clear();
                            }
                        }
                    }
					
                    for (int i = 0; i < new_q.size(); i++) {
                        collision_data col;
                        col.depth = new_depth[i];
                        col.num_of_i_points = 1;
                        col.i_point_new[0] = 1;
                        col.i_point_new[1] = 0;
                        col.i_point_new[2] = 0;
                        col.i_point_new[3] = 0;
                        col.n_vector[0] = sign * new_n[i].x;
                        col.n_vector[1] = sign * new_n[i].y;
                        col.n_vector[2] = sign * new_n[i].z;
                        col.i_points[0][0] = new_q[i].x;
                        col.i_points[0][1] = new_q[i].y;
                        col.i_points[0][2] = new_q[i].z;
                        cdata.push_back(col);
                    }
                }
            }
        }
		
        else
            std::cerr << "SphereCollider::Collide() failed" << std::endl;

    }
	
    return result;
}

bool ColdetModelPair::detectPlaneCylinderCollisions(bool detectAllContacts) {

    ColdetModel* plane = nullptr;
    ColdetModel* cylinder = nullptr;
    
    bool reversed=false;
    if(models[0]->getPrimitiveType() == ColdetModel::SP_PLANE){
        plane = models[0];
    } else if(models[0]->getPrimitiveType() == ColdetModel::SP_CYLINDER){
        cylinder = models[0];
    }
    if(models[1]->getPrimitiveType() == ColdetModel::SP_PLANE){
        plane = models[1];
        reversed = true;
    } else if(models[1]->getPrimitiveType() == ColdetModel::SP_CYLINDER){
        cylinder = models[1];
    }
    if(!plane || !cylinder){
        return false;
    }

    IceMaths::Matrix4x4 pTrans = (*(plane->pTransform)) * (*(plane->transform));
    IceMaths::Matrix4x4 cTrans = (*(cylinder->pTransform)) * (*(cylinder->transform));

    float radius, height; // height and radius of cylinder
    cylinder->getPrimitiveParam(0, radius);
    cylinder->getPrimitiveParam(1, height);

    IceMaths::Point pTopLocal(0, height/2, 0), pBottomLocal(0, -height/2, 0);
    IceMaths::Point pTop, pBottom; // center points of top and bottom discs
    IceMaths::TransformPoint4x3(pTop,    pTopLocal,    cTrans);
    IceMaths::TransformPoint4x3(pBottom, pBottomLocal, cTrans);
    
    IceMaths::Point pOnPlane, nLocal(0,0,1), n;
    IceMaths::TransformPoint3x3(n, nLocal, pTrans);
    pTrans.GetTrans(pOnPlane);
    float d = pOnPlane|n; // distance between origin and plane

    float dTop    = (pTop|n) - d;
    float dBottom = (pBottom|n) - d;

    if (dTop > radius && dBottom > radius) return false;

    double theta = asin((dTop - dBottom)/height);
    double rcosth = radius*cos(theta);

    int contactsCount = 0;
    if (rcosth >= dTop) contactsCount+=2;
    if (rcosth >= dBottom) contactsCount+=2;

    if (contactsCount){
        std::vector<collision_data>& cdata = collisionPairInserter->collisions();
        cdata.resize(contactsCount);
        for (unsigned int i=0; i<contactsCount; i++){
            cdata[i].num_of_i_points = 1;
            cdata[i].i_point_new[0]=1; 
            cdata[i].i_point_new[1]=0; 
            cdata[i].i_point_new[2]=0; 
            cdata[i].i_point_new[3]=0; 
            if (reversed){
                cdata[i].n_vector[0] = -n.x;
                cdata[i].n_vector[1] = -n.y;
                cdata[i].n_vector[2] = -n.z;
            }else{
                cdata[i].n_vector[0] = n.x;
                cdata[i].n_vector[1] = n.y;
                cdata[i].n_vector[2] = n.z;
            }
        }
        IceMaths::Point vBottomTop = pTop - pBottom;
        IceMaths::Point v = vBottomTop^n;
        v.Normalize();
        IceMaths::Point w = v^n;
        w.Normalize();

        unsigned int index=0;
        if (rcosth >= dBottom){ // bottom disc collides
            double depth = rcosth - dBottom;
            IceMaths::Point iPoint = pBottom - dBottom*n - dBottom*tan(theta)*w;
            double x = dBottom/cos(theta);
            IceMaths::Point dv = sqrt(radius*radius - x*x)*v;
            cdata[index].i_points[0][0] = iPoint.x + dv.x;
            cdata[index].i_points[0][1] = iPoint.y + dv.y;
            cdata[index].i_points[0][2] = iPoint.z + dv.z;
            cdata[index].depth = depth;
            index++;
            cdata[index].i_points[0][0] = iPoint.x - dv.x;
            cdata[index].i_points[0][1] = iPoint.y - dv.y;
            cdata[index].i_points[0][2] = iPoint.z - dv.z;
            cdata[index].depth = depth;
            index++;
        }
        if (rcosth >= dTop){ // top disc collides
            double depth = rcosth - dTop;
            IceMaths::Point iPoint = pTop - dTop*n - dTop*tan(theta)*w;
            double x = dTop/cos(theta);
            IceMaths::Point dv = sqrt(radius*radius - x*x)*v;
            cdata[index].i_points[0][0] = iPoint.x + dv.x;
            cdata[index].i_points[0][1] = iPoint.y + dv.y;
            cdata[index].i_points[0][2] = iPoint.z + dv.z;
            cdata[index].depth = depth;
            index++;
            cdata[index].i_points[0][0] = iPoint.x - dv.x;
            cdata[index].i_points[0][1] = iPoint.y - dv.y;
            cdata[index].i_points[0][2] = iPoint.z - dv.z;
            cdata[index].depth = depth;
            index++;
        }

        return true;
    }
    return false;
}


double ColdetModelPair::computeDistance(ColdetModel* model0, ColdetModel* model1, double* point0, double* point1)
{
    if(model0->isValid() && model1->isValid()){

        Opcode::BVTCache colCache;

        colCache.Model0 = &model1->internalModel->model;
        colCache.Model1 = &model0->internalModel->model;
        
        Opcode::SSVTreeCollider collider;
        
        float d;
        Point p0, p1;
        collider.Distance(colCache, d, p0, p1,
                          model1->transform, model0->transform);
        point0[0] = p1.x;
        point0[1] = p1.y;
        point0[2] = p1.z;
        point1[0] = p0.x;
        point1[1] = p0.y;
        point1[2] = p0.z;
        return d;
    }

    return -1.0;
}


double ColdetModelPair::computeDistance(double* point0, double* point1)
{
    return computeDistance(models[0], models[1], point0, point1);
}
    

double ColdetModelPair::computeDistance(int& triangle0, double* point0, int& triangle1, double* point1)
{
    if(models[0]->isValid() && models[1]->isValid()){

        Opcode::BVTCache colCache;

        colCache.Model0 = &models[1]->internalModel->model;
        colCache.Model1 = &models[0]->internalModel->model;
        
        Opcode::SSVTreeCollider collider;
        
        float d;
        Point p0, p1;
        collider.Distance(colCache, d, p0, p1,
                          models[1]->transform, models[0]->transform);
        point0[0] = p1.x;
        point0[1] = p1.y;
        point0[2] = p1.z;
        point1[0] = p0.x;
        point1[1] = p0.y;
        point1[2] = p0.z;
        triangle1 = colCache.id0;
        triangle0 = colCache.id1;
        return d;
    }

    return -1.0;
}


bool ColdetModelPair::detectIntersection()
{
    if(models[0]->isValid() && models[1]->isValid()){

        Opcode::BVTCache colCache;

        colCache.Model0 = &models[1]->internalModel->model;
        colCache.Model1 = &models[0]->internalModel->model;
        
        Opcode::SSVTreeCollider collider;
        
        return collider.Collide(colCache, tolerance_, 
                                models[1]->transform, models[0]->transform);
    }

    return false;
}

void ColdetModelPair::setCollisionPairInserter(Opcode::CollisionPairInserter* inserter)
{
    delete collisionPairInserter;
    collisionPairInserter = inserter;
    // inverse order because of historical background
    // this should be fixed.(note that the direction of normal is inversed when the order inversed 
    collisionPairInserter->set(models[1]->internalModel, models[0]->internalModel);
}

int ColdetModelPair::calculateCentroidIntersection(float &cx, float &cy, float &A, float radius, std::vector<float> vx, std::vector<float> vy) {
	
    int i;		// Vertex and Side
    int j[5];	// Point ID
    int k;		// Section
	
    int isOk = ColdetModelPair::makeCCW(vx, vy);
	
    if (isOk) {
	
        std::vector<pointStruct> point;
        pointStruct p;
        int numInter;
        std::vector<float> x_int(2), y_int(2);
		
        for (i = 0; i < vx.size(); i++) {
			
            // Recording of the vertex
			
            p.x = vx[i];
            p.y = vy[i];
            p.angle = atan2(vy[i], vx[i]);
            if (p.angle < 0) p.angle += TWOPI;
            p.type = vertex;
			
            p.code = isInsideCircle(radius, p.x, p.y);
            point.push_back(p);
			
            // Recording of the intersections

            numInter = calculateIntersection(x_int, y_int, radius, vx[i], vy[i], vx[(i + 1) % vx.size()], vy[(i + 1) % vx.size()]);
			
            if (numInter){
                for (k = 0; k < numInter; k++) {
                    p.x = x_int[k];
                    p.y = y_int[k];
                    p.angle = atan2(y_int[k], x_int[k]);					
                    if (p.angle < 0) p.angle += TWOPI;
                    p.type = inter;
                    p.code = i + 1;
                    point.push_back(p);
                }
            }
            numInter = 0;
        }
		
        j[0] = 0;
		
        int start = -1;
        bool finished = false;
		
        std::vector<figStruct> figure;
        figStruct f;

        while (!finished) {

            for (int cont = 1; cont <= 4; cont++)
                j[cont] = (j[0] + cont) % point.size();

            if (point[j[0]].code) {
				
                if (start == -1) start = j[0];
				
                if (point[j[1]].code) {
					
                    f.p1 = j[0];
                    f.p2 = j[1];
                    f.type = tri;
                    figure.push_back(f);
                    j[0] = f.p2;
                }
				
                else if (point[j[2]].code || point[j[3]].code || point[j[4]].code) {
					
                    f.type = sector;
                    f.p1 = j[0];
					
                    if	(point[j[2]].code) f.p2 = j[2];
                    else if (point[j[3]].code) f.p2 = j[3];
                    else if (point[j[4]].code) f.p2 = j[4];
										
                    figure.push_back(f);
                    j[0] = f.p2;
                }
				
                else {
                    cout << "Error: No intersection detected" << endl;
                    return 0;
                }
            }
			
            else {
                j[0] = j[1];
            }
			
            if (((j[0] == 0) && (start == -1)) || (j[0] == start))
                finished = true;
        }
		
        if (figure.size()) {
		
            std::vector<float> x(3, 0);
            std::vector<float> y(3, 0);
            float sumx, sumy;
            float th;
		
            for (k = 0; k < figure.size(); k++) {
                if (figure[k].type == tri) {
                    x[1] = point[figure[k].p1].x;
                    y[1] = point[figure[k].p1].y;
                    x[2] = point[figure[k].p2].x;
                    y[2] = point[figure[k].p2].y;
                    figure[k].area = calculatePolygonArea(x, y);
                    sumx = sumy = 0;
                    for (int cont = 0; cont < 3; cont++) {
                        sumx += x[cont];
                        sumy += y[cont];
                    }
                    figure[k].cx = sumx / 3;
                    figure[k].cy = sumy / 3;
                }
                else if (figure[k].type == sector) {
                    th = point[figure[k].p2].angle - point[figure[k].p1].angle;
                    if (th < 0) th += TWOPI;
                    figure[k].area = pow(radius, 2) * th / 2;
                    calculateSectorCentroid(figure[k].cx, figure[k].cy, radius, point[figure[k].p1].angle, point[figure[k].p2].angle);
                }
            }

            float sum_xA, sum_yA, sum_A;
            sum_xA = sum_yA = sum_A = 0;
		
            for (k = 0; k < figure.size(); k++) {
                sum_xA += figure[k].cx * figure[k].area;
                sum_yA += figure[k].cy * figure[k].area;
                sum_A  += figure[k].area;
            }
			
            if ((figure.size() == 1) && (sum_A == 0)) {
                cx = point[figure[0].p1].x;
                cy = point[figure[0].p1].y;
            }
            else {
                cx = sum_xA / sum_A;
                cy = sum_yA / sum_A;
            }

            A = sum_A;
			
            return 1;
        }

        else {
            if (isInsideTriangle(0, 0, vx, vy)) {
                cx = cy = 0;
                A = TWOPI * pow(radius, 2);
                return 1;
            }
            else{
                cx = cy = 0;
                A = TWOPI * pow(radius, 2);
                return 0;
            }
        }
    }

    else
        return 0;
}

int ColdetModelPair::makeCCW(std::vector<float> &vx, std::vector<float> &vy) {
	
    float vx_tmp, vy_tmp;

    if ((vx.size() == 3) && (vy.size() == 3)) {
        if (ColdetModelPair::calculatePolygonArea(vx, vy) < 0)	{
            vx_tmp = vx[0];
            vy_tmp = vy[0];
            vx[0] = vx[1];
            vy[0] = vy[1];
        }
        return 1;
    }
    else {
        cout << "The number of vertices does not correspond to a triangle" << endl;
        return 0;
    }
}

float ColdetModelPair::calculatePolygonArea(const std::vector<float> &vx, const std::vector<float> &vy) {
	
    float area = 0;
	
    if (vx.size() == vy.size()) {
        for (int i = 0; i < vx.size(); i++) {
            area += vx[i] * vy[(i + 1) % vx.size()] - vy[i] * vx[(i + 1) % vx.size()];
        }
        return area / 2;
    }
    else {
        cout << "The number of coordinates does not match" << endl;
        return 0;
    }
}

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#define trunc(x) ((int)(x))
#endif 
void ColdetModelPair::calculateSectorCentroid(float &cx, float &cy, float radius, float th1, float th2) {
	
    float th, psi, phi, g;

    th = th2 - th1;
    if (th2 < th1) th += TWOPI;	
	
    g = (abs(th) > LOCAL_EPSILON) ? 4.0 / 3.0 * radius / th * sin(th / 2) : 2.0 / 3.0 * radius;
	
    psi = th1 + th2;
    if (th2 < th1) psi += TWOPI;
	
    phi = psi / 2 - trunc(psi / 2 / TWOPI) * TWOPI;

    cx = g * cos(phi);
    cy = g * sin(phi);
}

bool ColdetModelPair::isInsideTriangle(float x, float y, const std::vector<float> &vx, const std::vector<float> &vy) {
	
    IceMaths::Point v1, v2;
    double m1, m2;
    double anglesum = 0;

    for (int i = 0; i < 3; i++) {
	
        v1 = IceMaths::Point(vx[i], vy[i], 0) - IceMaths::Point(x, y, 0);
        v2 = IceMaths::Point(vx[(i + 1) % vx.size()], vy[(i + 1) % vy.size()], 0) - IceMaths::Point(x, y, 0);
	
        m1 = v1.Magnitude();
        m2 = v2.Magnitude();

        if (m1 * m2 <= LOCAL_EPSILON) {
            anglesum = TWOPI;
            break;
        }
        else
            anglesum += acos((v1 | v2) / (m1 * m2));
    }

    return (abs(TWOPI - anglesum) < LOCAL_EPSILON);
}

int ColdetModelPair::calculateIntersection(std::vector<float> &x, std::vector<float> &y, float radius, float x1, float y1, float x2, float y2) {
	
    int numint;

    float x_test, y_test;
    x.clear();
    y.clear();

    float xmin = min(x1, x2);
    float xmax = max(x1, x2);
    float ymin = min(y1, y2);
    float ymax = max(y1, y2);

    float v_norm, proy_norm;
    float x_temp, y_temp;

    std::vector<float> t;

    if ((sqrt(pow(x1, 2) + pow(y1, 2)) != radius) && (sqrt(pow(x2, 2) + pow(y2, 2)) != radius)) {

        float m, b;		
        float D;

        if (abs(x2 - x1) > LOCAL_EPSILON) {
			
            m = (y2 - y1) / (x2 - x1);
            b = y1 - m * x1;

            D = 4 * pow(m, 2) * pow(b, 2) - 4 * (1 + pow(m, 2)) * (pow(b, 2) - pow(radius, 2));
        }
        else
            D = pow(radius, 2) - pow(x1, 2);

        numint = D < 0 ? 0 : (D > 0 ? 2 : 1);

        if (numint > 0) {

            for (int i = 0; i < numint; i++) {

                if (abs(x2 - x1) > LOCAL_EPSILON) {
                    x_test = (-2 * m * b + pow(-1.0, i) * sqrt(D)) / (2 * (1 + pow(m, 2)));
                    y_test = m * x_test + b;
                }
                else {
                    x_test = x1;
                    y_test = pow(-1.0, i) * sqrt(D);
                }
				
                cout.flush();
				
                if ((xmin <= x_test) && (x_test <= xmax) && (ymin <= y_test) && (y_test <= ymax)) {
                    x.push_back(x_test);
                    y.push_back(y_test);
                    v_norm = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
                    proy_norm = sqrt(pow(x_test - x1, 2) + pow(y_test - y1, 2));
                    t.push_back(proy_norm / v_norm);
                }				
            }

            if (t.size() > 1) {
                if (t[0] > t[1]) {
                    x_temp = x[0];
                    y_temp = y[0];
                    x[0] = x[1];
                    y[0] = y[1];
                    x[1] = x_temp;
                    y[1] = y_temp;
                }
            }
        }
    }

    return t.size();
}
