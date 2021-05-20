/**
   @author Shin'ichiro Nakaoka
*/

#include "ColdetModel.h"
#include "ColdetModelInternalModel.h"
#include "Opcode/Opcode.h"
#include <map>
#include <iostream>

using namespace std;
using namespace cnoid;

namespace {

class Edge
{
    int vertex[2];
public :
    Edge(int vertexIndex1, int vertexIndex2){
        if(vertexIndex1 < vertexIndex2){
            vertex[0] = vertexIndex1;
            vertex[1] = vertexIndex2;
        } else {
            vertex[0] = vertexIndex2;
            vertex[1] = vertexIndex1;
        }
    }
    bool operator<(const Edge& rhs) const {
        if(vertex[0] < rhs.vertex[0]){
            return true;
        } else if(vertex[0] == rhs.vertex[0]){
            return (vertex[1] < rhs.vertex[1]);
        } else {
            return false;
        }
    }
};

struct trianglePair {
    int t[2];
};

typedef std::map< Edge, trianglePair > EdgeToTriangleMap;
}


ColdetModel::ColdetModel()
{
    internalModel = new ColdetModelInternalModel();
    isValid_ = false;
    initialize();
}


ColdetModel::ColdetModel(const ColdetModel& org)
    : name_(org.name_),
      isValid_(org.isValid_)
{
    internalModel = org.internalModel;
    initialize();
}


void ColdetModel::initialize()
{
    internalModel->refCounter++;

    transform = new IceMaths::Matrix4x4();
    transform->Identity();

    pTransform = new IceMaths::Matrix4x4();
    pTransform->Identity();
}


ColdetModel* ColdetModel::clone() const
{
    return new ColdetModel(*this);
}


void ColdetModel::cloneInternalModel()
{
    ColdetModelInternalModel* oldInternalModel = internalModel;
    internalModel = new ColdetModelInternalModel();
    internalModel->refCounter++;

    internalModel->vertices = oldInternalModel->vertices;
    internalModel->triangles = oldInternalModel->triangles;

    build();

    if(--oldInternalModel->refCounter <= 0){
        delete oldInternalModel;
    }
}


ColdetModelInternalModel::ColdetModelInternalModel()
{
    refCounter = 0;
    pType = ColdetModel::SP_MESH;
    AABBTreeMaxDepth=0;
}    


ColdetModel::~ColdetModel()
{
    if(--internalModel->refCounter <= 0){
        delete internalModel;
    }
    delete pTransform;
    delete transform;
}


void ColdetModel::setNumVertices(int n)
{
    internalModel->vertices.resize(n);
}


int ColdetModel::getNumVertices() const
{
    return internalModel->vertices.size();
}


void ColdetModel::setNumTriangles(int n)
{
    internalModel->triangles.resize(n);
}


int ColdetModel::getNumTriangles() const
{
    return internalModel->triangles.size();
}

        
void ColdetModel::setVertex(int index, float x, float y, float z)
{
    internalModel->vertices[index].Set(x, y, z);
}


void ColdetModel::addVertex(float x, float y, float z)
{
    internalModel->vertices.push_back(IceMaths::Point(x, y, z));
}

        
void ColdetModel::getVertex(int index, float& x, float& y, float& z) const
{
    const Point& v = internalModel->vertices[index];
    x = v.x;
    y = v.y;
    z = v.z;
}

        
void ColdetModel::setTriangle(int index, int v1, int v2, int v3)
{
    udword* mVRef = internalModel->triangles[index].mVRef;
    mVRef[0] = v1;
    mVRef[1] = v2;
    mVRef[2] = v3;
}

void ColdetModel::getTriangle(int index, int& v1, int& v2, int& v3) const
{
    udword* mVRef = internalModel->triangles[index].mVRef;
    v1=mVRef[0];
    v2=mVRef[1];
    v3=mVRef[2];
}


void ColdetModel::addTriangle(int v1, int v2, int v3)
{
    internalModel->triangles.push_back(IceMaths::IndexedTriangle(v1, v2, v3));
}


void ColdetModel::build()
{
    isValid_ = internalModel->build();
    /*
      unsigned int maxDepth = internalModel->getAABBTreeDepth();
      for(unsigned int i=0; i<maxDepth; i++){
      vector<IceMaths::Point> data = getBoundingBoxData(i);   
      cout << "depth= " << i << endl;
      for(vector<IceMaths::Point>::iterator it=data.begin(); it!=data.end(); it++){
      cout << (*it).x << " " << (*it).y << " " << (*it).z << endl;
      }
      }
    */
}


int ColdetModel::numofBBtoDepth(int minNumofBB)
{
    for(int i=0; i < getAABBTreeDepth(); ++i){
        if(minNumofBB <= internalModel->getNumofBB(i)){
            return i;
        }
    }
    return getAABBTreeDepth();
}


int ColdetModel::getAABBTreeDepth()
{
    return internalModel->getAABBTreeDepth();
}


int ColdetModel::getAABBmaxNum()
{
    return internalModel->getmaxNumofBB();
}


static void getBoundingBoxDataSub
(const Opcode::AABBCollisionNode* node, unsigned int currentDepth, unsigned int depth, std::vector<Vector3>& out_data)
{
    if(currentDepth == depth || node->IsLeaf() ){
        const IceMaths::Point& p = node->mAABB.mCenter;
        out_data.push_back(Vector3(p.x, p.y, p.z));
        const IceMaths::Point& q = node->mAABB.mExtents;
        out_data.push_back(Vector3(q.x, q.y, q.z));
    }
    currentDepth++;
    if(currentDepth > depth){
        return;
    }
    if(!node->IsLeaf()){
        getBoundingBoxDataSub(node->GetPos(), currentDepth, depth, out_data);
        getBoundingBoxDataSub(node->GetNeg(), currentDepth, depth, out_data);
    }
}


void ColdetModel::getBoundingBoxData(const int depth, std::vector<Vector3>& out_data)
{
    const Opcode::AABBCollisionNode* rootNode=((Opcode::AABBCollisionTree*)internalModel->model.GetTree())->GetNodes();
    out_data.clear();
    getBoundingBoxDataSub(rootNode, 0, depth, out_data);
}


bool ColdetModelInternalModel::build()
{
    bool result = false;

    neighbors.clear();
    
    if(triangles.size() > 0){

        extractNeghiborTriangles();

        Opcode::OPCODECREATE OPCC;

        iMesh.SetPointers(&triangles[0], &vertices[0]);
        iMesh.SetNbTriangles(triangles.size());
        iMesh.SetNbVertices(vertices.size());
        
        OPCC.mIMesh = &iMesh;
        
        OPCC.mNoLeaf = false;
        OPCC.mQuantized = false;
        OPCC.mKeepOriginal = false;
        
        model.Build(OPCC);
        if(model.GetTree()){
            AABBTreeMaxDepth = computeDepth(((Opcode::AABBCollisionTree*)model.GetTree())->GetNodes(), 0, -1) + 1;
            for(int i=0; i<AABBTreeMaxDepth; i++)
                for(int j=0; j<i; j++)
                    numBBMap.at(i) += numLeafMap.at(j);
        }
        result = true;
    }

    return result;
}


void ColdetModel::setPosition(const Isometry3& T)
{
    transform->Set((float)T(0,0), (float)T(1,0), (float)T(2,0), 0.0f,
                   (float)T(0,1), (float)T(1,1), (float)T(2,1), 0.0f,
                   (float)T(0,2), (float)T(1,2), (float)T(2,2), 0.0f,
                   (float)T(0,3), (float)T(1,3), (float)T(2,3), 1.0f);
}    


#ifdef CNOID_BACKWARD_COMPATIBILITY
void ColdetModel::setPosition(const Matrix3& R, const Vector3& p)
{
    transform->Set((float)R(0,0), (float)R(1,0), (float)R(2,0), 0.0f,
                   (float)R(0,1), (float)R(1,1), (float)R(2,1), 0.0f,
                   (float)R(0,2), (float)R(1,2), (float)R(2,2), 0.0f,
                   (float)p(0),   (float)p(1),   (float)p(2),   1.0f);
}
#endif


void ColdetModel::setPosition(const double* R, const double* p)
{
    transform->Set((float)R[0], (float)R[3], (float)R[6], 0.0f,
                   (float)R[1], (float)R[4], (float)R[7], 0.0f,
                   (float)R[2], (float)R[5], (float)R[8], 0.0f,
                   (float)p[0], (float)p[1], (float)p[2], 1.0f);
}


void ColdetModel::setPrimitiveType(PrimitiveType ptype)
{
    internalModel->pType = ptype;
}


ColdetModel::PrimitiveType ColdetModel::getPrimitiveType() const
{
    return internalModel->pType;
}


void ColdetModel::setNumPrimitiveParams(unsigned int nparam)
{
    internalModel->pParams.resize(nparam);
}


bool ColdetModel::setPrimitiveParam(unsigned int index, float value)
{
    if (index >= internalModel->pParams.size()) return false;

    internalModel->pParams[index] = value;
    return true;
}


bool ColdetModel::getPrimitiveParam(unsigned int index, float& value) const
{
    if (index >= internalModel->pParams.size()) return false;

    value = internalModel->pParams[index];
    return true;
}


void ColdetModel::setPrimitivePosition(const double* R, const double* p)
{
    pTransform->Set((float)R[0], (float)R[3], (float)R[6], 0.0f,
                    (float)R[1], (float)R[4], (float)R[7], 0.0f,
                    (float)R[2], (float)R[5], (float)R[8], 0.0f,
                    (float)p[0], (float)p[1], (float)p[2], 1.0f);
}


double ColdetModel::computeDistanceWithRay(const double *point, 
                                           const double *dir)
{
    Opcode::RayCollider RC;
    Ray world_ray(Point(point[0], point[1], point[2]),
                  Point(dir[0], dir[1], dir[2]));
    Opcode::CollisionFace CF;
    Opcode::SetupClosestHit(RC, CF);
    udword Cache;
    RC.Collide(world_ray, internalModel->model, transform, &Cache);
    if (CF.mDistance == FLT_MAX){
        return 0;
    }else{
        return CF.mDistance;
    }
}


bool ColdetModel::checkCollisionWithPointCloud(const std::vector<Vector3> &i_cloud, double i_radius)
{
    Opcode::SphereCollider SC;
    SC.SetFirstContact(true);
    Opcode::SphereCache Cache;
    IceMaths::Point p(0,0,0);
    IceMaths::Sphere sphere(p, i_radius);
    IceMaths::Matrix4x4 sphereTrans(1,0,0,0, 0,1,0,0, 0,0,1,0,  0,0,0,1);
    for (unsigned int i=0; i<i_cloud.size(); i++){
        const Vector3& p = i_cloud[i];
        sphereTrans.m[3][0] = p[0];
        sphereTrans.m[3][1] = p[1];
        sphereTrans.m[3][2] = p[2];
        bool isOk = SC.Collide(Cache, sphere, internalModel->model, &sphereTrans, transform); 
        if (!isOk) std::cerr << "SphereCollider::Collide() failed" << std::endl;
        if (SC.GetContactStatus()) return true;
    }
    return false;
}


namespace {

inline bool extractNeighborTriangle
(ColdetModelInternalModel::NeighborTriangleSetArray& neighbors, EdgeToTriangleMap& triangleMap,
 int triangle, int vertex1, int vertex2)
{
    Edge edge(vertex1, vertex2);
        
    EdgeToTriangleMap::iterator p = triangleMap.find(edge);
    if(p == triangleMap.end()){
        triangleMap[edge].t[0] = triangle;
        triangleMap[edge].t[1] = -1;
        return true;
    } else {
        trianglePair& triangles = p->second;
        if( triangles.t[1] != -1 ){
            neighbors[triangles.t[0]].deleteNeighbor(triangles.t[1]);
            neighbors[triangles.t[1]].deleteNeighbor(triangles.t[0]);
            //cout << "neighbors[" << triangles.t[0] << "] " << neighbors[triangles.t[0]][0] << " " << neighbors[triangles.t[0]][1] << " " << neighbors[triangles.t[0]][2] << endl;
            //cout << "neighbors[" << triangles.t[1] << "] " << neighbors[triangles.t[1]][0] << " " << neighbors[triangles.t[1]][1] << " " << neighbors[triangles.t[1]][2] << endl;
            return false;
        }else{
            neighbors[triangle].addNeighbor(triangles.t[0]);
            neighbors[triangles.t[0]].addNeighbor(triangle);
            triangles.t[1] = triangle;
            //cout << "neighbors[" << triangle << "] " << neighbors[triangle][0] << " " << neighbors[triangle][1] << " " << neighbors[triangle][2] << endl;
            //cout << "neighbors[" << triangles.t[0] << "] " << neighbors[triangles.t[0]][0] << " " << neighbors[triangles.t[0]][1] << " " << neighbors[triangles.t[0]][2] << endl;
            return true;
        }
    }
}
}
        

void ColdetModelInternalModel::extractNeghiborTriangles()
{
    const int numTriangles = triangles.size();
    neighbors.resize(numTriangles);

    EdgeToTriangleMap edgeToExistingTriangleMap;

    bool ret = true;
    for(int i=0; i < numTriangles; ++i){
        udword* triangle = triangles[i].mVRef;
        ret &= extractNeighborTriangle(neighbors, edgeToExistingTriangleMap, i, triangle[0], triangle[1]);
        ret &= extractNeighborTriangle(neighbors, edgeToExistingTriangleMap, i, triangle[1], triangle[2]);
        ret &= extractNeighborTriangle(neighbors, edgeToExistingTriangleMap, i, triangle[2], triangle[0]);
    }

#ifndef NDEBUG
    if(!ret)
        cout << "Warning : Three or more triangles are defined for a edge." << endl;
#endif

}


int ColdetModelInternalModel::computeDepth(const Opcode::AABBCollisionNode* node, int currentDepth, int max)
{
    /*
      cout << "depth= " << currentDepth << " ";
      Point p = node->mAABB.mCenter;
      cout << p.x << " " << p.y << " " << p.z << "     ";
      p = node->mAABB.mExtents;
      cout << p.x << " " << p.y << " " << p.z << " ";
      if(node->IsLeaf()) cout << "is Leaf " ;
      cout << endl;
    */
    if(max < currentDepth){
        max = currentDepth;
        numBBMap.push_back(0);
        numLeafMap.push_back(0);
    }
    numBBMap.at(currentDepth)++;

    if(!node->IsLeaf()){
        currentDepth++;
        max = computeDepth(node->GetPos(), currentDepth, max);
        max = computeDepth(node->GetNeg(), currentDepth, max);
    } else {
        numLeafMap.at(currentDepth)++;
    }

    return max;
}
