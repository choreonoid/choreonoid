/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_AIST_COLLISION_DETECTOR_COLDET_MODEL_INTERNAL_MODEL_H
#define CNOID_AIST_COLLISION_DETECTOR_COLDET_MODEL_INTERNAL_MODEL_H

#include "ColdetModel.h"
#include "Opcode/Opcode.h"
#include <vector>

namespace cnoid {

class ColdetModelInternalModel
{
public:
    struct NeighborTriangleSet {
        int neighbors[3];
        NeighborTriangleSet(){
            neighbors[0] = neighbors[1] = neighbors[2] = -1;
        }
        void addNeighbor(int neighbor){
            for(int i=0; i < 3; ++i){
                if(neighbors[i] < 0){
                    neighbors[i] = neighbor;
                    break;
                }
            }
        }
        void deleteNeighbor(int neighbor){
            for(int i=0; i<3; i++){
                if(neighbors[i]==neighbor){
                    for(int j=i+1; j<3; j++){
                        neighbors[j-1] = neighbors[j];
                    }
                    neighbors[2] = -1;
                    break;
                }
            }
        }
        int operator[](int index) const { return neighbors[index]; }
    };

    typedef std::vector<NeighborTriangleSet> NeighborTriangleSetArray;

    ColdetModelInternalModel();

    bool build();

    // need two instances ?
    Opcode::Model model;
    Opcode::MeshInterface iMesh;
    std::vector<IceMaths::Point> vertices;
    std::vector<IceMaths::IndexedTriangle> triangles;
    NeighborTriangleSetArray neighbors;
    ColdetModel::PrimitiveType pType;
    std::vector<float> pParams;

    int getAABBTreeDepth() {
        return AABBTreeMaxDepth;
    };
    int getNumofBB(int depth){
        return numBBMap.at(depth);
    };
    int getmaxNumofBB(){
        if(AABBTreeMaxDepth>0){
            return numBBMap.at(AABBTreeMaxDepth-1);
        } else {
            return 0;
        }
    };

private:
    int refCounter;
    int AABBTreeMaxDepth;
    std::vector<int> numBBMap;
    std::vector<int> numLeafMap;

    void extractNeghiborTriangles();
    int computeDepth(const Opcode::AABBCollisionNode* node, int currentDepth, int max );

    friend class ColdetModel;
};
}

#endif
