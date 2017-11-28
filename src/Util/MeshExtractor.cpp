/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "MeshExtractor.h"
#include "SceneDrawables.h"
#include "PolymorphicFunctionSet.h"

using namespace cnoid;

namespace cnoid {

class MeshExtractorImpl
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PolymorphicFunctionSet<SgNode> functions;
    std::function<void()> callback;
    SgMesh* currentMesh;
    Affine3 currentTransform;
    Affine3 currentTransformWithoutScaling;
    bool isCurrentScaled;
    bool meshFound;
    
    MeshExtractorImpl();
    void visitGroup(SgGroup* group);
    void visitSwitch(SgSwitch* switchNode);    
    void visitTransform(SgTransform* transform);
    void visitPosTransform(SgPosTransform* transform);
    void visitShape(SgShape* shape);
};

}


MeshExtractor::MeshExtractor()
{
    impl = new MeshExtractorImpl;
}


MeshExtractorImpl::MeshExtractorImpl()
{
    functions.setFunction<SgGroup>(
        [&](SgGroup* node){ visitGroup(node); });
    functions.setFunction<SgSwitch>(
        [&](SgSwitch* node){ visitSwitch(node); });
    functions.setFunction<SgTransform>(
        [&](SgTransform* node){ visitTransform(node); });
    functions.setFunction<SgPosTransform>(
        [&](SgPosTransform* node){ visitPosTransform(node); });
    functions.setFunction<SgShape>(
        [&](SgShape* node){ visitShape(node); });
    functions.updateDispatchTable();
}
    
    
void MeshExtractorImpl::visitGroup(SgGroup* group)
{
    for(SgGroup::const_iterator p = group->begin(); p != group->end(); ++p){
        functions.dispatch(*p);
    }
}


void MeshExtractorImpl::visitSwitch(SgSwitch* switchNode)
{
    if(switchNode->isTurnedOn()){
        visitGroup(switchNode);
    }
}
    

void MeshExtractorImpl::visitTransform(SgTransform* transform)
{
    bool isParentScaled = isCurrentScaled;
    isCurrentScaled = true;
    Affine3 T0 = currentTransform;
    Affine3 T;
    transform->getTransform(T);
    currentTransform = T0 * T;
    visitGroup(transform);
    currentTransform = T0;
    isCurrentScaled = isParentScaled;
}


void MeshExtractorImpl::visitPosTransform(SgPosTransform* transform)
{
    const Affine3 T0(currentTransform);
    const Affine3 P0(currentTransformWithoutScaling);
    currentTransform = T0 * transform->T();
    currentTransformWithoutScaling = P0 * transform->T();
    visitGroup(transform);
    currentTransform = T0;
    currentTransformWithoutScaling = P0;
}


void MeshExtractorImpl::visitShape(SgShape* shape)
{
    SgMesh* mesh = shape->mesh();
    if(mesh && mesh->vertices() && !mesh->vertices()->empty() && !mesh->triangleVertices().empty()){
        meshFound = true;
        currentMesh = mesh;
        callback();
        currentMesh = 0;
    }
}


SgMesh* MeshExtractor::currentMesh() const
{
    return impl->currentMesh;
}


const Affine3& MeshExtractor::currentTransform() const
{
    return impl->currentTransform;
}


const Affine3& MeshExtractor::currentTransformWithoutScaling() const
{
    return impl->currentTransformWithoutScaling;
}


bool MeshExtractor::isCurrentScaled() const
{
    return impl->isCurrentScaled;
}


bool MeshExtractor::extract(SgNode* node, std::function<void()> callback)
{
    impl->callback = callback;
    impl->currentMesh = 0;
    impl->currentTransform.setIdentity();
    impl->currentTransformWithoutScaling.setIdentity();
    impl->isCurrentScaled = false;
    impl->meshFound = false;
    impl->functions.dispatch(node);
    return impl->meshFound;
}


static void integrateMesh(MeshExtractor* extractor, SgMesh* mesh)
{
    SgMesh* srcMesh = extractor->currentMesh();
    const Affine3f T = extractor->currentTransform().cast<Affine3f::Scalar>();

    if(srcMesh->hasVertices()){
        SgVertexArray& vertices = *mesh->getOrCreateVertices();
        const int numVertices = vertices.size();
        SgVertexArray& srcVertices = *srcMesh->vertices();
        const int numSrcVertices = srcVertices.size();
        vertices.reserve(numVertices + numSrcVertices);
        for(int i=0; i < numSrcVertices; ++i){
            vertices.push_back(T * srcVertices[i]);
        }

        SgIndexArray& indices = mesh->triangleVertices();
        const int numIndices = indices.size();
        SgIndexArray& srcIndices = srcMesh->triangleVertices();
        const int numSrcIndices = srcIndices.size();
        indices.reserve(numIndices + numSrcIndices);
        for(int i=0; i < numSrcIndices; ++i){
            indices.push_back(srcIndices[i] + numVertices);
        }
        
        if(srcMesh->hasNormals()){
            SgNormalArray& normals = *mesh->getOrCreateNormals();
            const int numNormals = normals.size();
            SgNormalArray& srcNormals = *srcMesh->normals();
            const int numSrcNormals = srcNormals.size();
            normals.reserve(numNormals + numSrcNormals);
            const Affine3f U = extractor->currentTransformWithoutScaling().cast<Affine3f::Scalar>();
            for(int i=0; i < numSrcNormals; ++i){
                normals.push_back(U * srcNormals[i]);
            }
            
            SgIndexArray& indices = mesh->normalIndices();
            const int numIndices = indices.size();
            SgIndexArray& srcIndices = srcMesh->normalIndices();
            const int numSrcIndices = srcIndices.size();
            indices.reserve(numIndices + numSrcIndices);
            for(int i=0; i < numSrcIndices; ++i){
                indices.push_back(srcIndices[i] + numNormals);
            }
        }
    }
}


/**
   \todo take into acount the case where some meshes have normals or colors
   and others don't have them.
*/
SgMesh* MeshExtractor::integrate(SgNode* node)
{
    SgMesh* mesh = new SgMesh;
    extract(node, std::bind(integrateMesh, this, mesh));
    return mesh;
}
