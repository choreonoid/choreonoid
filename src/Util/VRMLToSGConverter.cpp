/**
   @author Shin'ichiro Nakaoka
*/

#include "VRMLToSGConverter.h"
#include "SceneDrawables.h"
#include "SceneLights.h"
#include "SceneEffects.h"
#include "Triangulator.h"
#include "PolygonMeshTriangulator.h"
#include "MeshFilter.h"
#include "MeshGenerator.h"
#include "ImageIO.h"
#include "SceneLoader.h"
#include "Exception.h"
#include "NullOut.h"
#include "EigenUtil.h"
#include <fmt/format.h>
#include <boost/algorithm/string.hpp>
#include <tuple>

using namespace std;
using namespace cnoid;
using fmt::format;

namespace cnoid {

class VRMLToSGConverterImpl
{
public:
    VRMLToSGConverter* self;

    ostream* os_;
    ostream& os() { return *os_; }

    bool isTriangulationEnabled;
    bool isNormalGenerationEnabled;

    vector<int> removedFaceIndices;
    vector<int> removedFaceVertexIndices;

    PolygonMeshTriangulator polygonMeshTriangulator;
        
    Triangulator<SgVertexArray> triangulator;
    vector<int> polygon;

    vector<int> newColorPosToOrgColorPosMap;

    MeshFilter meshFilter;
    MeshGenerator meshGenerator;

    ImageIO imageIO;

    VRMLMaterialPtr defaultMaterial;

    typedef map<VRMLNodePtr, SgNodePtr> VRMLNodeToSgNodeMap;
    VRMLNodeToSgNodeMap vrmlNodeToSgNodeMap;

    typedef map<VRMLGeometryPtr, SgMeshPtr> VRMLGeometryToSgMeshMap;
    VRMLGeometryToSgMeshMap vrmlGeometryToSgMeshMap;

    typedef map<VRMLGeometryPtr, SgPlotPtr> VRMLGeometryToSgPlotMap;
    VRMLGeometryToSgPlotMap vrmlGeometryToSgPlotMap;
        
    typedef map<VRMLMaterialPtr, SgMaterialPtr> VRMLMaterialToSgMaterialMap;
    VRMLMaterialToSgMaterialMap vrmlMaterialToSgMaterialMap;

    typedef map<VRMLTexturePtr, SgTexturePtr> VRMLTextureToSgTextureMap;
    VRMLTextureToSgTextureMap vrmlTextureToSgTextureMap;

    typedef map<VRMLTextureTransformPtr, SgTextureTransformPtr> VRMLTextureTransformToSgTextureTransformMap;
    VRMLTextureTransformToSgTextureTransformMap vrmlTextureTransformToSgTextureTransformMap;
        
    typedef map<string, SgImagePtr> ImagePathToSgImageMap;
    ImagePathToSgImageMap imagePathToSgImageMap;
        
    enum BoxFaceID { NO_FACE, LEFT_FACE, TOP_FACE, FRONT_FACE, BOTTOM_FACE, RIGHT_FACE, BACK_FACE };

    unique_ptr<SceneLoader> sceneLoader;
        
    VRMLToSGConverterImpl(VRMLToSGConverter* self);
    void putMessage(const std::string& message);
    SgNode* convertNode(VRMLNode* vnode);
    SgNode* convertGroupNode(AbstractVRMLGroup* vgroup);
    pair<SgNode*, SgGroup*> createTransformNodeSet(VRMLTransform* vt);
    SgNode* convertShapeNode(VRMLShape* vshape);
    SgMeshPtr createMeshFromIndexedFaceSet(VRMLIndexedFaceSet* vface);
    bool setIndicesForPerTriangleData(SgIndexArray& indices, int dataSize);
    bool convertIndicesForTriangles(SgIndexArray& indices, const MFInt32& orgIndices, const bool perVertex, const bool ccw);
    SgPolygonMeshPtr createPolygonMeshFromIndexedFaceSet(VRMLIndexedFaceSet* vface);
    void setIndicesForPerPolygonData(SgIndexArray& indices, int dataSize, const MFInt32& orgCoordIndices);
    void convertIndicesForPolygons(
        SgIndexArray& indices, const MFInt32& orgIndices, const MFInt32& orgCoordIndices, const bool perVertex, const bool ccw);
    SgMeshPtr createMeshFromElevationGrid(VRMLElevationGrid* grid);
    void setDefaultTextureCoordinateForElevationGrid(const SgMeshPtr& mesh, const VRMLElevationGrid* grid);
    SgMeshPtr createMeshFromExtrusion(VRMLExtrusion* extrusion);
    void setDefaultTextureCoordinateForExtrusion(const SgMeshPtr& mesh, const VRMLExtrusion* extrusion);
    SgMaterial* createMaterial(VRMLMaterial* vm);
    SgTexture* createTexture(VRMLTexture* vt);
    SgNode* convertLineSet(VRMLIndexedLineSet* vLineSet);
    SgNode* convertPointSet(VRMLPointSet* vPointSet);
    SgTextureTransform* createTextureTransform(VRMLTextureTransform* tt);
    SgNode* convertLightNode(VRMLLight* vnode);
    void setLightCommonProperties(SgLight* light, VRMLLight* vlight);
    SgNode* createPointLight(VRMLPointLight* vlight);
    SgSpotLight* createSpotLight(VRMLSpotLight* vlight);
    SgDirectionalLight* createDirectionalLight(VRMLDirectionalLight* vlight);
    SgNode* convertFogNode(VRMLFog* vfog);
    SgNode* readNonVrmlInline(VRMLNonVrmlInline* nonVrmlInline);
};

}


VRMLToSGConverter::VRMLToSGConverter()
{
    impl = new VRMLToSGConverterImpl(this);
}


VRMLToSGConverterImpl::VRMLToSGConverterImpl(VRMLToSGConverter* self)
    : self(self)
{
    os_ = &nullout();
    isTriangulationEnabled = true;
    isNormalGenerationEnabled = true;
    imageIO.setUpsideDown(true);
    defaultMaterial = new VRMLMaterial();
}


VRMLToSGConverter::~VRMLToSGConverter()
{
    delete impl;
}


void VRMLToSGConverter::setMessageSink(std::ostream& os)
{
    impl->os_ = &os;
    if(impl->sceneLoader){
        impl->sceneLoader->setMessageSink(os);
    }
}


void VRMLToSGConverter::setTriangulationEnabled(bool on)
{
    impl->isTriangulationEnabled = on;
}


void VRMLToSGConverter::setDivisionNumber(int divisionNumber)
{
    impl->meshGenerator.setDivisionNumber(divisionNumber);
}


int VRMLToSGConverter::divisionNumber() const
{
    return impl->meshGenerator.divisionNumber();
}


void VRMLToSGConverter::setNormalGenerationEnabled(bool on, bool doOverwrite)
{
    impl->isNormalGenerationEnabled = on;
    impl->meshFilter.setNormalOverwritingEnabled(doOverwrite);
}


void VRMLToSGConverter::setMinCreaseAngle(double angle)
{
    impl->meshFilter.setMinCreaseAngle(angle);
}


void VRMLToSGConverter::setMaxCreaseAngle(double angle)
{
    impl->meshFilter.setMaxCreaseAngle(angle);
}


void VRMLToSGConverter::clearConvertedNodeMap()
{
    impl->vrmlNodeToSgNodeMap.clear();
    impl->vrmlGeometryToSgMeshMap.clear();
    impl->vrmlGeometryToSgPlotMap.clear();
    impl->vrmlMaterialToSgMaterialMap.clear();
    impl->vrmlTextureToSgTextureMap.clear();
    impl->vrmlTextureTransformToSgTextureTransformMap.clear();
    impl->imagePathToSgImageMap.clear();
}


SgNodePtr VRMLToSGConverter::convert(VRMLNodePtr vrmlNode)
{
    if(vrmlNode){
        return impl->convertNode(vrmlNode.get());
    }
    return 0;
}


void VRMLToSGConverterImpl::putMessage(const std::string& message)
{
    os() << message << endl;

    /*
      if(!self->sigMessage.empty()){
      self->sigMessage(message + "\n" );
      }
    */
}


SgNode* VRMLToSGConverterImpl::convertNode(VRMLNode* vnode)
{
    SgNode* node = 0;

    VRMLNodeToSgNodeMap::iterator p = vrmlNodeToSgNodeMap.find(vnode);
    if(p != vrmlNodeToSgNodeMap.end()){
        node = p->second.get();

    } else {
        if(VRMLProtoInstance* protoInstance = dynamic_cast<VRMLProtoInstance*>(vnode)){
            vnode = protoInstance->actualNode.get();
        }

        if(vnode){
            if(AbstractVRMLGroup* group = dynamic_cast<AbstractVRMLGroup*>(vnode)){
                node = convertGroupNode(group);
            } else if(VRMLShape* shape = dynamic_cast<VRMLShape*>(vnode)){
                node = convertShapeNode(shape);
            } else if(VRMLLight* light = dynamic_cast<VRMLLight*>(vnode)){
                node = convertLightNode(light);
            } else if(VRMLFog* fog = dynamic_cast<VRMLFog*>(vnode)){
                node = convertFogNode(fog);
            } else if(VRMLNonVrmlInline* nonVrmlInline = dynamic_cast<VRMLNonVrmlInline*>(vnode)){
                node = readNonVrmlInline(nonVrmlInline);
            }
            if(node){
                node->setName(vnode->defName);
                vrmlNodeToSgNodeMap[vnode] = node;
            }
        }
    }
        
    return node;
}


SgNode* VRMLToSGConverterImpl::convertGroupNode(AbstractVRMLGroup* vgroup)
{
    SgNode* top;
    SgGroup* group;

    if(VRMLTransform* transform = dynamic_cast<VRMLTransform*>(vgroup)){
        std::tie(top, group) = createTransformNodeSet(transform);
    } else {
        group = new SgGroup;
        top = group;
    }
    
    int num = vgroup->countChildren();
    for(int i=0; i < num; i++){
        SgNode* child = convertNode(vgroup->getChild(i));
        if(child){
            group->addChild(child);
        }
    }
    
    if(group->numChildren() == 0){
        delete top;
        top = 0;
    }
    
    return top;
}


pair<SgNode*, SgGroup*> VRMLToSGConverterImpl::createTransformNodeSet(VRMLTransform* vt)
{
    const Translation3d C(vt->center);
    const AngleAxisd& R = vt->rotation;
    const Translation3d T(vt->translation);
    SgPosTransform* transform = new SgPosTransform;
    
    if(vt->scale.isOnes()){
        // no scaling
        transform->setTransform(T * C * R * C.inverse());
        return make_pair(transform, transform);
    } else {
        SgScaleTransform* scale = new SgScaleTransform;
        scale->setScale(vt->scale);
        transform->addChild(scale);
        if(vt->center.isZero() && !vt->scaleOrientation.angle()){
            transform->setTransform(T * R);
            return make_pair(transform, scale);
        } else {
            SgPosTransform* transform2 = new SgPosTransform;
            const AngleAxisd& SR = vt->scaleOrientation;
            Affine3d S;
            S.linear() = vt->scale.asDiagonal();
            S.translation().setZero();
            transform->setTransform(T * C * R * SR);
            transform2->setTransform(SR.inverse() * C.inverse());
            scale->addChild(transform2);
            return make_pair(transform, transform2);
        }
    }
}


SgNode* VRMLToSGConverterImpl::convertShapeNode(VRMLShape* vshape)
{
    SgNode* converted = nullptr;

    VRMLGeometry* vrmlGeometry = dynamic_node_cast<VRMLGeometry>(vshape->geometry).get();
    if(vrmlGeometry){
        SgMeshPtr mesh;
        VRMLGeometryToSgMeshMap::iterator p = vrmlGeometryToSgMeshMap.find(vrmlGeometry);
        if(p != vrmlGeometryToSgMeshMap.end()){
            mesh = p->second;
        } else {
            bool generateTexCoord = false;
            if(vshape->appearance && vshape->appearance->texture){
                generateTexCoord = true;
            }
            if(VRMLIndexedFaceSet* faceSet = dynamic_cast<VRMLIndexedFaceSet*>(vrmlGeometry)){
                if(!isTriangulationEnabled){
                    mesh = createMeshFromIndexedFaceSet(faceSet);
                } else {
                    SgPolygonMeshPtr polygonMesh = createPolygonMeshFromIndexedFaceSet(faceSet);
                    if(polygonMesh){
                        mesh = polygonMeshTriangulator.triangulate(polygonMesh);
                        const string& errorMessage = polygonMeshTriangulator.errorMessage();
                        if(!errorMessage.empty()){
                            string message;
                            if(faceSet->defName.empty()){
                                message = "Error of an IndexedFaceSet node: \n";
                            } else {
                                message = format("Error of IndexedFaceSet node \"{}\": \n", faceSet->defName);
                            }
                            putMessage(message + errorMessage);
                        }
                    }
                }
                if(mesh && isNormalGenerationEnabled){
                    meshFilter.generateNormals(mesh, faceSet->creaseAngle);
                }
                    
            } else if(VRMLBox* box = dynamic_cast<VRMLBox*>(vrmlGeometry)){
                mesh = meshGenerator.generateBox(
                    Vector3(box->size[0], box->size[1], box->size[2]), generateTexCoord);
                
            } else if(VRMLSphere* sphere = dynamic_cast<VRMLSphere*>(vrmlGeometry)){
                mesh = meshGenerator.generateSphere(sphere->radius, generateTexCoord);
                
            } else if(VRMLCylinder* cylinder = dynamic_cast<VRMLCylinder*>(vrmlGeometry)){
                mesh = meshGenerator.generateCylinder(
                    cylinder->radius, cylinder->height, cylinder->bottom, cylinder->top, cylinder->side, generateTexCoord);
                
            } else if(VRMLCone* cone = dynamic_cast<VRMLCone*>(vrmlGeometry)){
                mesh = meshGenerator.generateCone(
                    cone->bottomRadius, cone->height, cone->bottom, cone->side, generateTexCoord);
                
            } else if(VRMLElevationGrid* elevationGrid = dynamic_cast<VRMLElevationGrid*>(vrmlGeometry)){
                mesh = createMeshFromElevationGrid(elevationGrid);
                
            } else if(VRMLExtrusion* extrusion = dynamic_cast<VRMLExtrusion*>(vrmlGeometry)){
                mesh = createMeshFromExtrusion(extrusion);
                
            } else if(VRMLIndexedLineSet* lineSet = dynamic_cast<VRMLIndexedLineSet*>(vrmlGeometry)){
                converted = convertLineSet(lineSet);

            } else if(VRMLPointSet* pointSet = dynamic_cast<VRMLPointSet*>(vrmlGeometry)){
                converted = convertPointSet(pointSet);
                
            } else {
                putMessage(format("VRML {} node is not supported as a geometry.", vrmlGeometry->typeName()));
            }
            
            if(mesh){
                mesh->setName(vrmlGeometry->defName);
                vrmlGeometryToSgMeshMap[vrmlGeometry] = mesh;
            }
        }
        if(mesh){
            SgShape* shape = new SgShape;
            converted = shape;
            shape->setMesh(mesh);

            if(vshape->appearance && vshape->appearance->material){
                auto vm = vshape->appearance->material;
                VRMLMaterialToSgMaterialMap::iterator p = vrmlMaterialToSgMaterialMap.find(vm);
                if(p != vrmlMaterialToSgMaterialMap.end()){
                    shape->setMaterial(p->second);
                } else {
                    shape->setMaterial(createMaterial(vm));
                    vrmlMaterialToSgMaterialMap[vm] = shape->material();
                }
            }

            if(vshape->appearance && vshape->appearance->texture){

                SgTextureTransformPtr textureTransform;
                if(vshape->appearance->textureTransform){
                    VRMLTextureTransform* vtt = vshape->appearance->textureTransform.get();
                    auto pp = vrmlTextureTransformToSgTextureTransformMap.find(vtt);
                    if(pp != vrmlTextureTransformToSgTextureTransformMap.end()){
                        textureTransform = pp->second;
                    } else {
                        textureTransform = createTextureTransform(vtt);
                        vrmlTextureTransformToSgTextureTransformMap[vtt] = textureTransform;
                    }
                }

                SgTexturePtr texture;
                VRMLTexture* vt = vshape->appearance->texture.get();
                VRMLTextureToSgTextureMap::iterator p = vrmlTextureToSgTextureMap.find(vt);
                if(p != vrmlTextureToSgTextureMap.end()){
                    if(p->second->textureTransform() == textureTransform){
                        texture = p->second;
                    }
                }
                if(!texture){
                    texture = createTexture(vt);
                    vrmlTextureToSgTextureMap[vt] = texture;
                }
                if(texture){
                    texture->setTextureTransform(textureTransform);
                    shape->setTexture(texture);

                    if(!mesh->texCoords()){
                        if(dynamic_cast<VRMLIndexedFaceSet*>(vrmlGeometry)){
                            meshGenerator.generateTextureCoordinateForIndexedFaceSet(mesh);
                        } else if(VRMLElevationGrid* elevationGrid = dynamic_cast<VRMLElevationGrid*>(vrmlGeometry)){
                            setDefaultTextureCoordinateForElevationGrid(mesh, elevationGrid);
                        } else if(VRMLExtrusion* extrusion = dynamic_cast<VRMLExtrusion*>(vrmlGeometry)){
                            setDefaultTextureCoordinateForExtrusion(mesh, extrusion);
                        }
                    }
                }
            }
        }
    }

    return converted;
}


SgMeshPtr VRMLToSGConverterImpl::createMeshFromIndexedFaceSet(VRMLIndexedFaceSet* vface)
{
    if(!vface->coord || vface->coord->point.empty() || vface->coordIndex.empty()){
        return nullptr;
    }
    
    SgMeshPtr mesh = new SgMesh;
    mesh->setSolid(vface->solid);
    mesh->setVertices(new SgVertexArray(vface->coord->point));

    removedFaceIndices.clear();
    removedFaceVertexIndices.clear();

    const MFInt32& orgCoordIndices = vface->coordIndex;
    const bool ccw = vface->ccw;

    const int orgCoordIndicesSize = orgCoordIndices.size();
    SgIndexArray& triangleVertices = mesh->triangleVertices();
    triangleVertices.reserve((orgCoordIndicesSize + 1) * 3 / 4);
    int faceIndex = 0;
    int faceVertexIndex = 0;
    int firstVertexIndex = 0;
    int numFaceVertices = 0;
    for(int i=0; i < orgCoordIndicesSize; ++i){
        int index = orgCoordIndices[i];
        if(index >= 0){
            ++numFaceVertices;
        } else {
            if(numFaceVertices == 3) { // Triangle ?
                if(ccw){
                    triangleVertices.push_back(orgCoordIndices[firstVertexIndex]);
                    triangleVertices.push_back(orgCoordIndices[firstVertexIndex + 1]);
                    triangleVertices.push_back(orgCoordIndices[firstVertexIndex + 2]);
                } else { // flip
                    triangleVertices.push_back(orgCoordIndices[firstVertexIndex + 2]);
                    triangleVertices.push_back(orgCoordIndices[firstVertexIndex + 1]);
                    triangleVertices.push_back(orgCoordIndices[firstVertexIndex]);
                }
                faceVertexIndex += 3;
            } else {
                removedFaceIndices.push_back(faceIndex);
                for(int j=0; j < numFaceVertices; ++j){
                    removedFaceVertexIndices.push_back(faceVertexIndex++);
                }
            }
            firstVertexIndex = i + 1; // next position
            numFaceVertices = 0;
            ++faceVertexIndex;
            ++faceIndex;
        }
    }

    if(!removedFaceIndices.empty()){
        if(vface->defName.empty()){
            putMessage(format("An IndexedFaceSet node contains {} non-triangle polygon(s).",
                    removedFaceIndices.size()));
        } else {
            putMessage(format("IndexedFaceSet node \"{0}\" contains {1} non-triangle polygon(s).",
                    vface->defName, removedFaceIndices.size()));
        }
    }

    if(vface->normal && !vface->normal->vector.empty()){
        bool converted;
        if(vface->normalIndex.empty() && !vface->normalPerVertex){
            converted = setIndicesForPerTriangleData(mesh->normalIndices(), vface->normal->vector.size());
        } else {
            converted = convertIndicesForTriangles(mesh->normalIndices(), vface->normalIndex, vface->normalPerVertex, ccw);
        }
        if(converted){
            mesh->setNormals(new SgNormalArray(vface->normal->vector));
        } else {
            putMessage("The normalIndex field of an IndexedFaceSet node contains illegal data.");
        }
    }

    if(vface->color && !vface->color->color.empty()){
        bool converted;
        if(vface->colorIndex.empty() && !vface->colorPerVertex){
            converted = setIndicesForPerTriangleData(mesh->colorIndices(), vface->color->color.size());
        } else {
            converted = convertIndicesForTriangles(mesh->colorIndices(), vface->colorIndex, vface->colorPerVertex, ccw);
        }
        if(converted){
            mesh->setColors(new SgColorArray(vface->color->color));
        } else {
            putMessage("The colorIndex field of an IndexedFaceSet node contains illegal data.");
        }
    }

    if(vface->texCoord && !vface->texCoord->point.empty()){
        if(convertIndicesForTriangles(mesh->texCoordIndices(), vface->texCoordIndex, true, ccw)){
            mesh->setTexCoords(new SgTexCoordArray(vface->texCoord->point));
        } else {
            putMessage("The texCoordIndex field of an IndexedFaceSet node contains illegal data.");
        }
    }

    mesh->updateBoundingBox();
    
    return mesh;
}


bool VRMLToSGConverterImpl::setIndicesForPerTriangleData(SgIndexArray& indices, int dataSize)
{
    const int numIndices = (dataSize - removedFaceIndices.size()) * 3;
    if(numIndices > 0){
        indices.reserve(numIndices);
    }
    int indexToSkip = removedFaceIndices.empty() ? std::numeric_limits<int>::min() : removedFaceIndices.front();
    size_t nextIndexToSkipIndex = 1;
    for(int i=0; i < dataSize; ++i){
        if(i == indexToSkip){
            if(nextIndexToSkipIndex < removedFaceIndices.size()){
                indexToSkip = removedFaceIndices[nextIndexToSkipIndex++];
            }
        } else {
            indices.push_back(i);
            indices.push_back(i);
            indices.push_back(i);
        }
    }
    return true;
}


bool VRMLToSGConverterImpl::convertIndicesForTriangles
(SgIndexArray& indices, const MFInt32& orgIndices, const bool perVertex, const bool ccw)
{
    bool converted = true;
    
    if(!orgIndices.empty()){
        vector<int>* indicesToSkip;
        if(perVertex){
            indicesToSkip = &removedFaceVertexIndices;
            indices.reserve((orgIndices.size() + 1) * 3 / 4);
        } else {
            indicesToSkip = &removedFaceIndices;
            indices.reserve(orgIndices.size() * 3);
        }
        int indexToSkip = indicesToSkip->empty() ? std::numeric_limits<int>::min() : indicesToSkip->front();
        size_t nextIndexToSkipIndex = 1;
        int numFaceVertices = 0;
        int firstVertexIndex = 0;
        const int numOrgIndices = orgIndices.size();
        for(int i=0; i < numOrgIndices; ++i){
            if(i == indexToSkip){
                if(nextIndexToSkipIndex < indicesToSkip->size()){
                    indexToSkip = (*indicesToSkip)[nextIndexToSkipIndex++];
                }
            } else {
                const int index = orgIndices[i];
                if(perVertex){
                    if(index >= 0){
                        ++numFaceVertices;
                    } else {
                        if(numFaceVertices != 3){
                            converted = false;
                            break;
                        }
                        if(ccw){
                            indices.push_back(orgIndices[firstVertexIndex]);
                            indices.push_back(orgIndices[firstVertexIndex + 1]);
                            indices.push_back(orgIndices[firstVertexIndex + 2]);
                        } else {
                            indices.push_back(orgIndices[firstVertexIndex + 2]);
                            indices.push_back(orgIndices[firstVertexIndex + 1]);
                            indices.push_back(orgIndices[firstVertexIndex]);
                        }
                        firstVertexIndex = i + 1;
                        numFaceVertices = 0;
                    }
                } else { // not perVertex
                    if(index < 0){
                        converted = false;
                        break;
                    }
                    indices.push_back(index);
                    indices.push_back(index);
                    indices.push_back(index);
                }
            }
        }
        if(!converted){
            indices.clear();
        }
    }

    return converted;
}


SgPolygonMeshPtr VRMLToSGConverterImpl::createPolygonMeshFromIndexedFaceSet(VRMLIndexedFaceSet* vface)
{
    if(!vface->coord){
        putMessage("VRMLIndexedFaceSet: The coord field is not defined." );
        return nullptr;
    }
    if(vface->coord->point.empty()){
        putMessage("VRMLIndexedFaceSet: The point field is empty." );
        return nullptr;
    }
    if(vface->coordIndex.empty()){
        putMessage("VRMLIndexedFaceSet: The coordIndex field is empty." );
        return nullptr;
    }
    
    SgPolygonMeshPtr mesh = new SgPolygonMesh;
    mesh->setSolid(vface->solid);
    mesh->setVertices(new SgVertexArray(vface->coord->point));

    const MFInt32& orgCoordIndices = vface->coordIndex;
    const bool ccw = vface->ccw;

    {
        SgIndexArray& polygonVertices = mesh->polygonVertices();
        if(ccw){
            polygonVertices = orgCoordIndices;
        } else {
            polygonVertices.reserve(orgCoordIndices.size());
            int firstVertexIndex = 0;
            int numFaceVertices = 0;
            for(size_t i=0; i < orgCoordIndices.size(); ++i){
                int index = orgCoordIndices[i];
                if(index >= 0){
                    ++numFaceVertices;
                } else {
                    while(--numFaceVertices >= 0){
                        polygonVertices.push_back(orgCoordIndices[firstVertexIndex + numFaceVertices]);
                    }
                    polygonVertices.push_back(-1);
                    firstVertexIndex = i + 1;
                    numFaceVertices = 0;
                }
            }
        }
    }

    if(vface->normal && !vface->normal->vector.empty()){
        mesh->setNormals(new SgNormalArray(vface->normal->vector));
        if(vface->normalIndex.empty()){
            if(!vface->normalPerVertex){
                setIndicesForPerPolygonData(mesh->normalIndices(), mesh->normals()->size(), orgCoordIndices);
            }
        } else {
            convertIndicesForPolygons(mesh->normalIndices(), vface->normalIndex, orgCoordIndices, vface->normalPerVertex, ccw);
        }
    }

    if(vface->color && !vface->color->color.empty()){
        mesh->setColors(new SgColorArray(vface->color->color));
        if(vface->colorIndex.empty()){
            if(!vface->colorPerVertex){
                setIndicesForPerPolygonData(mesh->colorIndices(), mesh->colors()->size(), orgCoordIndices);
            }
        } else {
            convertIndicesForPolygons(mesh->colorIndices(), vface->colorIndex, orgCoordIndices, vface->colorPerVertex, ccw);
        }
    }

    if(vface->texCoord && !vface->texCoord->point.empty()){
        mesh->setTexCoords(new SgTexCoordArray(vface->texCoord->point));
        if(!vface->texCoordIndex.empty()){
            convertIndicesForPolygons(mesh->texCoordIndices(), vface->texCoordIndex, orgCoordIndices, true, ccw);
        }
    }

    // The bounding box doesn't have to be updated because this polygon mesh is immediately converted into the triagnle mesh
    
    return mesh;
}


void VRMLToSGConverterImpl::setIndicesForPerPolygonData(SgIndexArray& indices, int dataSize, const MFInt32& orgCoordIndices)
{
    indices.reserve(orgCoordIndices.size());
    size_t indexInOrgCoordIndices = 0;
    for(int i=0; i < dataSize; ++i){
        while(indexInOrgCoordIndices < orgCoordIndices.size()){
            const int orgCoordIndex = orgCoordIndices[indexInOrgCoordIndices++];
            if(orgCoordIndex < 0){
                break;
            }
            indices.push_back(i);
        }
        indices.push_back(-1);
    }
}


void VRMLToSGConverterImpl::convertIndicesForPolygons
(SgIndexArray& indices, const MFInt32& orgIndices, const MFInt32& orgCoordIndices, const bool perVertex, const bool ccw)
{
    if(perVertex){
        if(ccw){
            indices = orgIndices;
            return;
        }
        indices.reserve(orgIndices.size());
        int firstVertexIndex = 0;
        int numFaceVertices = 0;
        for(size_t i=0; i < orgIndices.size(); ++i){
            if(orgIndices[i] >= 0){
                ++numFaceVertices;
            } else {
                while(--numFaceVertices >= 0){
                    indices.push_back(orgIndices[firstVertexIndex + numFaceVertices]);
                }
                indices.push_back(-1);
                firstVertexIndex = i + 1;
                numFaceVertices = 0;
            }
        }
    } else {
        indices.reserve(orgCoordIndices.size());
        size_t indexInOrgCoordIndices = 0;
        for(size_t i=0; i < orgIndices.size(); ++i){
            const int index = orgIndices[i];
            if(index >= 0){
                while(indexInOrgCoordIndices < orgCoordIndices.size()){
                    const int orgCoordIndex = orgCoordIndices[indexInOrgCoordIndices++];
                    if(orgCoordIndex < 0){
                        break;
                    }
                    indices.push_back(index);
                }
                indices.push_back(-1);
            }
        }
    }
}


SgMeshPtr VRMLToSGConverterImpl::createMeshFromElevationGrid(VRMLElevationGrid* grid)
{
    if(grid->xDimension * grid->zDimension != static_cast<SFInt32>(grid->height.size())){
        putMessage("A VRML ElevationGrid node has illegal parameters.");
        return SgMeshPtr();
    }

    SgMeshPtr mesh = new SgMesh;
    mesh->setVertices(new SgVertexArray());
    SgVertexArray& vertices = *mesh->vertices();
    vertices.reserve(grid->zDimension * grid->xDimension);
    
    for(int z=0; z < grid->zDimension; z++){
        for(int x=0; x < grid->xDimension; x++){
            vertices.push_back(Vector3f(x * grid->xSpacing, grid->height[z * grid->xDimension + x], z * grid->zSpacing));
        }
    }

    mesh->reserveNumTriangles((grid->zDimension - 1) * (grid->xDimension - 1) * 2);
    
    for(int z=0; z < grid->zDimension - 1; ++z){
        const int current = z * grid->xDimension;
        const int next = (z + 1) * grid->xDimension;
        for(int x=0; x < grid->xDimension - 1; ++x){
            if(grid->ccw){
                mesh->addTriangle( x + current, x + next, (x + 1) + next);
                mesh->addTriangle( x + current, (x + 1) + next, (x + 1) + current);
            }else{
                mesh->addTriangle( x + current, (x + 1) + next, x + next);
                mesh->addTriangle( x + current, (x + 1) + current, (x + 1) + next);
            }
        }
    }

    mesh->setSolid(grid->solid);

    if(isNormalGenerationEnabled){
        meshFilter.generateNormals(mesh, grid->creaseAngle);
    }

    if(grid->color){
        const MFColor& orgColors = grid->color->color;
        if(!orgColors.empty()){
            if(!grid->colorPerVertex){
                mesh->setColors(new SgColorArray());
                SgColorArray& colors = *mesh->colors();
                const int n = orgColors.size();
                colors.reserve(n * 2);
                for(int i=0; i < n; ++i){
                    const SFColor& c = orgColors[i];
                    for(int j=0; j < 6; ++j){
                        colors.push_back(c);
                    }
                }
            }
        }
    }
                
    if(grid->texCoord){
        const MFVec2s& point = grid->texCoord->point;
        const int n = point.size();
        mesh->setTexCoords(new SgTexCoordArray());
        SgTexCoordArray& texCoords = *mesh->texCoords();
        texCoords.resize(n);
        for(int i=0; i < n; ++i){
            texCoords[i] = point[i];
        }
        mesh->texCoordIndices() = mesh->triangleVertices();
    }

    mesh->updateBoundingBox();

    return mesh;
}


void VRMLToSGConverterImpl::setDefaultTextureCoordinateForElevationGrid(const SgMeshPtr& mesh, const VRMLElevationGrid* grid)
{
    float xmax = grid->xSpacing * (grid->xDimension - 1);
    float zmax = grid->zSpacing * (grid->zDimension - 1);

    mesh->setTexCoords(new SgTexCoordArray());
    SgTexCoordArray& texCoords = *mesh->texCoords();
    const SgVertexArray& vertices = *mesh->vertices();
    
    for(size_t i=0; i < vertices.size(); ++i){
        const Vector3f& v = vertices[i];
        texCoords.push_back(Vector2f(v.x() / xmax, v.z() / zmax));
    }
    mesh->texCoordIndices() = mesh->triangleVertices();
}


SgMeshPtr VRMLToSGConverterImpl::createMeshFromExtrusion(VRMLExtrusion* extrusion)
{
    bool isClosed = false;
    const int numSpine = extrusion->spine.size();
    const int numcross = extrusion->crossSection.size();
    if(extrusion->spine[0][0] == extrusion->spine[numSpine - 1][0] &&
       extrusion->spine[0][1] == extrusion->spine[numSpine - 1][1] &&
       extrusion->spine[0][2] == extrusion->spine[numSpine - 1][2] ){
        isClosed = true;
    }
    bool crossSectionisClosed = false;
    if(extrusion->crossSection[0][0] == extrusion->crossSection[numcross - 1][0] &&
       extrusion->crossSection[0][1] == extrusion->crossSection[numcross - 1][1] ){
        crossSectionisClosed = true;
    }

    SgMeshPtr mesh = new SgMesh;
    mesh->setVertices(new SgVertexArray());
    SgVertexArray& vertices = *mesh->vertices();
    vertices.reserve(numSpine*numcross);

    SFVec3f preZaxis(SFVec3f::Zero());
    int definedZaxis = -1;
    std::vector<SFVec3f> Yaxisarray;
    std::vector<SFVec3f> Zaxisarray;
    if(numSpine > 2){
        for(int i=0; i < numSpine; ++i){
            SFVec3f Yaxis, Zaxis;
            if(i == 0){
                if(isClosed){
                    const SFVec3f& spine1 = extrusion->spine[numSpine - 2];
                    const SFVec3f& spine2 = extrusion->spine[0];
                    const SFVec3f& spine3 = extrusion->spine[1];
                    Yaxis = spine3 - spine1;
                    Zaxis = (spine3 - spine2).cross(spine1 - spine2);
                } else {
                    const SFVec3f& spine1 = extrusion->spine[0];
                    const SFVec3f& spine2 = extrusion->spine[1];
                    const SFVec3f& spine3 = extrusion->spine[2];
                    Yaxis = spine2 - spine1;
                    Zaxis = (spine3 - spine2).cross(spine1 - spine2);
                }
            } else if(i == numSpine - 1){
                if(isClosed){
                    const SFVec3f& spine1 = extrusion->spine[numSpine - 2];
                    const SFVec3f& spine2 = extrusion->spine[0];
                    const SFVec3f& spine3 = extrusion->spine[1];
                    Yaxis = spine3 - spine1;
                    Zaxis = (spine3 - spine2).cross(spine1 - spine2);
                } else {
                    const SFVec3f& spine1 = extrusion->spine[numSpine - 3];
                    const SFVec3f& spine2 = extrusion->spine[numSpine - 2];
                    const SFVec3f& spine3 = extrusion->spine[numSpine - 1];
                    Yaxis = spine3 - spine2;
                    Zaxis = (spine3 - spine2).cross(spine1 - spine2);
                }
            } else {
                const SFVec3f& spine1 = extrusion->spine[i - 1];
                const SFVec3f& spine2 = extrusion->spine[i];
                const SFVec3f& spine3 = extrusion->spine[i + 1];
                Yaxis = spine3 - spine1;
                Zaxis = (spine3-spine2).cross(spine1-spine2);
            }
            if(!Zaxis.norm()){
                if(definedZaxis != -1)
                    Zaxis = preZaxis;
            } else {
                if(definedZaxis == -1){
                    definedZaxis = i;
                }
                preZaxis = Zaxis;
            }
            Yaxisarray.push_back(Yaxis);
            Zaxisarray.push_back(Zaxis);
        }
    } else {
        const SFVec3f Yaxis(extrusion->spine[1] - extrusion->spine[0]);
        Yaxisarray.push_back(Yaxis);
        Yaxisarray.push_back(Yaxis);
    }
    for(int i=0; i < numSpine; ++i){
        Eigen::Matrix3d Scp;
        SFVec3f y = Yaxisarray[i].normalized();
        if(definedZaxis == -1){
            SFRotation R(acos(y[1]), SFRotation::Vector3(y[2], 0.0, -y[0]));
            Scp = R.toRotationMatrix();
        } else {
            if(i < definedZaxis){
                Zaxisarray[i] = Zaxisarray[definedZaxis];
            }
            if(i && (Zaxisarray[i].dot(Zaxisarray[i - 1]) < 0.0)){
                Zaxisarray[i] *= -1.0;
            }
            SFVec3f z = Zaxisarray[i].normalized();
            SFVec3f x = y.cross(z);
            Scp << x, y, z;
        }

        const SFVec3f& spine = extrusion->spine[i];
        SFVec3f scale;
        if(extrusion->scale.size() == 1){
            scale << extrusion->scale[0][0], 0.0, extrusion->scale[0][1];
        } else {
            scale << extrusion->scale[i][0], 0.0, extrusion->scale[i][1];
        }
        SFRotation o;
        if(extrusion->orientation.size() == 1){
            o = extrusion->orientation[0];
        } else {
            o = extrusion->orientation[i];
        }

        for(int j=0; j < numcross; ++j){
            const SFVec3f crossSection(extrusion->crossSection[j][0], 0.0, extrusion->crossSection[j][1]);
            const SFVec3f v1(crossSection[0] * scale[0], 0.0, crossSection[2] * scale[2]);
            vertices.push_back((Scp * o.toRotationMatrix() * v1 + spine).cast<float>());
        }
    }

    for(int i=0; i < numSpine - 1 ; ++i){
        const int upper = i * numcross;
        const int lower = (i + 1) * numcross;
        for(int j=0; j < numcross - 1; ++j) {
            if(extrusion->ccw){
                mesh->addTriangle(j + upper, j + lower, (j + 1) + lower);
                mesh->addTriangle(j + upper, (j + 1) + lower, j + 1 + upper);
            } else {
                // upward convex triangle
                mesh->addTriangle(j + upper, (j + 1) + lower, j + lower);
                // downward convex triangle
                mesh->addTriangle(j + upper, (j + 1) + upper, j + 1 + lower);
            }
        }
    }

    int j = 0;
    if(crossSectionisClosed){
        j = 1;
    }
    if(extrusion->beginCap && !isClosed){
        triangulator.setVertices(vertices);
        polygon.clear();
        for(int i=0; i < numcross - j; ++i){
            polygon.push_back(i);
        }
        triangulator.apply(polygon);
        const vector<int>& triangles = triangulator.triangles();
        for(size_t i=0; i < triangles.size(); i += 3){
            if(extrusion->ccw){
                mesh->addTriangle(polygon[triangles[i]], polygon[triangles[i+1]], polygon[triangles[i+2]]);
            } else {
                mesh->addTriangle(polygon[triangles[i]], polygon[triangles[i+2]], polygon[triangles[i+1]]);
            }
        }
    }

    if(extrusion->endCap && !isClosed){
        triangulator.setVertices(vertices);
        polygon.clear();
        for(int i=0; i < numcross - j; ++i){
            polygon.push_back(numcross * (numSpine - 1) + i);
        }
        triangulator.apply(polygon);
        const vector<int>& triangles = triangulator.triangles();
        for(size_t i=0; i < triangles.size(); i +=3){
            if(extrusion->ccw){
                mesh->addTriangle(polygon[triangles[i]], polygon[triangles[i+2]], polygon[triangles[i+1]]);
            } else {
                mesh->addTriangle(polygon[triangles[i]], polygon[triangles[i+1]], polygon[triangles[i+2]]);
            }
        }
    }

    mesh->setSolid(extrusion->solid);

    if(isNormalGenerationEnabled){
        meshFilter.generateNormals(mesh, extrusion->creaseAngle);
    }

    mesh->updateBoundingBox();

    return mesh;
}


void VRMLToSGConverterImpl::setDefaultTextureCoordinateForExtrusion(const SgMeshPtr& mesh, const VRMLExtrusion* extrusion)
{
    const int numSpine = extrusion->spine.size();
    const int numcross = extrusion->crossSection.size();

    mesh->setTexCoords(new SgTexCoordArray());
    SgTexCoordArray& texCoords = *mesh->texCoords();
    
    vector<double> s;
    vector<double> t;
    double slen = 0.0;
    s.push_back(0.0);
    for(size_t i=1; i < extrusion->crossSection.size(); ++i){
        double x = extrusion->crossSection[i][0] - extrusion->crossSection[i-1][0];
        double z = extrusion->crossSection[i][1] - extrusion->crossSection[i-1][1];
        slen += sqrt(x*x + z*z);
        s.push_back(slen);
    }
    double tlen = 0.0;
    t.push_back(0.0);
    for(size_t i=1; i < extrusion->spine.size(); ++i){
        double x = extrusion->spine[i][0] - extrusion->spine[i-1][0];
        double y = extrusion->spine[i][1] - extrusion->spine[i-1][1];
        double z = extrusion->spine[i][2] - extrusion->spine[i-1][2];
        tlen += sqrt(x*x + y*y + z*z);
        t.push_back(tlen);
    }
    for(size_t i=0; i < extrusion->spine.size(); ++i){
        Vector2f point;
        point[1] = t[i] / tlen;
        for(size_t j=0; j < extrusion->crossSection.size(); ++j){
            point[0] = s[j] / slen;
            texCoords.push_back(point);
        }
    }

    SgIndexArray& texCoordIndices = mesh->texCoordIndices();
    texCoordIndices.clear();
    const int endOfSpineVertices = (numSpine - 1) * (numcross - 1) * 2 * 3;
    texCoordIndices.resize(endOfSpineVertices);
    const SgIndexArray& triangleVertices = mesh->triangleVertices();
    copy(triangleVertices.begin(), triangleVertices.begin() + endOfSpineVertices, texCoordIndices.begin());
    
    int endOfBeginCapVertices = endOfSpineVertices;
    const int endOfSpineTexCoords = texCoords.size();

    if(extrusion->beginCap){
        if(extrusion->endCap){
            endOfBeginCapVertices += (triangleVertices.size() - endOfSpineVertices) / 2;
        } else {
            endOfBeginCapVertices = triangleVertices.size();
        }
        double xmin, xmax;
        double zmin, zmax;
        xmin = xmax = extrusion->crossSection[0][0];
        zmin = zmax = extrusion->crossSection[0][1];
        for(size_t i=1; i < extrusion->crossSection.size(); ++i){
            xmax = std::max(xmax, extrusion->crossSection[i][0]);
            xmin = std::min(xmin, extrusion->crossSection[i][0]);
            zmax = std::max(zmax, extrusion->crossSection[i][1]);
            zmin = std::min(xmin, extrusion->crossSection[i][1]);
        }
        float xsize = xmax - xmin;
        float zsize = zmax - zmin;
        for(int i=0; i < numcross; ++i){
            Vector2f point;
            point[0] = (extrusion->crossSection[i][0] - xmin) / xsize;
            point[1] = (extrusion->crossSection[i][1] - zmin) / zsize;
            texCoords.push_back(point);
        }
        for(int i = endOfSpineVertices; i < endOfBeginCapVertices; ++i){
            texCoordIndices.push_back(triangleVertices[i] + endOfSpineTexCoords);
        }
    }

    if(extrusion->endCap){
        double xmax, xmin;
        double zmax, zmin;
        xmin = xmax = extrusion->crossSection[0][0];
        zmin = zmax = extrusion->crossSection[0][1];
        for(size_t i=1; i < extrusion->crossSection.size(); ++i){
            xmax = std::max(xmax, extrusion->crossSection[i][0]);
            xmin = std::min(xmin, extrusion->crossSection[i][0]);
            zmax = std::max(zmax, extrusion->crossSection[i][1]);
            zmin = std::min(xmin, extrusion->crossSection[i][1]);
        }
        double xsize = xmax - xmin;
        double zsize = zmax - zmin;
        for(size_t i=0; i < extrusion->crossSection.size(); ++i){
            Vector2f point;
            point[0] = (extrusion->crossSection[i][0] - xmin) / xsize;
            point[1] = (extrusion->crossSection[i][1] - zmin) / zsize;
            texCoords.push_back(point);
        }
        const int offset = texCoords.size() - endOfSpineTexCoords;
        for(size_t i = endOfBeginCapVertices; i < triangleVertices.size(); ++i){
            texCoordIndices.push_back(triangleVertices[i] + offset);
        }
    }
}


SgMaterial* VRMLToSGConverterImpl::createMaterial(VRMLMaterial* vm)
{
    SgMaterial* material = new SgMaterial;
    material->setName(vm->defName);
    material->setDiffuseColor(vm->diffuseColor);
    material->setAmbientIntensity(vm->ambientIntensity);
    material->setEmissiveColor(vm->emissiveColor);
    material->setSpecularColor(vm->specularColor);
    material->setShininess(vm->shininess);
    material->setTransparency(vm->transparency);
    return material;
}


SgTextureTransform* VRMLToSGConverterImpl::createTextureTransform(VRMLTextureTransform* tt)
{
    SgTextureTransform* textureTransform = new SgTextureTransform;
    textureTransform->setName(tt->defName);
    textureTransform->setCenter(tt->center);
    textureTransform->setRotation(tt->rotation);
    textureTransform->setScale(tt->scale);
    textureTransform->setTranslation(tt->translation);
    return textureTransform;
}


SgTexture* VRMLToSGConverterImpl::createTexture(VRMLTexture* vt)
{
    SgTexture* texture = 0;

    VRMLImageTexturePtr imageTextureNode = dynamic_node_cast<VRMLImageTexture>(vt);
    if(imageTextureNode){
        SgImagePtr image;
        SgImagePtr imageForLoading;
        const MFString& urls = imageTextureNode->url;
        for(size_t i=0; i < urls.size(); ++i){
            const string& url = urls[i];
            if(!url.empty()){
                ImagePathToSgImageMap::iterator p = imagePathToSgImageMap.find(url);
                if(p != imagePathToSgImageMap.end()){
                    image = p->second;
                    break;
                } else {
                    try {
                        if(!imageForLoading){
                            imageForLoading = new SgImage;
                        }
                        imageIO.load(imageForLoading->image(), url);
                        image = imageForLoading;
                        imagePathToSgImageMap[url] = image;
                        break;
                    } catch(const exception_base& ex){
                        putMessage(*boost::get_error_info<error_info_message>(ex));
                    }
                }
            }
        }
        if(image){
            texture = new SgTexture;
            texture->setImage(image);
            texture->setRepeat(imageTextureNode->repeatS, imageTextureNode->repeatT);
        }
        
    } else if(VRMLPixelTexturePtr pixelTextureNode =  dynamic_node_cast<VRMLPixelTexture>(vt)){

        const int width = pixelTextureNode->image.width;
        const int height = pixelTextureNode->image.height;
        const int nc = pixelTextureNode->image.numComponents;

        if(width > 0 && height > 0 && nc > 0){
            texture = new SgTexture;
            SgImage* image = new SgImage;
            image->setSize(width, height, nc);

            // copy the pixels in the upside-down way
            std::vector<unsigned char>& src = pixelTextureNode->image.pixels;
            unsigned char* dest = image->pixels();
            for(int i=0; i<height; ++i){
                int ii = height - i - 1;
                for(int j=0; j < width; ++j){
                    for(int l=0; l < nc; ++l)
                    dest[(i * width + j) * nc + l] = src[(ii * width +j) * nc + l];
                }
            }
            texture->setImage(image);
            texture->setRepeat(pixelTextureNode->repeatS, pixelTextureNode->repeatT);
        }
    } else {
        putMessage("MovieTextureNode is not supported");
    }

    if(texture){
        texture->setName(vt->defName);
    }

    return texture;
}


SgNode* VRMLToSGConverterImpl::convertLineSet(VRMLIndexedLineSet* vLineSet)
{
    VRMLGeometryToSgPlotMap::iterator p = vrmlGeometryToSgPlotMap.find(vLineSet);
    if(p != vrmlGeometryToSgPlotMap.end()){
        return p->second.get();
    }

    if(!vLineSet->coord || vLineSet->coord->point.empty() || vLineSet->coordIndex.empty()){
        return 0;
    }

    SgLineSet* lineSet = new SgLineSet;
    lineSet->setVertices(new SgVertexArray(vLineSet->coord->point));

    const bool hasColors = (vLineSet->color && !vLineSet->color->color.empty());
    if(hasColors){
        newColorPosToOrgColorPosMap.clear();
    }
    const bool colorPerVertex = vLineSet->colorPerVertex;

    const MFInt32& coordIndex = vLineSet->coordIndex;
    int topPosition = 0;
    int polylineIndex = 0;
    for(size_t i=0; i < coordIndex.size(); ++i){
        const int index = coordIndex[i];
        if(index < 0){
            int n = i - topPosition;
            if(n >= 2){
                --n;
                for(int j=0; j < n; ++j){
                    const int v1 = topPosition + j;
                    const int v2 = topPosition + j + 1;
                    lineSet->addLine(coordIndex[v1], coordIndex[v2]);
                    if(hasColors){
                        if(colorPerVertex){
                            newColorPosToOrgColorPosMap.push_back(v1);
                            newColorPosToOrgColorPosMap.push_back(v2);
                        } else {
                            newColorPosToOrgColorPosMap.push_back(polylineIndex);
                            newColorPosToOrgColorPosMap.push_back(polylineIndex);
                        }
                    }
                }
            }
            topPosition = i + 1;
            ++polylineIndex;
        }
    }

    if(hasColors){
        lineSet->setColors(new SgColorArray(vLineSet->color->color));
        const int numColors = lineSet->colors()->size();
        const MFInt32& orgColorIndices = vLineSet->colorIndex;
        const int numOrgColorIndices = orgColorIndices.size();
        bool doWarning = false;
        SgIndexArray& colorIndices = lineSet->colorIndices();
        const SgIndexArray& lineVertices = lineSet->lineVertices();
        for(size_t i=0; i < lineVertices.size(); ++i){
            int orgPos = newColorPosToOrgColorPosMap[i];
            if(orgPos >= numOrgColorIndices){
                orgPos = numOrgColorIndices - 1;
                doWarning = true;
            }
            int index = orgColorIndices[orgPos];
            if(index < 0){
                index = 0;
                doWarning = true;
            } else if(index >= numColors){
                index = numColors - 1;
                doWarning = true;
            }
            colorIndices.push_back(index);
        }
        if(doWarning){
            putMessage("Warning: The colorIndex elements do not correspond to the colors or the coordIndex elements in an IndexedLineSet node.");
        }
    }

    lineSet->updateBoundingBox();
    
    vrmlGeometryToSgPlotMap[vLineSet] = lineSet;
    return lineSet;
}


SgNode* VRMLToSGConverterImpl::convertPointSet(VRMLPointSet* vPointSet)
{
    VRMLGeometryToSgPlotMap::iterator p = vrmlGeometryToSgPlotMap.find(vPointSet);
    if(p != vrmlGeometryToSgPlotMap.end()){
        return p->second;
    }

    if(!vPointSet->coord || vPointSet->coord->point.empty()){
        return nullptr;
    }

    SgPointSet* pointSet = new SgPointSet;
    pointSet->setVertices(new SgVertexArray(vPointSet->coord->point));
    pointSet->updateBoundingBox();

    if(vPointSet->color && !vPointSet->color->color.empty()){
        pointSet->setColors(new SgColorArray(vPointSet->color->color));
    }

    vrmlGeometryToSgPlotMap[vPointSet] = pointSet;

    return pointSet;
}


SgNode* VRMLToSGConverterImpl::convertLightNode(VRMLLight* vlight)
{
    if(VRMLPointLight* vPointLight = dynamic_cast<VRMLSpotLight*>(vlight)){
        return createPointLight(vPointLight);
    } else if(VRMLDirectionalLight* vDirectionalLight = dynamic_cast<VRMLDirectionalLight*>(vlight)){
        return createDirectionalLight(vDirectionalLight);
    }
	return 0;
}


void VRMLToSGConverterImpl::setLightCommonProperties(SgLight* light, VRMLLight* vlight)
{
    light->on(vlight->on);
    light->setColor(vlight->color);
    light->setIntensity(vlight->intensity);
    light->setAmbientIntensity(vlight->ambientIntensity);
}


SgNode* VRMLToSGConverterImpl::createPointLight(VRMLPointLight* vlight)
{
    SgPointLight* light;
    if(VRMLSpotLight* vSpotLight = dynamic_cast<VRMLSpotLight*>(vlight)){
        light = createSpotLight(vSpotLight);
    } else {
        light = new SgPointLight();
    }
    light->setConstantAttenuation(vlight->attenuation[0]);
    light->setLinearAttenuation(vlight->attenuation[1]);
    light->setQuadraticAttenuation(vlight->attenuation[2]);
    
    setLightCommonProperties(light, vlight);

    if(vlight->location == SFVec3f::Zero()){
        return light;
    } else {
        SgPosTransform* transform = new SgPosTransform;
        transform->setTranslation(vlight->location);
        transform->addChild(light);
        return transform;
    }
}


SgSpotLight* VRMLToSGConverterImpl::createSpotLight(VRMLSpotLight* vlight)
{
    SgSpotLight* light = new SgSpotLight();
    
    light->setDirection(vlight->direction);
    light->setBeamWidth(vlight->beamWidth);
    light->setCutOffAngle(vlight->cutOffAngle);

    return light;
}


SgDirectionalLight* VRMLToSGConverterImpl::createDirectionalLight(VRMLDirectionalLight* vlight)
{
    SgDirectionalLight* light = new SgDirectionalLight();
    light->setDirection(vlight->direction);
    setLightCommonProperties(light, vlight);
    return light;
}


SgNode* VRMLToSGConverterImpl::convertFogNode(VRMLFog* vfog)
{
    SgFog* fog = new SgFog;
    fog->setColor(vfog->color);
    fog->setVisibilityRange(vfog->visibilityRange);
    return fog;
}


SgNode* VRMLToSGConverterImpl::readNonVrmlInline(VRMLNonVrmlInline* nonVrmlInline)
{
    if(!nonVrmlInline->url.empty()){
        if(!sceneLoader){
            sceneLoader.reset(new SceneLoader);
            sceneLoader->setMessageSink(os());
        }
        return sceneLoader->load(nonVrmlInline->url);
    }
    return 0;
}
