/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "SceneGraph.h"
#include "SceneShape.h"
#include "SceneLight.h"
#include "SceneCamera.h"
#include "SceneVisitor.h"
#include "Exception.h"
#include "MeshNormalGenerator.h"
#include <boost/unordered_map.hpp>
#include <boost/make_shared.hpp>

using namespace std;
using namespace cnoid;

namespace {

const double PI = 3.14159265358979323846;

}


SgUpdate::~SgUpdate()
{

}

namespace {
typedef boost::unordered_map<const SgObject*, SgObjectPtr> CloneMap;
}

namespace cnoid {
class SgCloneMapImpl : public CloneMap { };
}


SgCloneMap::SgCloneMap()
{
    cloneMap = new SgCloneMapImpl;
    isNonNodeCloningEnabled_ = true;
}


SgCloneMap::SgCloneMap(const SgCloneMap& org)
{
    cloneMap = new SgCloneMapImpl(*org.cloneMap);
    isNonNodeCloningEnabled_ = org.isNonNodeCloningEnabled_;
}


void SgCloneMap::clear()
{
    cloneMap->clear();
}


SgObject* SgCloneMap::findOrCreateClone(const SgObject* org)
{
    CloneMap::iterator p = cloneMap->find(org);
    if(p == cloneMap->end()){
        SgObject* clone = org->clone(*this);
        (*cloneMap)[org] = clone;
        return clone;
    } else {
        return p->second.get();
    }
}


SgCloneMap::~SgCloneMap()
{
    delete cloneMap;
}


SgObject::SgObject()
{

}


SgObject::SgObject(const SgObject& org)
    : name_(org.name_)
{

}


SgObject* SgObject::clone(SgCloneMap& cloneMap) const
{
    return new SgObject(*this);
}


int SgObject::numChildObjects() const
{
    return 0;
}


SgObject* SgObject::childObject(int index)
{
    return 0;
}


void SgObject::onUpdated(SgUpdate& update)
{
    update.push(this);
    sigUpdated_(update);
    for(const_parentIter p = parents.begin(); p != parents.end(); ++p){
        (*p)->onUpdated(update);
    }
    update.pop();
}


void SgObject::addParent(SgObject* parent, bool doNotify)
{
    parents.insert(parent);
    if(doNotify){
        SgUpdate update(SgUpdate::ADDED);
        update.push(this);
        parent->onUpdated(update);
    }
    if(parents.size() == 1){
        sigGraphConnection_(true);
    }
}


void SgObject::removeParent(SgObject* parent)
{
    parents.erase(parent);
    if(parents.empty()){
        sigGraphConnection_(false);
    }
}


SgNode::SgNode()
{

}


SgNode::SgNode(const SgNode& org)
    : SgObject(org)
{

}


SgNode::~SgNode()
{

}


SgObject* SgNode::clone(SgCloneMap& cloneMap) const
{
    return new SgNode(*this);
}


void SgNode::accept(SceneVisitor& visitor)
{
    visitor.visitNode(this);
}


const BoundingBox& SgNode::boundingBox() const
{
    static const BoundingBox bbox; // empty one
    return bbox;
}


bool SgNode::isGroup() const
{
    return false;
}


SgGroup::SgGroup()
{
    isBboxCacheValid = false;
}


SgGroup::SgGroup(const SgGroup& org)
    : SgNode(org)
{
    children.reserve(org.numChildren());

    // shallow copy
    for(const_iterator p = org.begin(); p != org.end(); ++p){
        addChild(*p, false);
    }

    isBboxCacheValid = true;
    bboxCache = org.bboxCache;
}


SgGroup::SgGroup(const SgGroup& org, SgCloneMap& cloneMap)
    : SgNode(org)
{
    children.reserve(org.numChildren());

    for(const_iterator p = org.begin(); p != org.end(); ++p){
        addChild(cloneMap.getClone<SgNode>(p->get()), false);
    }

    isBboxCacheValid = true;
    bboxCache = org.bboxCache;
}


SgGroup::~SgGroup()
{
    for(const_iterator p = begin(); p != end(); ++p){
        (*p)->removeParent(this);
    }
}


SgObject* SgGroup::clone(SgCloneMap& cloneMap) const
{
    return new SgGroup(*this, cloneMap);
}


int SgGroup::numChildObjects() const
{
    return children.size();
}


SgObject* SgGroup::childObject(int index)
{
    return children[index].get();
}


void SgGroup::accept(SceneVisitor& visitor)
{
    visitor.visitGroup(this);
}


void SgGroup::onUpdated(SgUpdate& update)
{
    //if(update.action() & SgUpdate::BBOX_UPDATED){
    invalidateBoundingBox();
    SgNode::onUpdated(update);
    //}
}


const BoundingBox& SgGroup::boundingBox() const
{
    if(isBboxCacheValid){
        return bboxCache;
    }
    bboxCache.clear();
    for(const_iterator p = begin(); p != end(); ++p){
        bboxCache.expandBy((*p)->boundingBox());
    }
    isBboxCacheValid = true;

    return bboxCache;
}


bool SgGroup::isGroup() const
{
    return true;
}


bool SgGroup::contains(SgNode* node) const
{
    for(const_iterator p = begin(); p != end(); ++p){
        if((*p) == node){
            return true;
        }
    }
    return false;
}


void SgGroup::addChild(SgNode* node, bool doNotify)
{
    if(node){
        children.push_back(node);
        node->addParent(this, doNotify);
    }
}


bool SgGroup::addChildOnce(SgNode* node, bool doNotify)
{
    if(!contains(node)){
        addChild(node, doNotify);
        return true;
    }
    return false;
}


SgGroup::iterator SgGroup::removeChild(iterator childIter, bool doNotify)
{
    iterator next;
    SgNode* child = *childIter;
    child->removeParent(this);
    
    if(!doNotify){
        next = children.erase(childIter);
    } else {
        SgNodePtr childHolder = child;
        next = children.erase(childIter);
        SgUpdate update(SgUpdate::REMOVED);
        update.push(child);
        onUpdated(update);
    }
    return next;
}


bool SgGroup::removeChild(SgNode* node, bool doNotify)
{
    bool removed = false;
    iterator p = children.begin();
    while(p != children.end()){
        if((*p) == node){
            p = removeChild(p, doNotify);
            removed = true;
        } else {
            ++p;
        }
    }
    return removed;
}


void SgGroup::removeChildAt(int index, bool doNotify)
{
    removeChild(children.begin() + index, doNotify);
}


void SgGroup::clearChildren(bool doNotify)
{
    iterator p = children.begin();
    while(p != children.end()){
        p = removeChild(p, doNotify);
    }
}


void SgGroup::copyChildrenTo(SgGroup* group, bool doNotify)
{
    for(int i=0; i < children.size(); ++i){
        group->addChild(child(i), doNotify);
    }
}


void SgGroup::moveChildrenTo(SgGroup* group, bool doNotify)
{
    const int destTop = group->children.size();
    
    for(int i=0; i < children.size(); ++i){
        group->addChild(child(i));
    }
    clearChildren(doNotify);
    if(doNotify){
        SgUpdate update(SgUpdate::ADDED);
        for(int i=destTop; i < group->numChildren(); ++i){
            group->child(i)->notifyUpdate(update);
        }
    }
}


void SgGroup::throwTypeMismatchError()
{
    throw type_mismatch_error();
}


SgInvariantGroup::SgInvariantGroup()
{

}


SgInvariantGroup::SgInvariantGroup(const SgInvariantGroup& org)
    : SgGroup(org)
{

}


SgInvariantGroup::SgInvariantGroup(const SgInvariantGroup& org, SgCloneMap& cloneMap)
    : SgGroup(org, cloneMap)
{

}


SgObject* SgInvariantGroup::clone(SgCloneMap& cloneMap) const
{
    return new SgInvariantGroup(*this, cloneMap);
}


void SgInvariantGroup::accept(SceneVisitor& visitor)
{
    visitor.visitInvariantGroup(this);
}


SgTransform::SgTransform()
{

}


SgTransform::SgTransform(const SgTransform& org)
    : SgGroup(org)
{
    untransformedBboxCache = org.untransformedBboxCache;
}
    

SgTransform::SgTransform(const SgTransform& org, SgCloneMap& cloneMap)
    : SgGroup(org, cloneMap)
{
    untransformedBboxCache = org.untransformedBboxCache;
}


const BoundingBox& SgTransform::untransformedBoundingBox() const
{
    if(!isBboxCacheValid){
        boundingBox();
    }
    return untransformedBboxCache;
}


SgPosTransform::SgPosTransform()
    : T_(Affine3::Identity())
{

}


SgPosTransform::SgPosTransform(const Affine3& T)
    : T_(T)
{

}


SgPosTransform::SgPosTransform(const SgPosTransform& org)
    : SgTransform(org),
      T_(org.T_)
{

}


SgPosTransform::SgPosTransform(const SgPosTransform& org, SgCloneMap& cloneMap)
    : SgTransform(org, cloneMap),
      T_(org.T_)
{

}


SgObject* SgPosTransform::clone(SgCloneMap& cloneMap) const
{
    return new SgPosTransform(*this, cloneMap);
}


void SgPosTransform::accept(SceneVisitor& visitor)
{
    visitor.visitPosTransform(this);
}


const BoundingBox& SgPosTransform::boundingBox() const
{
    if(isBboxCacheValid){
        return bboxCache;
    }
    bboxCache.clear();
    for(const_iterator p = begin(); p != end(); ++p){
        bboxCache.expandBy((*p)->boundingBox());
    }
    untransformedBboxCache = bboxCache;
    bboxCache.transform(T_);
    isBboxCacheValid = true;
    return bboxCache;
}


void SgPosTransform::getTransform(Affine3& out_T) const
{
    out_T = T_;
}


SgScaleTransform::SgScaleTransform()
{
    scale_.setOnes();
}


SgScaleTransform::SgScaleTransform(const SgScaleTransform& org)
    : SgTransform(org),
      scale_(org.scale_)
{

}
      
    
SgScaleTransform::SgScaleTransform(const SgScaleTransform& org, SgCloneMap& cloneMap)
    : SgTransform(org, cloneMap),
      scale_(org.scale_)
{

}


SgObject* SgScaleTransform::clone(SgCloneMap& cloneMap) const
{
    return new SgScaleTransform(*this, cloneMap);
}


void SgScaleTransform::accept(SceneVisitor& visitor)
{
    visitor.visitScaleTransform(this);
}


const BoundingBox& SgScaleTransform::boundingBox() const
{
    if(isBboxCacheValid){
        return bboxCache;
    }
    bboxCache.clear();
    for(const_iterator p = begin(); p != end(); ++p){
        bboxCache.expandBy((*p)->boundingBox());
    }
    untransformedBboxCache = bboxCache;
    bboxCache.transform(Affine3(scale_.asDiagonal()));
    isBboxCacheValid = true;
    return bboxCache;
}


void SgScaleTransform::getTransform(Affine3& out_T) const
{
    out_T = scale_.asDiagonal();
}


SgSwitch::SgSwitch()
{
    isTurnedOn_ = true;
}


SgSwitch::SgSwitch(const SgSwitch& org)
    : SgGroup(org)
{
    isTurnedOn_ = org.isTurnedOn_;
}


SgSwitch::SgSwitch(const SgSwitch& org, SgCloneMap& cloneMap)
    : SgGroup(org, cloneMap)
{
    isTurnedOn_ = org.isTurnedOn_;
}


SgObject* SgSwitch::clone(SgCloneMap& cloneMap) const
{
    return new SgSwitch(*this, cloneMap);
}


void SgSwitch::accept(SceneVisitor& visitor)
{
    visitor.visitSwitch(this);
}


SgUnpickableGroup::SgUnpickableGroup()
{

}


SgUnpickableGroup::SgUnpickableGroup(const SgUnpickableGroup& org)
    : SgGroup(org)
{

}


SgUnpickableGroup::SgUnpickableGroup(const SgUnpickableGroup& org, SgCloneMap& cloneMap)
    : SgGroup(org, cloneMap)
{

}


SgObject* SgUnpickableGroup::clone(SgCloneMap& cloneMap) const
{
    return new SgUnpickableGroup(*this, cloneMap);
}


void SgUnpickableGroup::accept(SceneVisitor& visitor)
{
    visitor.visitUnpickableGroup(this);
}


SgMaterial::SgMaterial()
{
    ambientIntensity_ = 0.02;
    diffuseColor_ << 0.8, 0.8, 0.8;
    emissiveColor_.setZero();
    specularColor_.setZero();
    shininess_ = 0.2;
    transparency_ = 0.0;
}


SgMaterial::SgMaterial(const SgMaterial& org)
    : SgObject(org)
{
    ambientIntensity_ = org.ambientIntensity_;
    diffuseColor_ = org.diffuseColor_;
    emissiveColor_ = org.emissiveColor_;
    specularColor_ = org.specularColor_;
    shininess_ = org.shininess_;
    transparency_ = org.transparency_;
}


SgObject* SgMaterial::clone(SgCloneMap& cloneMap) const
{
    return new SgMaterial(*this);
}


SgImage::SgImage()
    : image_(boost::make_shared<Image>())
{

}


SgImage::SgImage(const Image& image)
    : image_(boost::make_shared<Image>(image))
{

}


SgImage::SgImage(boost::shared_ptr<Image> sharedImage)
    : image_(sharedImage)
{

}


SgImage::SgImage(const SgImage& org)
    : SgObject(org),
      image_(org.image_)
{

}


SgObject* SgImage::clone(SgCloneMap& cloneMap) const
{
    return new SgImage(*this);
}


Image& SgImage::image()
{
    if(image_.use_count() > 1){
        image_ = boost::make_shared<Image>(*image_);
    }
    return *image_;
}


unsigned char* SgImage::pixels()
{
    if(image_.use_count() > 1){
        image_ = boost::make_shared<Image>(*image_);
    }
    return image_->pixels();
}


void SgImage::setSize(int width, int height, int nComponents)
{
    image().setSize(width, height, nComponents);
}


void SgImage::setSize(int width, int height)
{
    image().setSize(width, height);
}


SgTextureTransform::SgTextureTransform()
{
    center_ << 0.0, 0.0; 
    rotation_ = 0;
    scale_ << 1.0, 1.0;
    translation_ << 0.0, 0.0;
}


SgTextureTransform::SgTextureTransform(const SgTextureTransform& org)
    : SgObject(org)
{
    center_ = org.center_;
    rotation_ = org.rotation_;
    scale_ = org.scale_;
    translation_ = org.translation_;
}


SgObject* SgTextureTransform::clone(SgCloneMap& cloneMap) const
{
    return new SgTextureTransform(*this);
}


SgTexture::SgTexture()
{
    repeatS_ = true; 
    repeatT_ = true; 
}


SgTexture::SgTexture(const SgTexture& org, SgCloneMap& cloneMap)
    : SgObject(org)
{
    if(cloneMap.isNonNodeCloningEnabled()){
        if(org.image()){
            setImage(cloneMap.getClone<SgImage>(org.image()));
        }
        if(org.textureTransform()){
            setTextureTransform(cloneMap.getClone<SgTextureTransform>(org.textureTransform()));
        }
    } else {
        setImage(const_cast<SgImage*>(org.image()));
        setTextureTransform(const_cast<SgTextureTransform*>(org.textureTransform()));
    }
    
    repeatS_ = org.repeatS_;
    repeatT_ = org.repeatT_;
}


SgObject* SgTexture::clone(SgCloneMap& cloneMap) const
{
    return new SgTexture(*this, cloneMap);
}


int SgTexture::numChildObjects() const
{
    int n = 0;
    if(image_) ++n;
    if(textureTransform_) ++n;
    return n;
}


SgObject* SgTexture::childObject(int index)
{
    SgObject* objects[2] = { 0, 0 };
    int i = 0;
    if(image_) objects[i++] = image_;
    if(textureTransform_) objects[i++] = textureTransform_;
    return objects[index];
}


SgImage* SgTexture::setImage(SgImage* image)
{
    if(image_){
        image_->removeParent(this);
    }
    image_ = image;
    if(image){
        image->addParent(this);
    }
    return image;
}


SgImage* SgTexture::getOrCreateImage()
{
    if(!image_){
        setImage(new SgImage);
    }
    return image_;
}    


SgTextureTransform* SgTexture::setTextureTransform(SgTextureTransform* textureTransform)
{
    if(textureTransform_){
        textureTransform_->removeParent(this);
    }
    textureTransform_ = textureTransform;
    if(textureTransform){
        textureTransform->addParent(this);
    }
    return textureTransform;
}


SgMeshBase::SgMeshBase()
{
    isSolid_ = false;
}


SgMeshBase::SgMeshBase(const SgMeshBase& org, SgCloneMap& cloneMap)
    : SgObject(org),
      normalIndices_(org.normalIndices_),
      colorIndices_(org.colorIndices_),
      texCoordIndices_(org.texCoordIndices_)
{
    if(cloneMap.isNonNodeCloningEnabled()){
        if(org.vertices_){
            setVertices(cloneMap.getClone<SgVertexArray>(org.vertices()));
        }
        if(org.normals_){
            setNormals(cloneMap.getClone<SgNormalArray>(org.normals()));
        }
        if(org.colors_){
            setColors(cloneMap.getClone<SgColorArray>(org.colors()));
        }
        if(org.texCoords_){
            setTexCoords(cloneMap.getClone<SgTexCoordArray>(org.texCoords()));
        }
    } else {
        setVertices(const_cast<SgVertexArray*>(org.vertices()));
        setNormals(const_cast<SgNormalArray*>(org.normals()));
        setColors(const_cast<SgColorArray*>(org.colors()));
        setTexCoords(const_cast<SgTexCoordArray*>(org.texCoords()));
    }
    isSolid_ = org.isSolid_;
    bbox = org.bbox;
}

    
int SgMeshBase::numChildObjects() const
{
    int n = 0;
    if(vertices_) ++n;
    if(normals_) ++n;
    if(colors_) ++n;
    return n;
}


SgObject* SgMeshBase::childObject(int index)
{
    SgObject* objects[3] = { 0, 0, 0 };
    int i = 0;
    if(vertices_) objects[i++] = vertices_.get();
    if(normals_) objects[i++] = normals_.get();
    if(colors_) objects[i++] = colors_.get();
    return objects[index];
}


const BoundingBox& SgMeshBase::boundingBox() const
{
    return bbox;
}


void SgMeshBase::updateBoundingBox()
{
    if(!vertices_){
        bbox.clear();
    } else {
        BoundingBoxf bboxf;
        for(SgVertexArray::const_iterator p = vertices_->begin(); p != vertices_->end(); ++p){
            bboxf.expandBy(*p);
        }
        bbox = bboxf;
    }
}


SgVertexArray* SgMeshBase::setVertices(SgVertexArray* vertices)
{
    if(vertices_){
        vertices_->removeParent(this);
    }
    vertices_ = vertices;
    if(vertices){
        vertices->addParent(this);
    }
    return vertices;
}


SgVertexArray* SgMeshBase::getOrCreateVertices()
{
    if(!vertices_){
        setVertices(new SgVertexArray);
    }
    return vertices_;
}


SgNormalArray* SgMeshBase::setNormals(SgNormalArray* normals)
{
    if(normals_){
        normals_->removeParent(this);
    }
    normals_ = normals;
    if(normals){
        normals->addParent(this);
    }
    return normals;
}


SgNormalArray* SgMeshBase::getOrCreateNormals()
{
    if(!normals_){
        setNormals(new SgNormalArray);
    }
    return normals_;
}


SgColorArray* SgMeshBase::setColors(SgColorArray* colors)
{
    if(colors_){
        colors_->removeParent(this);
    }
    colors_ = colors;
    if(colors){
        colors->addParent(this);
    }
    return colors;
}


SgColorArray* SgMeshBase::getOrCreateColors()
{
    if(!colors_){
        setColors(new SgColorArray);
    }
    return colors_;
}


SgTexCoordArray* SgMeshBase::setTexCoords(SgTexCoordArray* texCoords)
{
    if(texCoords_){
        texCoords_->removeParent(this);
    }
    texCoords_ = texCoords;
    if(texCoords){
        texCoords->addParent(this);
    }
    return texCoords;
}


SgMesh::SgMesh()
{

}


SgMesh::SgMesh(const SgMesh& org, SgCloneMap& cloneMap)
    : SgMeshBase(org, cloneMap),
      triangleVertices_(org.triangleVertices_),
      primitive_(org.primitive_)
{

}


SgObject* SgMesh::clone(SgCloneMap& cloneMap) const
{
    return new SgMesh(*this, cloneMap);
}


SgPolygonMesh::SgPolygonMesh()
{

}


SgPolygonMesh::SgPolygonMesh(const SgPolygonMesh& org, SgCloneMap& cloneMap)
    : SgMeshBase(org, cloneMap),
      polygonVertices_(org.polygonVertices_)
{

}
    

SgObject* SgPolygonMesh::clone(SgCloneMap& cloneMap) const
{
    return new SgPolygonMesh(*this, cloneMap);
}


SgShape::SgShape()
{

}


SgShape::SgShape(const SgShape& org, SgCloneMap& cloneMap)
    : SgNode(org)
{
    if(cloneMap.isNonNodeCloningEnabled()){
        if(org.mesh()){
            setMesh(cloneMap.getClone<SgMesh>(org.mesh()));
        }
        if(org.material()){
            setMaterial(cloneMap.getClone<SgMaterial>(org.material()));
        }
        if(org.texture()){
            setTexture(cloneMap.getClone<SgTexture>(org.texture()));
        }
    } else {
        setMesh(const_cast<SgMesh*>(org.mesh()));
        setMaterial(const_cast<SgMaterial*>(org.material()));
        setTexture(const_cast<SgTexture*>(org.texture()));
    }
}


SgObject* SgShape::clone(SgCloneMap& cloneMap) const
{
    return new SgShape(*this, cloneMap);
}


int SgShape::numChildObjects() const
{
    int n = 0;
    if(mesh_) ++n;
    if(material_) ++n;
    if(texture_) ++n;
    return n;
}


SgObject* SgShape::childObject(int index)
{
    SgObject* objects[3] = { 0, 0, 0 };
    int i = 0;
    if(mesh_) objects[i++] = mesh_.get();
    if(material_) objects[i++] = material_.get();
    if(texture_) objects[i++] = texture_.get();
    return objects[index];
}


void SgShape::accept(SceneVisitor& visitor)
{
    visitor.visitShape(this);
}


const BoundingBox& SgShape::boundingBox() const
{
    if(mesh()){
        return mesh()->boundingBox();
    }
    return SgNode::boundingBox();
}


SgMesh* SgShape::setMesh(SgMesh* mesh)
{
    if(mesh_){
        mesh_->removeParent(this);
    }
    mesh_ = mesh;
    if(mesh){
        mesh->addParent(this);
    }
    return mesh;
}


SgMesh* SgShape::getOrCreateMesh()
{
    if(!mesh_){
        setMesh(new SgMesh);
    }
    return mesh_;
}


SgMaterial* SgShape::setMaterial(SgMaterial* material)
{
    if(material_){
        material_->removeParent(this);
    }
    material_ = material;
    if(material){
        material->addParent(this);
    }
    return material;
}


SgMaterial* SgShape::getOrCreateMaterial()
{
    if(!material_){
        setMaterial(new SgMaterial);
    }
    return material_;
}


SgTexture* SgShape::setTexture(SgTexture* texture)
{
    if(texture_){
        texture_->removeParent(this);
    }
    texture_ = texture;
    if(texture){
        texture->addParent(this);
    }
    return texture;
}


SgTexture* SgShape::getOrCreateTexture()
{
    if(!texture_){
        setTexture(new SgTexture);
    }
    return texture_;
}


SgPlot::SgPlot()
{

}
        

SgPlot::SgPlot(const SgPlot& org, SgCloneMap& cloneMap)
    : SgNode(org)
{
    if(cloneMap.isNonNodeCloningEnabled()){
        if(org.vertices()){
            setVertices(cloneMap.getClone<SgVertexArray>(org.vertices()));
        }
        if(org.colors()){
            setColors(cloneMap.getClone<SgColorArray>(org.colors()));
        }
        if(org.material()){
            setMaterial(cloneMap.getClone<SgMaterial>(org.material()));
        }
    } else {
        setVertices(const_cast<SgVertexArray*>(org.vertices()));
        setColors(const_cast<SgColorArray*>(org.colors()));
        setMaterial(const_cast<SgMaterial*>(org.material()));
    }
    normalIndices_ = org.normalIndices_;
    colorIndices_ = org.colorIndices_;
    bbox = org.bbox;
}


int SgPlot::numChildObjects() const
{
    int n = 0;
    if(vertices_) ++n;
    if(colors_) ++n;
    return n;
}
    

SgObject* SgPlot::childObject(int index)
{
    SgObject* objects[2] = { 0, 0 };
    int i = 0;
    if(vertices_) objects[i++] = vertices_.get();
    if(colors_) objects[i++] = colors_.get();
    return objects[index];
}
    

const BoundingBox& SgPlot::boundingBox() const
{
    return bbox;
}


void SgPlot::updateBoundingBox()
{
    if(!vertices_){
        bbox.clear();
    } else {
        BoundingBoxf bboxf;
        for(SgVertexArray::const_iterator p = vertices_->begin(); p != vertices_->end(); ++p){
            bboxf.expandBy(*p);
        }
        bbox = bboxf;
    }
}


SgVertexArray* SgPlot::setVertices(SgVertexArray* vertices)
{
    if(vertices_){
        vertices_->removeParent(this);
    }
    vertices_ = vertices;
    if(vertices){
        vertices->addParent(this);
    }
    return vertices;
}


SgVertexArray* SgPlot::getOrCreateVertices()
{
    if(!vertices_){
        setVertices(new SgVertexArray);
    }
    return vertices_;
}


SgNormalArray* SgPlot::setNormals(SgNormalArray* normals)
{
    if(normals_){
        normals_->removeParent(this);
    }
    normals_ = normals;
    if(normals){
        normals->addParent(this);
    }
    return normals;
}


SgNormalArray* SgPlot::getOrCreateNormals()
{
    if(!normals_){
        setNormals(new SgNormalArray);
    }
    return normals_;
}


SgMaterial* SgPlot::setMaterial(SgMaterial* material)
{
    if(material_){
        material_->removeParent(this);
    }
    material_ = material;
    if(material){
        material->addParent(this);
    }
    return material;
}


SgColorArray* SgPlot::setColors(SgColorArray* colors)
{
    if(colors_){
        colors_->removeParent(this);
    }
    colors_ = colors;
    if(colors){
        colors->addParent(this);
    }
    return colors;
}


SgColorArray* SgPlot::getOrCreateColors()
{
    if(!colors_){
        setColors(new SgColorArray);
    }
    return colors_;
}


SgPointSet::SgPointSet()
{
    pointSize_ = 0.0;
}


SgPointSet::SgPointSet(const SgPointSet& org, SgCloneMap& cloneMap)
    : SgPlot(org, cloneMap)
{
    pointSize_ = org.pointSize_;
}


SgObject* SgPointSet::clone(SgCloneMap& cloneMap) const
{
    return new SgPointSet(*this, cloneMap);
}


void SgPointSet::accept(SceneVisitor& visitor)
{
    visitor.visitPointSet(this);
}

   
SgLineSet::SgLineSet()
{
    lineWidth_ = 0.0;
}


SgLineSet::SgLineSet(const SgLineSet& org, SgCloneMap& cloneMap)
    : SgPlot(org, cloneMap)
{
    lineWidth_ = org.lineWidth_;
}

    
SgObject* SgLineSet::clone(SgCloneMap& cloneMap) const
{
    return new SgLineSet(*this, cloneMap);
}
    

void SgLineSet::accept(SceneVisitor& visitor)
{
    visitor.visitLineSet(this);
}


SgPreprocessed::SgPreprocessed()
{

}


SgPreprocessed::SgPreprocessed(const SgPreprocessed& org)
    : SgNode(org)
{

}


SgObject* SgPreprocessed::clone(SgCloneMap& cloneMap) const
{
    return new SgPreprocessed(*this);
}


void SgPreprocessed::accept(SceneVisitor& visitor)
{
    visitor.visitPreprocessed(this);
}


SgLight::SgLight()
{
    on_ = true;
    color_.setOnes();
    intensity_ = 1.0f;
    ambientIntensity_ = 0.0f;
}


SgLight::SgLight(const SgLight& org)
    : SgPreprocessed(org)
{
    on_ = org.on_;
    color_ = org.color_;
    intensity_ = org.intensity_;
    ambientIntensity_ = org.ambientIntensity_;
}


SgObject* SgLight::clone(SgCloneMap& cloneMap) const
{
    return new SgLight(*this);
}


void SgLight::accept(SceneVisitor& visitor)
{
    visitor.visitLight(this);
}


SgDirectionalLight::SgDirectionalLight()
{
    direction_ << 0.0, 0.0, -1.0;
}


SgDirectionalLight::SgDirectionalLight(const SgDirectionalLight& org)
    : SgLight(org)
{
    direction_ = org.direction_;
}


SgObject* SgDirectionalLight::clone(SgCloneMap& cloneMap) const
{
    return new SgDirectionalLight(*this);
}


void SgDirectionalLight::accept(SceneVisitor& visitor)
{
    visitor.visitLight(this);
}


SgPointLight::SgPointLight()
{
    constantAttenuation_ = 1.0f;
    linearAttenuation_ = 0.0f;
    quadraticAttenuation_ = 0.0f;
}


SgPointLight::SgPointLight(const SgPointLight& org)
    : SgLight(org)
{
    constantAttenuation_ = org.constantAttenuation_;
    linearAttenuation_ = org.linearAttenuation_;
    quadraticAttenuation_ = org.quadraticAttenuation_;
}


SgObject* SgPointLight::clone(SgCloneMap& cloneMap) const
{
    return new SgPointLight(*this);
}


void SgPointLight::accept(SceneVisitor& visitor)
{
    visitor.visitLight(this);
}


SgSpotLight::SgSpotLight()
{
    direction_ << 0.0, 0.0, -1.0;
    beamWidth_ = 1.570796f;
    cutOffAngle_ = 0.785398f;
}


SgSpotLight::SgSpotLight(const SgSpotLight& org)
    : SgPointLight(org)
{
    direction_ = org.direction_;
    beamWidth_ = org.beamWidth_;
    cutOffAngle_ = org.cutOffAngle_;
}


SgObject* SgSpotLight::clone(SgCloneMap& cloneMap) const
{
    return new SgSpotLight(*this);
}


void SgSpotLight::accept(SceneVisitor& visitor)
{
    visitor.visitLight(this);
}


SgCamera::SgCamera()
{
    nearDistance_ = 0.01;
    farDistance_ = 100.0;
}


SgCamera::SgCamera(const SgCamera& org)
    : SgPreprocessed(org)
{
    nearDistance_ = org.nearDistance_;
    farDistance_ = org.farDistance_;
}


void SgCamera::accept(SceneVisitor& visitor)
{
    visitor.visitCamera(this);
}


Affine3 SgCamera::positionLookingFor(const Vector3& eye, const Vector3& direction, const Vector3& up)
{
    Vector3 d = direction.normalized();
    Vector3 c = d.cross(up).normalized();
    Vector3 u = c.cross(d);
    Affine3 C;
    C.linear() << c, u, -d;
    C.translation() = eye;
    return C;
}


Affine3 SgCamera::positionLookingAt(const Vector3& eye, const Vector3& center, const Vector3& up)
{
    return positionLookingFor(eye, (center - eye), up);
}


SgPerspectiveCamera::SgPerspectiveCamera()
{
    fieldOfView_ = 0.785398;
}


SgPerspectiveCamera::SgPerspectiveCamera(const SgPerspectiveCamera& org)
    : SgCamera(org)
{
    fieldOfView_ = org.fieldOfView_;
}


SgObject* SgPerspectiveCamera::clone(SgCloneMap& cloneMap) const
{
    return new SgPerspectiveCamera(*this);
}


void SgPerspectiveCamera::accept(SceneVisitor& visitor)
{
    visitor.visitCamera(this);
}


/**
   @param aspectRatio width / height
*/
double SgPerspectiveCamera::fovy(double aspectRatio, double fieldOfView)
{
    if(aspectRatio >= 1.0){
        return fieldOfView;
    } else {
        return 2.0 * atan(tan(fieldOfView / 2.0) / aspectRatio);
    }
}


SgOrthographicCamera::SgOrthographicCamera()
{
    height_ = 2.0;
}


SgOrthographicCamera::SgOrthographicCamera(const SgOrthographicCamera& org)
    : SgCamera(org)
{
    height_ = org.height_;
}


SgObject* SgOrthographicCamera::clone(SgCloneMap& cloneMap) const
{
    return new SgOrthographicCamera(*this);
}


void SgOrthographicCamera::accept(SceneVisitor& visitor)
{
    visitor.visitCamera(this);
}


SgFog::SgFog()
{

}


SgFog::SgFog(const SgFog& org)
    : SgPreprocessed(org)
{

}


SgObject* SgFog::clone(SgCloneMap& cloneMap) const
{
    return new SgFog(*this);
}


void SgFog::accept(SceneVisitor& visitor)
{
    
}


SgOverlay::SgOverlay()
{

}


SgOverlay::SgOverlay(const SgOverlay& org, SgCloneMap& cloneMap)
    : SgGroup(org, cloneMap)
{

}


SgOverlay::~SgOverlay()
{

}


SgObject* SgOverlay::clone(SgCloneMap& cloneMap) const
{
    return new SgOverlay(*this, cloneMap);
}


void SgOverlay::accept(SceneVisitor& visitor)
{
    visitor.visitOverlay(this);
}


void SgOverlay::calcViewVolume(double viewportWidth, double viewportHeight, ViewVolume& io_volume)
{

}


SgOutlineGroup::SgOutlineGroup()
{
    lineWidth_ = 1.0;
    color_ << 1.0, 0.0, 0.0;
}


void SgOutlineGroup::accept(SceneVisitor& visitor)
{
    visitor.visitOutlineGroup(this);
}
