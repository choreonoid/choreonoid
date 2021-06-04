/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "SceneDrawables.h"
#include "CloneMap.h"
#include "SceneNodeClassRegistry.h"

using namespace std;
using namespace cnoid;

namespace {

const bool USE_FACES_FOR_BOUNDING_BOX_CALCULATION = true;

struct NodeClassRegistration {
    NodeClassRegistration() {
        SceneNodeClassRegistry::instance()
            .registerClass<SgShape, SgNode>()
            .registerClass<SgPlot, SgNode>()
            .registerClass<SgPointSet, SgPlot>()
            .registerClass<SgLineSet, SgPlot>()
            .registerClass<SgOverlay, SgGroup>()
            .registerClass<SgViewportOverlay, SgOverlay>();
    }
} registration;

}

SgMaterial::SgMaterial()
{
    setAttribute(Appearance);
    
    ambientIntensity_ = 1.0f;
    diffuseColor_ << 1.0f, 1.0f, 1.0f;
    emissiveColor_.setZero();
    specularColor_.setZero();
    specularExponent_ = 25.0f;
    transparency_ = 0.0f;
}


SgMaterial::SgMaterial(const SgMaterial& org)
    : SgObject(org)
{
    ambientIntensity_ = org.ambientIntensity_;
    diffuseColor_ = org.diffuseColor_;
    emissiveColor_ = org.emissiveColor_;
    specularColor_ = org.specularColor_;
    specularExponent_ = org.specularExponent_;
    transparency_ = org.transparency_;
}


Referenced* SgMaterial::doClone(CloneMap*) const
{
    return new SgMaterial(*this);
}


float SgMaterial::shininess() const
{
    return std::max(0.0f, std::min(specularExponent_ - 1.0f, 127.0f)) / 127.0f;
}


void SgMaterial::setShininess(float s)
{
    specularExponent_ = 127.0f * std::max(0.0f, std::min(s, 1.0f)) + 1.0f;
}


SgImage::SgImage()
    : image_(std::make_shared<Image>())
{
    setAttribute(Appearance);
}


SgImage::SgImage(const Image& image)
    : image_(std::make_shared<Image>(image))
{
    setAttribute(Appearance);
}


SgImage::SgImage(std::shared_ptr<Image> sharedImage)
    : image_(sharedImage)
{
    setAttribute(Appearance);
}


SgImage::SgImage(const SgImage& org)
    : SgObject(org),
      image_(org.image_)
{

}


Referenced* SgImage::doClone(CloneMap*) const
{
    return new SgImage(*this);
}


Image& SgImage::image()
{
    if(image_.use_count() > 1){
        image_ = std::make_shared<Image>(*image_);
    }
    return *image_;
}


unsigned char* SgImage::pixels()
{
    if(image_.use_count() > 1){
        image_ = std::make_shared<Image>(*image_);
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
    setAttribute(Appearance);
    
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


Referenced* SgTextureTransform::doClone(CloneMap*) const
{
    return new SgTextureTransform(*this);
}


SgTexture::SgTexture()
{
    setAttributes(Composite | Appearance);
    repeatS_ = true; 
    repeatT_ = true; 
}


SgTexture::SgTexture(const SgTexture& org, CloneMap* cloneMap)
    : SgObject(org)
{
    if(cloneMap && checkNonNodeCloning(*cloneMap)){
        if(org.image()){
            setImage(cloneMap->getClone<SgImage>(org.image()));
        }
        if(org.textureTransform()){
            setTextureTransform(cloneMap->getClone<SgTextureTransform>(org.textureTransform()));
        }
    } else {
        setImage(const_cast<SgImage*>(org.image()));
        setTextureTransform(const_cast<SgTextureTransform*>(org.textureTransform()));
    }
    
    repeatS_ = org.repeatS_;
    repeatT_ = org.repeatT_;
}


SgTexture::~SgTexture()
{
    if(image_){
        image_->removeParent(this);
    }
    if(textureTransform_){
        textureTransform_->removeParent(this);
    }
}    


Referenced* SgTexture::doClone(CloneMap* cloneMap) const
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


SgTextureTransform* SgTexture::getOrCreateTextureTransform()
{
    if(!textureTransform_){
        setTextureTransform(new SgTextureTransform);
    }
    return textureTransform_;
}


SgMeshBase::SgMeshBase()
{
    setAttribute(Composite | Geometry);
    creaseAngle_ = 0.0f;
    isSolid_ = false;
}


SgMeshBase::SgMeshBase(const SgMeshBase& org, CloneMap* cloneMap)
    : SgObject(org),
      faceVertexIndices_(org.faceVertexIndices_),
      normalIndices_(org.normalIndices_),
      colorIndices_(org.colorIndices_),
      texCoordIndices_(org.texCoordIndices_)
{
    if(cloneMap && checkNonNodeCloning(*cloneMap)){
        if(org.vertices_){
            setVertices(cloneMap->getClone<SgVertexArray>(org.vertices()));
        }
        if(org.normals_){
            setNormals(cloneMap->getClone<SgNormalArray>(org.normals()));
        }
        if(org.colors_){
            setColors(cloneMap->getClone<SgColorArray>(org.colors()));
        }
        if(org.texCoords_){
            setTexCoords(cloneMap->getClone<SgTexCoordArray>(org.texCoords()));
        }
    } else {
        setVertices(const_cast<SgVertexArray*>(org.vertices()));
        setNormals(const_cast<SgNormalArray*>(org.normals()));
        setColors(const_cast<SgColorArray*>(org.colors()));
        setTexCoords(const_cast<SgTexCoordArray*>(org.texCoords()));
    }
    creaseAngle_ = org.creaseAngle_;
    isSolid_ = org.isSolid_;
    bbox = org.bbox;
}


SgMeshBase::~SgMeshBase()
{
    if(vertices_){
        vertices_->removeParent(this);
    }
    if(normals_){
        normals_->removeParent(this);
    }
    if(colors_){
        colors_->removeParent(this);
    }
    if(texCoords_){
        texCoords_->removeParent(this);
    }
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
        vertices->setAttribute(Geometry);
        vertices->addParent(this);
    }
    return vertices;
}


SgVertexArray* SgMeshBase::getOrCreateVertices(int size)
{
    if(!vertices_){
        setVertices(new SgVertexArray(size));
    } else if(size > 0){
        vertices_->resize(size);
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
        normals->setAttribute(Appearance);
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
        colors->setAttribute(Appearance);
        colors->addParent(this);
    }
    return colors;
}


SgColorArray* SgMeshBase::getOrCreateColors(int size)
{
    if(!colors_){
        setColors(new SgColorArray(size));
    } else if(size > 0){
        colors_->resize(size);
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
        texCoords->setAttribute(Appearance);
        texCoords->addParent(this);
    }
    return texCoords;
}


SgTexCoordArray* SgMeshBase::getOrCreateTexCoords()
{
    if(!texCoords_){
        setTexCoords(new SgTexCoordArray);
    }
    return texCoords_;
}


SgMesh::SgMesh()
{
    divisionNumber_ = -1;
    extraDivisionNumber_ = -1;
    extraDivisionMode_ = ExtraDivisionPreferred;
}


SgMesh::SgMesh(Primitive primitive)
    : SgMesh()
{
    primitive_ = primitive;
}
    

SgMesh::SgMesh(const SgMesh& org, CloneMap* cloneMap)
    : SgMeshBase(org, cloneMap),
      primitive_(org.primitive_),
      divisionNumber_(org.divisionNumber_),
      extraDivisionNumber_(org.extraDivisionNumber_),
      extraDivisionMode_(org.extraDivisionMode_)
{

}


Referenced* SgMesh::doClone(CloneMap* cloneMap) const
{
    return new SgMesh(*this, cloneMap);
}


void SgMesh::updateBoundingBox()
{
    if(!USE_FACES_FOR_BOUNDING_BOX_CALCULATION){
        SgMeshBase::updateBoundingBox();

    } else {
        if(!hasVertices()){
            bbox.clear();
        } else {
            BoundingBoxf bboxf;
            const SgVertexArray& v = *vertices();
            for(auto& index : faceVertexIndices_){
                bboxf.expandBy(v[index]);
            }
            bbox = bboxf;
        }
    }
}


void SgMesh::transform(const Affine3& T)
{
    if(hasVertices()){
        auto& v = *vertices();
        for(size_t i=0; i < v.size(); ++i){
            v[i] = (T * v[i].cast<Affine3::Scalar>()).cast<Vector3f::Scalar>();
        }
        if(hasNormals()){
            Matrix3 R = T.linear();
            auto& n = *normals();
            for(size_t i=0; i < n.size(); ++i){
                n[i] = (R * n[i].cast<Affine3::Scalar>()).cast<Vector3f::Scalar>();
            }
        }
    }
    setPrimitive(SgMesh::Mesh()); // clear the primitive information
}


void SgMesh::transform(const Affine3f& T)
{
    if(hasVertices()){
        auto& v = *vertices();
        for(size_t i=0; i < v.size(); ++i){
            v[i] = T.linear() * v[i] + T.translation();
        }
        if(hasNormals()){
            auto& n = *normals();
            for(size_t i=0; i < n.size(); ++i){
                n[i] = T.linear() * n[i];
            }
        }
    }
    setPrimitive(SgMesh::Mesh()); // clear the primitive information
}


void SgMesh::translate(const Vector3f& translation)
{
    if(hasVertices()){
        auto& v = *vertices();
        for(size_t i=0; i < v.size(); ++i){
            v[i] += translation;
        }
    }
    setPrimitive(SgMesh::Mesh()); // clear the primitive information
}
    

void SgMesh::rotate(const Matrix3f& R)
{
    if(hasVertices()){
        auto& v = *vertices();
        for(size_t i=0; i < v.size(); ++i){
            v[i] = R * v[i];
        }
        if(hasNormals()){
            auto& n = *normals();
            for(size_t i=0; i < n.size(); ++i){
                n[i] = R * n[i];
            }
        }
    }
    setPrimitive(SgMesh::Mesh()); // clear the primitive information
}    
    

SgPolygonMesh::SgPolygonMesh()
{

}


SgPolygonMesh::SgPolygonMesh(const SgPolygonMesh& org, CloneMap* cloneMap)
    : SgMeshBase(org, cloneMap)
{

}
    

Referenced* SgPolygonMesh::doClone(CloneMap* cloneMap) const
{
    return new SgPolygonMesh(*this, cloneMap);
}


void SgPolygonMesh::updateBoundingBox()
{
    if(!USE_FACES_FOR_BOUNDING_BOX_CALCULATION){
        SgMeshBase::updateBoundingBox();

    } else {
        if(!hasVertices()){
            bbox.clear();
        } else {
            BoundingBoxf bboxf;
            const SgVertexArray& v = *vertices();
            for(auto& index : faceVertexIndices_){
                if(index >= 0){
                    bboxf.expandBy(v[index]);
                }
            }
            bbox = bboxf;
        }
    }
}


SgShape::SgShape(int classId)
    : SgNode(classId)
{
    setAttribute(Composite | Geometry | Appearance);
}


SgShape::SgShape()
    : SgShape(findClassId<SgShape>())
{

}


SgShape::SgShape(const SgShape& org, CloneMap* cloneMap)
    : SgNode(org)
{
    if(cloneMap && checkNonNodeCloning(*cloneMap)){
        if(org.mesh()){
            setMesh(cloneMap->getClone<SgMesh>(org.mesh()));
        }
        if(org.material()){
            setMaterial(cloneMap->getClone<SgMaterial>(org.material()));
        }
        if(org.texture()){
            setTexture(cloneMap->getClone<SgTexture>(org.texture()));
        }
    } else {
        setMesh(const_cast<SgMesh*>(org.mesh()));
        setMaterial(const_cast<SgMaterial*>(org.material()));
        setTexture(const_cast<SgTexture*>(org.texture()));
    }
}


SgShape::~SgShape()
{
    if(mesh_){
        mesh_->removeParent(this);
    }
    if(material_){
        material_->removeParent(this);
    }
    if(texture_){
        texture_->removeParent(this);
    }
}    


Referenced* SgShape::doClone(CloneMap* cloneMap) const
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


const BoundingBox& SgShape::boundingBox() const
{
    if(mesh()){
        return mesh()->boundingBox();
    }
    return SgNode::boundingBox();
}


const BoundingBox& SgShape::untransformedBoundingBox() const
{
    return SgShape::boundingBox();
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


SgPlot::SgPlot(int classId)
    : SgNode(classId)
{
    setAttribute(Composite | Geometry | Appearance);
}
        

SgPlot::SgPlot(const SgPlot& org, CloneMap* cloneMap)
    : SgNode(org)
{
    bbox = org.bbox;

    if(cloneMap && checkNonNodeCloning(*cloneMap)){
        if(org.vertices()){
            setVertices(cloneMap->getClone<SgVertexArray>(org.vertices()));
        }
        if(org.material()){
            setMaterial(cloneMap->getClone<SgMaterial>(org.material()));
        }
        if(org.colors()){
            setColors(cloneMap->getClone<SgColorArray>(org.colors()));
        }
        if(org.normals()){
            setNormals(cloneMap->getClone<SgNormalArray>(org.normals()));
        }
    } else {
        setVertices(const_cast<SgVertexArray*>(org.vertices()));
        setMaterial(const_cast<SgMaterial*>(org.material()));
        setColors(const_cast<SgColorArray*>(org.colors()));
        setNormals(const_cast<SgNormalArray*>(org.normals()));
    }
    
    colorIndices_ = org.colorIndices_;
    normalIndices_ = org.normalIndices_;
}


SgPlot::~SgPlot()
{
    if(vertices_){
        vertices_->removeParent(this);
    }
    if(material_){
        material_->removeParent(this);
    }
    if(colors_){
        colors_->removeParent(this);
    }
    if(normals_){
        normals_->removeParent(this);
    }
}    


int SgPlot::numChildObjects() const
{
    int n = 0;
    if(vertices_) ++n;
    if(colors_) ++n;
    if(normals_) ++n;
    return n;
}
    

SgObject* SgPlot::childObject(int index)
{
    SgObject* objects[3] = { nullptr, nullptr, nullptr };
    int i = 0;
    if(vertices_) objects[i++] = vertices_.get();
    if(colors_) objects[i++] = colors_.get();
    if(normals_) objects[i++] = normals_.get();
    return objects[index];
}
    

const BoundingBox& SgPlot::boundingBox() const
{
    return bbox;
}


const BoundingBox& SgPlot::untransformedBoundingBox() const
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


void SgPlot::clear()
{
    if(vertices_){
        vertices_->clear();
    }
    
    if(colors_){
        colors_->clear();
    }
    colorIndices_.clear();

    if(normals_){
        normals_->clear();
    }
    normalIndices_.clear();
}


SgVertexArray* SgPlot::setVertices(SgVertexArray* vertices)
{
    if(vertices_){
        vertices_->removeParent(this);
    }
    vertices_ = vertices;
    if(vertices){
        vertices->setAttribute(Geometry);
        vertices->addParent(this);
    }
    return vertices;
}


SgVertexArray* SgPlot::getOrCreateVertices(int size)
{
    if(!vertices_){
        setVertices(new SgVertexArray(size));
    } else if(size > 0){
        vertices_->resize(size);
    }
    return vertices_;
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


SgMaterial* SgPlot::getOrCreateMaterial()
{
    if(!material_){
        setMaterial(new SgMaterial);
    }
    return material_;
}


SgColorArray* SgPlot::setColors(SgColorArray* colors)
{
    if(colors_){
        colors_->removeParent(this);
    }
    colors_ = colors;
    if(colors){
        colors->setAttribute(Appearance);
        colors->addParent(this);
    }
    return colors;
}


SgColorArray* SgPlot::getOrCreateColors(int size)
{
    if(!colors_){
        setColors(new SgColorArray(size));
    } else if(size > 0){
        colors_->resize(size);
    }
    return colors_;
}


SgNormalArray* SgPlot::setNormals(SgNormalArray* normals)
{
    if(normals_){
        normals_->removeParent(this);
    }
    normals_ = normals;
    if(normals){
        normals->setAttribute(Appearance);
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


SgPointSet::SgPointSet(int classId)
    : SgPlot(classId)
{
    pointSize_ = 0.0;
}


SgPointSet::SgPointSet()
    : SgPointSet(findClassId<SgPointSet>())
{

}


SgPointSet::SgPointSet(const SgPointSet& org, CloneMap* cloneMap)
    : SgPlot(org, cloneMap)
{
    pointSize_ = org.pointSize_;
}


Referenced* SgPointSet::doClone(CloneMap* cloneMap) const
{
    return new SgPointSet(*this, cloneMap);
}


SgLineSet::SgLineSet(int classId)
    : SgPlot(classId)
{
    lineWidth_ = 0.0;
}


SgLineSet::SgLineSet()
    : SgLineSet(findClassId<SgLineSet>())
{
    lineWidth_ = 0.0;
}


SgLineSet::SgLineSet(const SgLineSet& org, CloneMap* cloneMap)
    : SgPlot(org, cloneMap),
      lineVertexIndices_(org.lineVertexIndices_)
{
    lineWidth_ = org.lineWidth_;
}

    
Referenced* SgLineSet::doClone(CloneMap* cloneMap) const
{
    return new SgLineSet(*this, cloneMap);
}
    

SgOverlay::SgOverlay(int classId)
    : SgGroup(classId)
{

}


SgOverlay::SgOverlay()
    : SgGroup(findClassId<SgOverlay>())
{

}


SgOverlay::SgOverlay(const SgOverlay& org, CloneMap* cloneMap)
    : SgGroup(org, cloneMap)
{

}


SgOverlay::~SgOverlay()
{

}


Referenced* SgOverlay::doClone(CloneMap* cloneMap) const
{
    return new SgOverlay(*this, cloneMap);
}


SgViewportOverlay::SgViewportOverlay(int classId)
    : SgOverlay(classId)
{

}


SgViewportOverlay::SgViewportOverlay()
    : SgOverlay(findClassId<SgViewportOverlay>())
{

}


SgViewportOverlay::SgViewportOverlay(const SgViewportOverlay& org, CloneMap* cloneMap)
    : SgOverlay(org, cloneMap)
{

}


SgViewportOverlay::~SgViewportOverlay()
{

}


Referenced* SgViewportOverlay::doClone(CloneMap* cloneMap) const
{
    return new SgViewportOverlay(*this, cloneMap);
}


void SgViewportOverlay::calcViewVolume(double /* viewportWidth */, double /* viewportHeight */, ViewVolume& io_volume)
{
    io_volume.left = -1.0;
    io_volume.right = 1.0;
    io_volume.bottom = -1.0;
    io_volume.top = 1.0;
    io_volume.zNear = 1.0;
    io_volume.zFar = -1.0;
}
