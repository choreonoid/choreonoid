/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_SCENE_DRAWABLES_H
#define CNOID_UTIL_SCENE_DRAWABLES_H

#include "SceneGraph.h"
#include "Image.h"
#include <cnoid/stdx/variant>
#include <memory>
#include <initializer_list>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT SgMaterial : public SgObject
{
public:
    SgMaterial();
    SgMaterial(const SgMaterial& org);
    virtual SgObject* doClone(SgCloneMap* cloneMap) const override;

    float ambientIntensity() const { return ambientIntensity_; }
    void setAmbientIntensity(float intensity) { ambientIntensity_ = intensity; }
    const Vector3f& diffuseColor() const { return diffuseColor_; }
    template<typename Derived> void setDiffuseColor(const Eigen::MatrixBase<Derived>& c) {
        diffuseColor_ = c.template cast<Vector3f::Scalar>(); }
    const Vector3f& emissiveColor() const { return emissiveColor_; }
    template<typename Derived> void setEmissiveColor(const Eigen::MatrixBase<Derived>& c) {
        emissiveColor_ = c.template cast<Vector3f::Scalar>(); }
    float shininess() const { return shininess_; }
    void setShininess(float s) { shininess_ = s; }
    const Vector3f& specularColor() const { return specularColor_; }
    template<typename Derived> void setSpecularColor(const Eigen::MatrixBase<Derived>& c) {
        specularColor_ = c.template cast<Vector3f::Scalar>(); }
    float transparency() const { return transparency_; }
    void setTransparency(float t) { transparency_ = t; }

private:
    Vector3f diffuseColor_;
    Vector3f emissiveColor_;
    Vector3f specularColor_;
    float ambientIntensity_;
    float transparency_;
    float shininess_;
};
typedef ref_ptr<SgMaterial> SgMaterialPtr;


class CNOID_EXPORT SgImage : public SgObject
{
public:
    SgImage();
    SgImage(const Image& image);
    SgImage(std::shared_ptr<Image> sharedImage);
    SgImage(const SgImage& org);
    virtual SgObject* doClone(SgCloneMap* cloneMap) const override;

    Image& image();
    const Image& image() const { return *image_; }
    const Image& constImage() const { return *image_; }

    bool empty() const { return image_->empty(); }
        
    unsigned char* pixels();
    const unsigned char* pixels() const { return image_->pixels(); }
    const unsigned char* constPixels() const { return image_->pixels(); }

    int width() const { return image_->width(); }
    int height() const { return image_->height(); }
    int numComponents() const { return image_->numComponents(); }
        
    void setSize(int width, int height, int nComponents);
    void setSize(int width, int height);

private:
    std::shared_ptr<Image> image_;
};
typedef ref_ptr<SgImage> SgImagePtr;


class CNOID_EXPORT SgTextureTransform : public SgObject
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SgTextureTransform();
    SgTextureTransform(const SgTextureTransform& org);
    virtual SgObject* doClone(SgCloneMap* cloneMap) const override;

    const Vector2& center() const { return center_; }
    template<typename Derived> void setCenter(const Eigen::MatrixBase<Derived>& c) {
        center_ = c.template cast<Vector3::Scalar>(); }
    double rotation() const { return rotation_; }
    void setRotation(double rotation) { rotation_ = rotation; }
    const Vector2& scale() const { return scale_; }
    template<typename Derived> void setScale(const Eigen::MatrixBase<Derived>& c) {
        scale_ = c.template cast<Vector2::Scalar>(); }
    const Vector2& translation() const { return translation_; }
    template<typename Derived> void setTranslation(const Eigen::MatrixBase<Derived>& c) {
        translation_ = c.template cast<Vector3::Scalar>(); }

private:
    Vector2 center_;
    Vector2 scale_;
    Vector2 translation_;
    double rotation_;
};
typedef ref_ptr<SgTextureTransform> SgTextureTransformPtr;


class CNOID_EXPORT SgTexture : public SgObject
{
public:
    SgTexture();
    SgTexture(const SgTexture& org, SgCloneMap* cloneMap = nullptr);
    ~SgTexture();
    
    virtual SgObject* doClone(SgCloneMap* cloneMap) const override;
    virtual int numChildObjects() const override;
    virtual SgObject* childObject(int index) override;

    SgImage* image() { return image_; }
    const SgImage* image() const { return image_; }
    SgImage* setImage(SgImage* image);
    SgImage* getOrCreateImage();

    bool repeatS() const { return repeatS_; }
    bool repeatT() const { return repeatT_; }
    void setRepeat(bool s, bool t) { repeatS_ = s; repeatT_ = t; }

    SgTextureTransform* textureTransform() { return textureTransform_; }
    const SgTextureTransform* textureTransform() const { return textureTransform_; }
    SgTextureTransform* setTextureTransform(SgTextureTransform* textureTransform);

private:
    SgImagePtr image_;
    SgTextureTransformPtr textureTransform_;
    bool repeatS_;
    bool repeatT_;
};
typedef ref_ptr<SgTexture> SgTexturePtr;


template<class T, class Alloc = std::allocator<T>> class SgVectorArray : public SgObject
{
    typedef std::vector<T> Container;
        
public:
    typedef typename Container::iterator iterator;
    typedef typename Container::const_iterator const_iterator;
    typedef typename Container::size_type size_type;
    typedef typename Container::value_type value_type;
    typedef typename Container::reference reference;
    typedef typename Container::const_reference const_reference;
    typedef typename Container::pointer pointer;
    typedef typename Container::const_pointer const_pointer;
    typedef typename T::Scalar Scalar;

    SgVectorArray() { }
    SgVectorArray(size_t size) : values(size) { }
    SgVectorArray(const std::vector<T>& org) : values(org) { }
    SgVectorArray(std::initializer_list<T> init) : values(init.begin(), init.end()) { }

    template<class Element>
    SgVectorArray(const std::vector<Element>& org) {
        values.reserve(org.size());
        for(typename std::vector<Element>::const_iterator p = org.begin(); p != org.end(); ++p){
            values.push_back(p->template cast<typename T::Scalar>());
        }
    }
        
    SgVectorArray(const SgVectorArray& org) : SgObject(org), values(org.values) { }

    virtual SgObject* doClone(SgCloneMap*) const override { return new SgVectorArray(*this); }
        
    SgVectorArray<T>& operator=(const SgVectorArray<T>& rhs) {
        values = rhs.values;
        return *this;
    }
    iterator begin() { return values.begin(); }
    const_iterator begin() const { return values.begin(); }
    iterator end() { return values.end(); }
    const_iterator end() const { return values.end(); }
    size_type size() const { return values.size(); }
    void resize(size_type s) { values.resize(s); }
    void resize(size_type s, const T& v) { values.resize(s, v); }
    bool empty() const { return values.empty(); }
    void reserve(size_type s) { values.reserve(s); }
    size_type capacity() const { return values.capacity(); }
    T& operator[](size_type i) { return values[i]; }
    const T& operator[](size_type i) const { return values[i]; }
    T& at(size_type i) { return values[i]; }
    const T& at(size_type i) const { return values[i]; }
    T& front() { return values.front(); }
    const T& front() const { return values.front(); }
    T& back() { return values.back(); }
    const T& back() const { return values.back(); }
    Scalar* data() { return values.front().data(); }
    const Scalar* data() const { return values.front().data(); }
    void push_back(const T& v) { values.push_back(v); }
    void pop_back() { values.pop_back(); }
    iterator erase(iterator p) { return values.erase(p); }
    iterator erase(iterator first, iterator last) { return values.erase(first, last); }
    void clear() { values.clear(); }
    void shrink_to_fit() { values.shrink_to_fit(); }

private:
    Container values;
};

typedef SgVectorArray<Vector3f> SgVertexArray;
typedef ref_ptr<SgVertexArray> SgVertexArrayPtr;

typedef SgVectorArray<Vector3f> SgNormalArray;
typedef ref_ptr<SgNormalArray> SgNormalArrayPtr;

typedef SgVectorArray<Vector3f> SgColorArray;
typedef ref_ptr<SgColorArray> SgColorArrayPtr;

typedef SgVectorArray<Vector2f, Eigen::aligned_allocator<Vector2f>> SgTexCoordArray;
typedef ref_ptr<SgTexCoordArray> SgTexCoordArrayPtr;

typedef std::vector<int> SgIndexArray;


class CNOID_EXPORT SgMeshBase : public SgObject
{
protected:
    SgMeshBase();
    SgMeshBase(const SgMeshBase& org, SgCloneMap* cloneMap = nullptr);
    ~SgMeshBase();
    
public:
    virtual int numChildObjects() const override;
    virtual SgObject* childObject(int index) override;

    const BoundingBox& boundingBox() const { return bbox; }
    virtual void updateBoundingBox();
    void setBoundingBox(const BoundingBox& bb){ bbox = bb; };
    void setBoundingBox(const BoundingBoxf& bb){ bbox = bb; };

    bool hasVertices() const { return (vertices_ && !vertices_->empty()); }
    SgVertexArray* vertices() { return vertices_; }
    const SgVertexArray* vertices() const { return vertices_; }
    SgVertexArray* setVertices(SgVertexArray* vertices);
    SgVertexArray* getOrCreateVertices(size_t size = 0);
        
    bool hasNormals() const { return (normals_ && !normals_->empty()); }
    SgNormalArray* normals() { return normals_; }
    const SgNormalArray* normals() const { return normals_; }
    SgNormalArray* setNormals(SgNormalArray* normals);
    SgNormalArray* getOrCreateNormals();
        
    bool hasColors() const { return (colors_ && !colors_->empty()); }
    SgColorArray* colors() { return colors_; }
    const SgColorArray* colors() const { return colors_; }
    SgColorArray* setColors(SgColorArray* colors);
    SgColorArray* getOrCreateColors();

    bool hasTexCoords() const { return (texCoords_ && !texCoords_->empty()); }
    SgTexCoordArray* texCoords() { return texCoords_; }
    const SgTexCoordArray* texCoords() const { return texCoords_; }
    SgTexCoordArray* setTexCoords(SgTexCoordArray* texCoords);
    SgTexCoordArray* getOrCreateTexCoords();

    /**
       Normals are assinged for vertices in triangles.
    */
    bool hasNormalIndices() const { return !normalIndices_.empty(); }
    const SgIndexArray& normalIndices() const { return normalIndices_; }
    SgIndexArray& normalIndices() { return normalIndices_; }

    bool hasColorIndices() const { return !colorIndices_.empty(); }
    const SgIndexArray& colorIndices() const { return colorIndices_; }
    SgIndexArray& colorIndices() { return colorIndices_; }

    bool hasTexCoordIndices() const { return !texCoordIndices_.empty(); }
    const SgIndexArray& texCoordIndices() const { return texCoordIndices_; }
    SgIndexArray& texCoordIndices() { return texCoordIndices_; }

    bool isSolid() const { return isSolid_; }
    void setSolid(bool on) { isSolid_ = on; }

protected:
    BoundingBox bbox;
    
private:
    SgVertexArrayPtr vertices_;
    SgNormalArrayPtr normals_;
    SgIndexArray normalIndices_;
    SgColorArrayPtr colors_;
    SgIndexArray colorIndices_;
    SgTexCoordArrayPtr texCoords_;
    SgIndexArray texCoordIndices_;
    bool isSolid_;
};
typedef ref_ptr<SgMeshBase> SgMeshBasePtr;


class CNOID_EXPORT SgMesh : public SgMeshBase
{
public:
    SgMesh();
    SgMesh(const SgMesh& org, SgCloneMap* cloneMap = nullptr);

    virtual SgObject* doClone(SgCloneMap* cloneMap) const override;

    virtual void updateBoundingBox() override;

    /**
       Triangle indices (triangles variable) should be CCW.
    */
    const SgIndexArray& triangleVertices() const { return triangleVertices_; }
    SgIndexArray& triangleVertices() { return triangleVertices_; }

    int numTriangles() const { return static_cast<int>(triangleVertices_.size()) / 3; }
    void setNumTriangles(int n) { triangleVertices_.resize(n * 3); }
    void reserveNumTriangles(int n) { triangleVertices_.reserve(n * 3); }

    typedef Eigen::Map<Array3i> TriangleRef;
    TriangleRef triangle(int index){
        return TriangleRef(&triangleVertices_[index * 3]);
    }

    typedef Eigen::Map<const Array3i> ConstTriangleRef;
    ConstTriangleRef triangle(int index) const {
        return ConstTriangleRef(&triangleVertices_[index * 3]);
    }

    void setTriangle(int index, int v0, int v1, int v2){
        const int i = index * 3;
        triangleVertices_[i+0] = v0;
        triangleVertices_[i+1] = v1;
        triangleVertices_[i+2] = v2;
    }

    TriangleRef newTriangle(){
        const size_t s = triangleVertices_.size();
        triangleVertices_.resize(s + 3);
        return TriangleRef(&triangleVertices_[s]);
    }

    // deprecated
    TriangleRef addTriangle(){ return newTriangle(); }

    void addTriangle(int v0, int v1, int v2){
        triangleVertices_.push_back(v0);
        triangleVertices_.push_back(v1);
        triangleVertices_.push_back(v2);
    }
        
    enum PrimitiveType { MESH = 0, BOX, SPHERE, CYLINDER, CONE, CAPSULE };

    class Mesh { }; // defined for no primitive information

    class Box {
    public:
        Box() { }
        Box(Vector3 size) : size(size) { }
        Vector3 size;
    };
    class Sphere {
    public:
        Sphere() { }
        Sphere(double radius) : radius(radius) { }
        double radius;
    };
    class Cylinder {
    public:
        Cylinder() { }
        Cylinder(double radius, double height) :
            radius(radius), height(height), bottom(true), side(true), top(true) { }
        double radius;
        double height;
        bool bottom;
        bool side;
        bool top;
    };
    class Cone {
    public:
        Cone() { }
        Cone(double radius, double height) :
            radius(radius), height(height), bottom(true), side(true) { }
        double radius;
        double height;
        bool bottom;
        bool side;
    };
    class Capsule {
        public:
            Capsule() { }
            Capsule(double radius, double height) :
                radius(radius), height(height) { }
            double radius;
            double height;
        };

    typedef stdx::variant<Mesh, Box, Sphere, Cylinder, Cone, Capsule> Primitive;

    const int primitiveType() const { return stdx::get_variant_index(primitive_); }
    template<class TPrimitive> const TPrimitive& primitive() const { return stdx::get<TPrimitive>(primitive_); }
    template<class TPrimitive> void setPrimitive(const TPrimitive& prim) { primitive_ = prim; }

    void transform(const Affine3f& T);
    void translate(const Vector3f& translation);
    void rotate(const Matrix3f& R);

private:
    SgIndexArray triangleVertices_;
    Primitive primitive_;
};
typedef ref_ptr<SgMesh> SgMeshPtr;


class CNOID_EXPORT SgPolygonMesh : public SgMeshBase
{
public:
    SgPolygonMesh();
    SgPolygonMesh(const SgPolygonMesh& org, SgCloneMap* cloneMap = nullptr);

    virtual SgObject* doClone(SgCloneMap* cloneMap) const override;

    virtual void updateBoundingBox() override;
    
    /**
       The array of vertex indices corresponding to polygons.
       Indices are delimited by index value '-1'.
       When the actual instance type is SgPolygonMesh,
       the other index arrays defined in the SgMesh class also have to contain
       indices in the same way.
    */
    SgIndexArray& polygonVertices() { return polygonVertices_; }
    const SgIndexArray& polygonVertices() const { return polygonVertices_; }

private:
    SgIndexArray polygonVertices_;
};

typedef ref_ptr<SgPolygonMesh> SgPolygonMeshPtr;


class CNOID_EXPORT SgShape : public SgNode
{
public:
    SgShape();
    SgShape(const SgShape& org, SgCloneMap* cloneMap = nullptr);
    ~SgShape();
    
    virtual SgObject* doClone(SgCloneMap* cloneMap) const override;
    virtual int numChildObjects() const override;
    virtual SgObject* childObject(int index) override;
    virtual const BoundingBox& boundingBox() const override;
        
    SgMesh* mesh() { return mesh_; }
    const SgMesh* mesh() const { return mesh_; }
    SgMesh* setMesh(SgMesh* mesh);
    SgMesh* getOrCreateMesh();
        
    SgMaterial* material() { return material_; }
    const SgMaterial* material() const { return material_; }
    SgMaterial* setMaterial(SgMaterial* material);
    SgMaterial* getOrCreateMaterial();
        
    SgTexture* texture() { return texture_; }
    const SgTexture* texture() const { return texture_; }
    SgTexture* setTexture(SgTexture* texture);
    SgTexture* getOrCreateTexture();

protected:
    SgShape(int polymorhicId);

private:
    SgMeshPtr mesh_;
    SgMaterialPtr material_;
    SgTexturePtr texture_;
};
typedef ref_ptr<SgShape> SgShapePtr;


class CNOID_EXPORT SgPlot : public SgNode
{
protected:
    SgPlot(int polymorhicId);
    SgPlot(const SgPlot& org, SgCloneMap* cloneMap = nullptr);
    ~SgPlot();
    
public:

    virtual int numChildObjects() const override;
    virtual SgObject* childObject(int index) override;

    virtual const BoundingBox& boundingBox() const override;
    void updateBoundingBox();

    void clear();

    bool hasVertices() const { return (vertices_ && !vertices_->empty()); }
    SgVertexArray* vertices() { return vertices_; }
    const SgVertexArray* vertices() const { return vertices_; }
    SgVertexArray* setVertices(SgVertexArray* vertices);
    SgVertexArray* getOrCreateVertices(size_t size = 0);
        
    SgMaterial* material() { return material_; }
    const SgMaterial* material() const { return material_; }
    SgMaterial* setMaterial(SgMaterial* material);
    SgMaterial* getOrCreateMaterial();
        
    bool hasNormals() const { return (normals_ && !normals_->empty()); }
    SgNormalArray* normals() { return normals_; }
    const SgNormalArray* normals() const { return normals_; }
    SgNormalArray* setNormals(SgNormalArray* normals);
    SgVertexArray* getOrCreateNormals();
        
    const SgIndexArray& normalIndices() const { return normalIndices_; }
    SgIndexArray& normalIndices() { return normalIndices_; }

    bool hasColors() const { return (colors_ && !colors_->empty()); }
    SgColorArray* colors() { return colors_; }
    const SgColorArray* colors() const { return colors_; }
    SgColorArray* setColors(SgColorArray* colors);
    SgColorArray* getOrCreateColors();

    const SgIndexArray& colorIndices() const { return colorIndices_; }
    SgIndexArray& colorIndices() { return colorIndices_; }

private:
    BoundingBox bbox;
    SgVertexArrayPtr vertices_;
    SgNormalArrayPtr normals_;
    SgIndexArray normalIndices_;
    SgColorArrayPtr colors_;
    SgIndexArray colorIndices_;
    SgMaterialPtr material_;
};
typedef ref_ptr<SgPlot> SgPlotPtr;


class CNOID_EXPORT SgPointSet : public SgPlot
{
public:
    SgPointSet();
    SgPointSet(const SgPointSet& org, SgCloneMap* cloneMap = nullptr);
    virtual SgObject* doClone(SgCloneMap* cloneMap) const override;

    void setPointSize(double size) { pointSize_ = size; }

    /**
       The default value of this is zero and the current system value is used then.
    */
    double pointSize() const { return pointSize_; }

protected:
    SgPointSet(int polymorhicId);

private:
    double pointSize_;
};
typedef ref_ptr<SgPointSet> SgPointSetPtr;


class CNOID_EXPORT SgLineSet : public SgPlot
{
public:
    SgLineSet();
    SgLineSet(const SgLineSet& org, SgCloneMap* cloneMap = nullptr);
    virtual SgObject* doClone(SgCloneMap* cloneMap) const override;

    const SgIndexArray& lineVertices() const { return lineVertices_; }
    SgIndexArray& lineVertices() { return lineVertices_; }

    int numLines() const { return static_cast<int>(lineVertices_.size()) / 2; }
    void setNumLines(int n) { lineVertices_.resize(n * 2); }
    void reserveNumLines(int n) { lineVertices_.reserve(n * 2); }
    void clearLines() { lineVertices_.clear(); }

    typedef Eigen::Map<Array2i> LineRef;
    LineRef line(int index){
        return LineRef(&lineVertices_[index * 2]);
    }

    typedef Eigen::Map<const Array2i> ConstLineRef;
    ConstLineRef line(int index) const {
        return ConstLineRef(&lineVertices_[index * 2]);
    }

    void setLine(int index, int v0, int v1){
        const int i = index * 2;
        lineVertices_[i+0] = v0;
        lineVertices_[i+1] = v1;
    }

    LineRef addLine(){
        const size_t s = lineVertices_.size();
        lineVertices_.resize(s + 2);
        return LineRef(&lineVertices_[s]);
    }

    void addLine(int v0, int v1){
        lineVertices_.push_back(v0);
        lineVertices_.push_back(v1);
    }

    void setLineWidth(float width) { lineWidth_ = width; }

    /**
       The default value of this is zero and the current system value is used then.
    */
    float lineWidth() const { return lineWidth_; }

protected:
    SgLineSet(int polymorhicId);

private:
    SgIndexArray lineVertices_;
    float lineWidth_;
};
typedef ref_ptr<SgLineSet> SgLineSetPtr;


class CNOID_EXPORT SgOverlay : public SgGroup
{
public:
    SgOverlay();
    SgOverlay(const SgOverlay& org, SgCloneMap* cloneMap = nullptr);
    ~SgOverlay();

    virtual SgObject* doClone(SgCloneMap* cloneMap) const override;

    struct ViewVolume {
        double left;
        double right;
        double bottom;
        double top;
        double zNear;
        double zFar;
    };

    virtual void calcViewVolume(double viewportWidth, double viewportHeight, ViewVolume& io_volume);

protected:
    SgOverlay(int polymorhicId);
};

}

#endif
