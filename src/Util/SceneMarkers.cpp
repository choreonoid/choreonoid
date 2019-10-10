/**
   @author Shizuko Hattori
   @author Shin'ichiro Nakaoka
*/

#include "SceneMarkers.h"
#include "MeshGenerator.h"
#include "EigenUtil.h"

using namespace std;
using namespace cnoid;

SceneMarker::SceneMarker()
{
    markerType_ = NO_MARKER;
    markerSize_ = 0.01;
    emission_ = 0.5;

    material = new SgMaterial;
    Vector3f c(1.0f, 0.0f, 0.0f);
    material->setDiffuseColor(c);
    material->setEmissiveColor(emission_ * c);
    material->setTransparency(0.0f);
    material->setAmbientIntensity(0.0f);
}


void SceneMarker::updateMarker(bool doNotify)
{
    clearChildren();

    switch(markerType_){
    case NO_MARKER:
        break;
    case CROSS_MARKER:
        setCross();
        break;
    case SPHERE_MARKER:
        setSphere();
        break;
    case AXES_MARKER:
        setAxes();
        break;
    default:
        break;
    }

    if(doNotify){
        notifyUpdate(SgUpdate::ADDED | SgUpdate::REMOVED);
    }
}


void SceneMarker::setCross()
{
    const float p = markerSize_ / 0.2f;
    auto vertices = new SgVertexArray {
        {   -p, 0.0f, 0.0f },
        {    p, 0.0f, 0.0f },
        { 0.0f,   -p, 0.0f },
        { 0.0f,    p, 0.0f },
        { 0.0f, 0.0f,   -p },
        { 0.0f, 0.0f,    p }
    };
    
    SgLineSet* lineSet = new SgLineSet;
    lineSet->setVertices(vertices);
    lineSet->setNumLines(3);
    lineSet->setLine(0, 0, 1);
    lineSet->setLine(1, 2, 3);
    lineSet->setLine(2, 4, 5);

    lineSet->setMaterial(material);

    addChild(lineSet);
}


void SceneMarker::setSphere()
{
    SgShape* sphere = new SgShape;
    MeshGenerator mg;
    sphere->setMesh(mg.generateSphere(markerSize_ / 2.0f));
    sphere->setMaterial(material);

    addChild(sphere);
}


void SceneMarker::setAxes()
{
    double r1 = markerSize_ * 0.1;
    double h1 = markerSize_ * 0.7;
    double r2 = markerSize_ * 0.2;
    double h2 = markerSize_ * 0.3;
    
    MeshGenerator mg;
    SgMeshPtr mesh = mg.generateArrow(r1, h1, r2, h2);
    mesh->translate(Vector3f(0.0f, h1 / 2.0, 0.0f));

    for(int i=0; i < 3; ++i){
        SgShape* shape = new SgShape;
        shape->setMesh(mesh);

        SgMaterial* material = new SgMaterial;
        Vector3f color(0.2f, 0.2f, 0.2f);
        color[i] = 1.0f;
        material->setDiffuseColor(Vector3f::Zero());
        material->setEmissiveColor(color);
        material->setAmbientIntensity(0.0f);
        shape->setMaterial(material);
            
        SgPosTransform* arrow = new SgPosTransform;
        arrow->addChild(shape);
        if(i == 0){
            arrow->setRotation(AngleAxis(-PI / 2.0, Vector3::UnitZ()));
        } else if(i == 2){
            arrow->setRotation(AngleAxis( PI / 2.0, Vector3::UnitX()));
        }

        addChild(arrow);
    }
}


const Vector3f& SceneMarker::color() const
{
    return material->emissiveColor();
}


void SceneMarker::setColor(const Vector3f& color)
{
    material->setDiffuseColor(color);
    material->setEmissiveColor(emission_ * color);
}


float SceneMarker::emission() const
{
    return emission_;
}


void SceneMarker::setEmission(float r)
{
    emission_ = r;
    material->setEmissiveColor(r * material->diffuseColor());
}


double SceneMarker::transparency() const
{
    return material->transparency();
}


void SceneMarker::setTransparency(float t)
{
    material->setTransparency(t);
}


CrossMarker::CrossMarker(double size, const Vector3f& color, double lineWidth)
{
    size_ = 0.0;
    vertices = new SgVertexArray;
    setSize(size);

    SgLineSet* lineSet = new SgLineSet;
    lineSet->setVertices(vertices);

    lineSet->setNumLines(3);
    lineSet->setLine(0, 0, 1);
    lineSet->setLine(1, 2, 3);
    lineSet->setLine(2, 4, 5);

    lineSet->setColors(new SgColorArray)->push_back(color);
    SgIndexArray& iColors = lineSet->colorIndices();
    iColors.resize(6, 0);

    lineSet->setLineWidth(lineWidth);
    
    addChild(lineSet);
}


void CrossMarker::setSize(double size)
{
    if(size != size_){
        const float s = size;

        *vertices = {
            { -s,     0.0f,  0.0f },
            {  s,     0.0f,  0.0f },
            {  0.0f, -s,     0.0f },
            {  0.0f,  s,     0.0f },
            {  0.0f,  0.0f, -s    },
            {  0.0f,  0.0f,  s    }
        };
            
        size_ = size;
        vertices->notifyUpdate();
    }
}


SphereMarker::SphereMarker()
{
    initialize(1.0, Vector3f(0.8, 0.8, 0.8), 0.0);
}

SphereMarker::SphereMarker(double radius, const Vector3f& color, float transparency)
{
    initialize(radius, color, transparency);
}


void SphereMarker::initialize(double radius, const Vector3f& color, float transparency)
{
    SgShapePtr shape = new SgShape;
    MeshGenerator meshGenerator;
    shape->setMesh(meshGenerator.generateSphere(1.0));
    material = shape->setMaterial(new SgMaterial);
    material->setDiffuseColor(color);
    material->setEmissiveColor(color);
    material->setTransparency(transparency);
    scale = new SgScaleTransform;
    scale->addChild(shape);
    scale->setScale(radius);
    addChild(scale);
}


void SphereMarker::setRadius(double r)
{
    scale->setScale(r);
    scale->notifyUpdate();
}


void SphereMarker::setColor(const Vector3f& c)
{
    material->setDiffuseColor(c);
    material->setEmissiveColor(c);
    material->notifyUpdate();
}


BoundingBoxMarker::BoundingBoxMarker(const BoundingBox& bbox, const Vector3f& color, float transparency, double width)
{
    create(bbox, color, transparency, width);
}


BoundingBoxMarker::BoundingBoxMarker(const BoundingBox& bbox, const Vector3f& color, float transparency)
{
    double h = bbox.max().x() - bbox.min().x();
    double w = bbox.max().y() - bbox.min().y();
    double d = bbox.max().z() - bbox.min().z();
    double width = (h + w + d) / 3.0 / 10.0;
    create(bbox, color, transparency, width);
}


void BoundingBoxMarker::create(const BoundingBox& bbox, const Vector3f& color, float transparency, double width)
{
    SgShape* shape = new SgShape;
    MeshGenerator meshGenerator;
    shape->setMesh(meshGenerator.generateBox(Vector3(width, width, width)));
    
    SgMaterial* material = shape->setMaterial(new SgMaterial);
    material->setTransparency(transparency);
    material->setDiffuseColor(color);
    material->setEmissiveColor(color);

    const double& x0 = bbox.min().x();
    const double& x1 = bbox.max().x();
    const double& y0 = bbox.min().y();
    const double& y1 = bbox.max().y();
    const double& z0 = bbox.min().z();
    const double& z1 = bbox.max().z();
    
    addMarker(shape, x0, y0, z0);
    addMarker(shape, x0, y0, z1);
    addMarker(shape, x0, y1, z0);
    addMarker(shape, x0, y1, z1);

    addMarker(shape, x1, y0, z0);
    addMarker(shape, x1, y0, z1);
    addMarker(shape, x1, y1, z0);
    addMarker(shape, x1, y1, z1);
}


void BoundingBoxMarker::addMarker(SgShape* shape, double x, double y, double z)
{
    SgPosTransformPtr transform = new SgPosTransform;
    transform->setTranslation(Vector3(x, y, z));
    transform->addChild(shape);
    addChild(transform);
}
