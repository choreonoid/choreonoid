/**
   @author Shizuko Hattori
   @author Shin'ichiro Nakaoka
*/

#include "SceneMarker.h"
#include "MeshGenerator.h"

using namespace std;
using namespace cnoid;


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
        vertices->resize(6);
        vertices->at(0) << - s, 0.0f, 0.0f;
        vertices->at(1) <<   s, 0.0f, 0.0f;
        vertices->at(2) << 0.0f,  -s, 0.0f;
        vertices->at(3) << 0.0f,   s, 0.0f;
        vertices->at(4) << 0.0f,0.0f,   -s;
        vertices->at(5) << 0.0f,0.0f,    s;
        size_ = size;
        vertices->notifyUpdate();
    }
};


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
