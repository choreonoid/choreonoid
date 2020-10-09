#include "StdSceneWriter.h"
#include <cnoid/SceneGraph>
#include <cnoid/SceneDrawables>
#include <cnoid/PolymorphicSceneNodeFunctionSet>
#include <cnoid/EigenArchive>
#include <cnoid/FilePathVariableProcessor>
#include <cnoid/FileUtil>
#include <cnoid/stdx/filesystem>
#include "gettext.h"

using namespace std;
using namespace cnoid;
namespace filesystem = cnoid::stdx::filesystem;

namespace cnoid {

class StdSceneWriter::Impl
{
public:
    StdSceneWriter* self;
    PolymorphicSceneNodeFunctionSet writeFunctions;
    bool isDegreeMode;
    MappingPtr currentArchive;
    SgMaterialPtr defaultMaterial;
    FilePathVariableProcessorPtr pathVariableProcessor;

    Impl(StdSceneWriter* self);
    MappingPtr writeSceneNode(SgNode* node);
    void writeType(Mapping& archive, const char* typeName);
    void writeObjectHeader(Mapping& archive, SgObject* object);
    void writePosTransform(Mapping& archive, SgPosTransform* transform);
    void writeScaleTransform(Mapping& archive, SgScaleTransform* transform);
    void writeShape(Mapping& archive, SgShape* shape);
    MappingPtr writeGeometry(SgMesh* mesh);
    void writeBox(Mapping& archive, const SgMesh::Box& box);
    void writeSphere(Mapping& archive, const SgMesh::Sphere& sphere);
    void writeCylinder(Mapping& archive, const SgMesh::Cylinder& cylinder);
    void writeCone(Mapping& archive, const SgMesh::Cone& cone);
    void writeCapsule(Mapping& archive, const SgMesh::Capsule& capsule);
    MappingPtr writeAppearance(SgShape* shape);
    MappingPtr writeMaterial(SgMaterial* material);
};

}

StdSceneWriter::StdSceneWriter()
{
    impl = new Impl(this);
}


StdSceneWriter::Impl::Impl(StdSceneWriter* self)
    : self(self)
{
    writeFunctions.setFunction<SgPosTransform>(
        [&](SgPosTransform* transform){ writePosTransform(*currentArchive, transform); });
    writeFunctions.setFunction<SgScaleTransform>(
        [&](SgScaleTransform* transform){ writeScaleTransform(*currentArchive, transform); });
    writeFunctions.setFunction<SgShape>(
        [&](SgShape* shape){ writeShape(*currentArchive, shape); });

    writeFunctions.updateDispatchTable();

    isDegreeMode = true;
}


StdSceneWriter::~StdSceneWriter()
{
    delete impl;
}


void StdSceneWriter::setBaseDirectory(const std::string& directory)
{
    impl->pathVariableProcessor = new FilePathVariableProcessor;
    impl->pathVariableProcessor->setBaseDirectory(directory);
}


void StdSceneWriter::setFilePathVariableProcessor(FilePathVariableProcessor* processor)
{
    impl->pathVariableProcessor = processor;
}


MappingPtr StdSceneWriter::writeScene(SgNode* node)
{
    return impl->writeSceneNode(node);
}


MappingPtr StdSceneWriter::Impl::writeSceneNode(SgNode* node)
{
    if(!pathVariableProcessor){
        pathVariableProcessor = new FilePathVariableProcessor;
    }
    
    MappingPtr archive = new Mapping;

    writeObjectHeader(*archive, node);

    currentArchive = archive;
    writeFunctions.dispatch(node);
    currentArchive.reset();

    if(node->isGroup()){
        auto group = node->toGroup();
        ListingPtr elements = new Listing;
        for(auto& child : *group){
            if(auto childArchive = writeSceneNode(child)){
                elements->append(childArchive);
            }
        }
        if(!elements->empty()){
            archive->insert("elements", elements);
        }
    }
    
    return archive;
}


void StdSceneWriter::Impl::writeType(Mapping& archive, const char* typeName)
{
    auto typeNode = new ScalarNode(typeName);
    typeNode->setAsHeaderInMapping();
    archive.insert("type", typeNode);
}


void StdSceneWriter::Impl::writeObjectHeader(Mapping& archive, SgObject* object)
{
    if(!object->name().empty()){
        archive.write("name", object->name());
    }
}
    
    
void StdSceneWriter::Impl::writePosTransform(Mapping& archive, SgPosTransform* transform)
{
    writeType(archive, "Transform");
    AngleAxis aa(transform->rotation());
    if(aa.angle() != 0.0){
        writeDegreeAngleAxis(archive, "rotation", aa);
    }
    Vector3 p(transform->translation());
    if(!p.isZero()){
        write(archive, "translation", p);
    }
}


void StdSceneWriter::Impl::writeScaleTransform(Mapping& archive, SgScaleTransform* transform)
{
    writeType(archive, "Transform");
    write(archive, "scale", transform->scale());
}


void StdSceneWriter::Impl::writeShape(Mapping& archive, SgShape* shape)
{
    writeType(archive, "Shape");

    if(auto geometry = writeGeometry(shape->mesh())){
        archive.insert("geometry", geometry);
    }
    if(auto appearance = writeAppearance(shape)){
        archive.insert("appearance", appearance);
    }
}


MappingPtr StdSceneWriter::Impl::writeGeometry(SgMesh* mesh)
{
    if(!mesh){
        return nullptr;
    }

    MappingPtr archive = new Mapping;

    if(!mesh->uri().empty()){
        archive->write("type", "Resource");
        archive->write("uri", pathVariableProcessor->parameterize(mesh->uri()), DOUBLE_QUOTED);

    } else {
        switch(mesh->primitiveType()){
        case SgMesh::BoxType:
            writeBox(*archive, mesh->primitive<SgMesh::Box>());
            break;
        case SgMesh::SphereType:
            writeSphere(*archive, mesh->primitive<SgMesh::Sphere>());
            break;
        case SgMesh::CylinderType:
            writeCylinder(*archive, mesh->primitive<SgMesh::Cylinder>());
            break;
        case SgMesh::ConeType:
            writeCone(*archive, mesh->primitive<SgMesh::Cone>());
            break;
        case SgMesh::CapsuleType:
            writeCapsule(*archive, mesh->primitive<SgMesh::Capsule>());
            break;
        default:
            archive = nullptr;
            break;
        }
    }

    if(mesh->creaseAngle() > 0.0f){
        archive->write("creaseAngle", mesh->creaseAngle());
    }

    return archive;
}


void StdSceneWriter::Impl::writeBox(Mapping& archive, const SgMesh::Box& box)
{
    archive.write("type", "Box");
    write(archive, "size", box.size);
}


void StdSceneWriter::Impl::writeSphere(Mapping& archive, const SgMesh::Sphere& sphere)
{
    archive.write("type", "Sphere");
    archive.write("radius", sphere.radius);
}


void StdSceneWriter::Impl::writeCylinder(Mapping& archive, const SgMesh::Cylinder& cylinder)
{
    archive.write("type", "Cylinder");
    archive.write("radius", cylinder.radius);
    archive.write("height", cylinder.height);
}


void StdSceneWriter::Impl::writeCone(Mapping& archive, const SgMesh::Cone& cone)
{
    archive.write("type", "Cone");
    archive.write("radius", cone.radius);
    archive.write("height", cone.height);
}


void StdSceneWriter::Impl::writeCapsule(Mapping& archive, const SgMesh::Capsule& capsule)
{
    archive.write("type", "Cone");
    archive.write("radius", capsule.radius);
    archive.write("height", capsule.height);
}


MappingPtr StdSceneWriter::Impl::writeAppearance(SgShape* shape)
{
    MappingPtr archive = new Mapping;

    if(auto material = writeMaterial(shape->material())){
        archive->insert("material", material);
    }
    if(archive->empty()){
        archive = nullptr;
    }

    return archive;
}


MappingPtr StdSceneWriter::Impl::writeMaterial(SgMaterial* material)
{
    if(!material){
        return nullptr;
    }
    if(!defaultMaterial){
        defaultMaterial = new SgMaterial;
    }
    MappingPtr archive = new Mapping;
    archive->setDoubleFormat("%.2f");

    if(material->diffuseColor() != defaultMaterial->diffuseColor()){
        write(*archive, "diffuseColor", material->diffuseColor());
    }
    if(material->emissiveColor() != defaultMaterial->emissiveColor()){
        write(*archive, "emissiveColor", material->emissiveColor());
    }
    if(material->specularColor() != defaultMaterial->specularColor()){
        write(*archive, "specularColor", material->specularColor());
    }
    if(material->shininess() != defaultMaterial->shininess()){
        archive->write("shininess", material->shininess());
    }
    if(material->transparency() != defaultMaterial->transparency()){
        archive->write("transparency", material->transparency());
    }

    if(archive->empty()){
        archive = nullptr;
    }
    
    return archive;
}

        
    
    

    
    
