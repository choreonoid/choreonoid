#include "StdSceneWriter.h"
#include "YAMLWriter.h"
#include "ObjSceneWriter.h"
#include "SceneGraph.h"
#include "SceneDrawables.h"
#include "SceneLights.h"
#include "SceneCameras.h"
#include "SceneEffects.h"
#include "SceneGraphOptimizer.h"
#include "PolymorphicSceneNodeFunctionSet.h"
#include "EigenArchive.h"
#include "FilePathVariableProcessor.h"
#include "CloneMap.h"
#include "NullOut.h"
#include <cnoid/stdx/filesystem>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;
namespace filesystem = cnoid::stdx::filesystem;

namespace cnoid {

class StdSceneWriter::Impl
{
public:
    StdSceneWriter* self;
    PolymorphicSceneNodeFunctionSet writeFunctions;
    MappingPtr currentArchive;
    bool isDegreeMode;
    bool isTransformIntegrationEnabled;
    bool isAppearanceEnabled;
    bool isReplacingExistingModelFile;
    int extModelFileMode;
    SgMaterialPtr defaultMaterial;
    FilePathVariableProcessorPtr pathVariableProcessor;
    unique_ptr<YAMLWriter> yamlWriter;
    unique_ptr<StdSceneWriter> subSceneWriter;
    unique_ptr<ObjSceneWriter> objSceneWriter;
    int numSkippedNode;
    filesystem::path baseDirPath;
    vector<filesystem::path> uriDirectoryStack;
    set<string> extModelFiles;

    ostream* os_;
    ostream& os() { return *os_; }

    Impl(StdSceneWriter* self);
    void copyConfigurations(const Impl* org);
    YAMLWriter* getOrCreateYamlWriter();
    FilePathVariableProcessor* getOrCreatePathVariableProcessor();
    StdSceneWriter* getOrCreateSubSceneWriter();
    ObjSceneWriter* getOrCreateObjSceneWriter();
    void setBaseDirectory(const std::string& directory);
    void pushToUriDirectoryStack(const std::string& uri);
    void popFromUriDirectoryStack();
    bool writeScene(const std::string& filename, SgNode* node, const std::vector<SgNode*>* pnodes);
    MappingPtr writeSceneNode(SgNode* node);
    void makeLinkToOriginalModelFile(Mapping* archive, SgObject* sceneObject);
    bool replaceOriginalModelFile(
        Mapping* archive, SgNode* node, bool isAppearanceEnabled, SgObject* objectOfUri);
    void processUnknownNode(Mapping* archive, SgNode* node);
    void writeObjectHeader(Mapping* archive, const char* typeName, SgObject* object);
    void writeGroup(Mapping* archive, SgGroup* group);
    void writePosTransform(Mapping* archive, SgPosTransform* transform);
    void writeScaleTransform(Mapping* archive, SgScaleTransform* transform);
    void writeShape(Mapping* archive, SgShape* shape);
    MappingPtr writeGeometry(SgShape* shape);
    void writeMeshAttributes(Mapping* archive, SgMesh* mesh);
    bool writeMesh(Mapping* archive, SgMesh* mesh);
    void writePrimitiveAttributes(Mapping* archive, SgMesh* mesh);
    void writeBox(Mapping* archive, SgMesh* mesh);
    void writeSphere(Mapping* archive, SgMesh* mesh);
    void writeCylinder(Mapping* archive, SgMesh* mesh);
    void writeCone(Mapping* archive, SgMesh* mesh);
    void writeCapsule(Mapping* archive, SgMesh* mesh);
    MappingPtr writeAppearance(SgShape* shape);
    MappingPtr writeMaterial(SgMaterial* material);
    MappingPtr writeTexture(SgTexture* texture);
    MappingPtr writeTextureTransform(SgTextureTransform * transform);
    void writeLight(Mapping* archive, SgLight* light);
    void writeDirectionalLight(Mapping* archive, SgDirectionalLight* light);
    void writePointLight(Mapping* archive, SgPointLight* light);
    void writeSpotLight(Mapping* archive, SgSpotLight* light);
    void writeCamera(Mapping* archive, SgCamera* camera);
    void writePerspectiveCamera(Mapping* archive, SgPerspectiveCamera* camera);
    void writeOrthographicCamera(Mapping* archive, SgOrthographicCamera* camera);
    void writeFog(Mapping* archive, SgFog* fog);
};

}

StdSceneWriter::StdSceneWriter()
{
    impl = new Impl(this);
}


StdSceneWriter::Impl::Impl(StdSceneWriter* self)
    : self(self)
{
    writeFunctions.setFunction<SgNode>(
        [&](SgNode* node){ processUnknownNode(currentArchive, node); });
    writeFunctions.setFunction<SgGroup>(
        [&](SgGroup* group){ writeGroup(currentArchive, group); });
    writeFunctions.setFunction<SgPosTransform>(
        [&](SgPosTransform* transform){ writePosTransform(currentArchive, transform); });
    writeFunctions.setFunction<SgScaleTransform>(
        [&](SgScaleTransform* transform){ writeScaleTransform(currentArchive, transform); });
    writeFunctions.setFunction<SgShape>(
        [&](SgShape* shape){ writeShape(currentArchive, shape); });
    writeFunctions.setFunction<SgDirectionalLight>(
        [&](SgDirectionalLight* light){ writeDirectionalLight(currentArchive, light); });
    writeFunctions.setFunction<SgPointLight>(
        [&](SgPointLight* light){ writePointLight(currentArchive, light); });
    writeFunctions.setFunction<SgSpotLight>(
        [&](SgSpotLight* light){ writeSpotLight(currentArchive, light); });
    writeFunctions.setFunction<SgPerspectiveCamera>(
        [&](SgPerspectiveCamera* camera){ writePerspectiveCamera(currentArchive, camera); });
    writeFunctions.setFunction<SgOrthographicCamera>(
        [&](SgOrthographicCamera* camera){ writeOrthographicCamera(currentArchive, camera); });
    writeFunctions.setFunction<SgFog>(
        [&](SgFog* fog){ writeFog(currentArchive, fog); });

    writeFunctions.updateDispatchTable();

    isDegreeMode = true;
    isTransformIntegrationEnabled = false;
    isAppearanceEnabled = true;
    isReplacingExistingModelFile = false;
    extModelFileMode = EmbedModels;

    os_ = &nullout();    
}


StdSceneWriter::StdSceneWriter(const StdSceneWriter& org)
    : StdSceneWriter()
{
    impl->copyConfigurations(org.impl);
}


void StdSceneWriter::Impl::copyConfigurations(const Impl* org)
{
    isDegreeMode = org->isDegreeMode;
    isTransformIntegrationEnabled = org->isTransformIntegrationEnabled;
    isAppearanceEnabled = org->isAppearanceEnabled;
    extModelFileMode = org->extModelFileMode;
    os_ = org->os_;
    if(org->yamlWriter){
        getOrCreateYamlWriter()->setIndentWidth(org->yamlWriter->indentWidth());
    }
    
    if(org->pathVariableProcessor){
        getOrCreatePathVariableProcessor()->setBaseDirectory(
            org->pathVariableProcessor->baseDirectory());
    }
}


StdSceneWriter::~StdSceneWriter()
{
    delete impl;
}


void StdSceneWriter::setMessageSink(std::ostream& os)
{
    AbstractSceneWriter::setMessageSink(os);
    
    impl->os_ = &os;
    if(impl->objSceneWriter){
        impl->objSceneWriter->setMessageSink(os);
    }
}


YAMLWriter* StdSceneWriter::Impl::getOrCreateYamlWriter()
{
    if(!yamlWriter){
        yamlWriter.reset(new YAMLWriter);
        yamlWriter->setKeyOrderPreservationMode(true);
    }
    return yamlWriter.get();
}


FilePathVariableProcessor* StdSceneWriter::Impl::getOrCreatePathVariableProcessor()
{
    if(!pathVariableProcessor){
        pathVariableProcessor = new FilePathVariableProcessor;
    }
    return pathVariableProcessor;
}


StdSceneWriter* StdSceneWriter::Impl::getOrCreateSubSceneWriter()
{
    if(!subSceneWriter){
        subSceneWriter.reset(new StdSceneWriter(*self));
        subSceneWriter->setExtModelFileMode(EmbedModels);
        subSceneWriter->impl->isReplacingExistingModelFile = true;
    }
    return subSceneWriter.get();
}


ObjSceneWriter* StdSceneWriter::Impl::getOrCreateObjSceneWriter()
{
    if(!objSceneWriter){
        objSceneWriter.reset(new ObjSceneWriter);
        objSceneWriter->setMessageSink(os());
    }
    return objSceneWriter.get();
}


void StdSceneWriter::setBaseDirectory(const std::string& directory)
{
    impl->setBaseDirectory(directory);
}


void StdSceneWriter::Impl::setBaseDirectory(const std::string& directory)
{
    auto pvp = getOrCreatePathVariableProcessor();
    pvp->setBaseDirectory(directory);
    baseDirPath = pvp->baseDirPath();
}


void StdSceneWriter::setFilePathVariableProcessor(FilePathVariableProcessor* processor)
{
    impl->pathVariableProcessor = processor;
}


void StdSceneWriter::setIndentWidth(int n)
{
    impl->getOrCreateYamlWriter()->setIndentWidth(n);
}


void StdSceneWriter::setExtModelFileMode(int mode)
{
    impl->extModelFileMode = mode;
}


int StdSceneWriter::extModelFileMode() const
{
    return impl->extModelFileMode;
}


void StdSceneWriter::setTransformIntegrationEnabled(bool on)
{
    impl->isTransformIntegrationEnabled = on;
}


bool StdSceneWriter::isTransformIntegrationEnabled() const
{
    return impl->isTransformIntegrationEnabled;
}


void StdSceneWriter::setAppearanceEnabled(bool on)
{
    impl->isAppearanceEnabled = on;
}


bool StdSceneWriter::isAppearanceEnabled() const
{
    return impl->isAppearanceEnabled;
}


void StdSceneWriter::Impl::pushToUriDirectoryStack(const std::string& uri)
{
    if(!isReplacingExistingModelFile){
        filesystem::path dirPath = filesystem::path(uri).parent_path();
        if(!uriDirectoryStack.empty()){
            if(dirPath.is_relative()){
                dirPath = uriDirectoryStack.back() / dirPath;
            }
        }
        uriDirectoryStack.push_back(dirPath);
    }
}


void StdSceneWriter::Impl::popFromUriDirectoryStack()
{
    if(!isReplacingExistingModelFile){
        if(!uriDirectoryStack.empty()){
            uriDirectoryStack.pop_back();
        }
    }
}


MappingPtr StdSceneWriter::writeScene(SgNode* node)
{
    return impl->writeSceneNode(node);
}


bool StdSceneWriter::writeScene(const std::string& filename, SgNode* node)
{
    return impl->writeScene(filename, node, nullptr);
}


bool StdSceneWriter::writeScene(const std::string& filename, const std::vector<SgNode*>& nodes)
{
    return impl->writeScene(filename, nullptr, &nodes);
}


bool StdSceneWriter::Impl::writeScene
(const std::string& filename, SgNode* node, const std::vector<SgNode*>* pnodes)
{
    getOrCreateYamlWriter();

    if(!yamlWriter->openFile(filename)){
        return false;
    }

    setBaseDirectory(filesystem::path(filename).parent_path().string());

    SgGroupPtr group = new SgGroup;
    if(node){
        group->addChild(node);
    } else if(pnodes){
        for(auto& node : *pnodes){
            group->addChild(node);
        }
    }

    if(isTransformIntegrationEnabled && (extModelFileMode != LinkToOriginalModelFiles)){
        SceneGraphOptimizer optimizer;
        CloneMap cloneMap;
        SgObject::setNonNodeCloning(cloneMap, false);
        group = cloneMap.getClone(group);
        SgObject::setNonNodeCloning(cloneMap, true);
        optimizer.simplifyTransformPathsWithTransformedMeshes(group, cloneMap);
    }
       
    MappingPtr header = new Mapping;
    header->write("format", "choreonoid_scene");
    header->write("format_version", "1.0");
    header->write("angle_unit", "degree");
    
    auto directory = filesystem::path(filename).parent_path().generic_string();
    setBaseDirectory(directory);
    uriDirectoryStack.clear();

    numSkippedNode = 0;

    ListingPtr nodeList = new Listing;
    for(auto& node : *group){
        nodeList->append(writeSceneNode(node));
    }
    header->insert("scene", nodeList);

    if(numSkippedNode == 1){
        os() << _("Warning: There is an unsupported node.") << endl;
    } else if(numSkippedNode >= 2){
        os() << format(_("Warning: {0} unsupported nodes were skipped to output."), numSkippedNode) << endl;
    }
    
    yamlWriter->putNode(header);
    yamlWriter->closeFile();

    extModelFiles.clear();
    
    return true;
}


MappingPtr StdSceneWriter::Impl::writeSceneNode(SgNode* node)
{
    MappingPtr archive = new Mapping;

    if(node->hasUri()){
        if(extModelFileMode != EmbedModels){
            writeObjectHeader(archive, "Resource", node);
            if(extModelFileMode == LinkToOriginalModelFiles){
                makeLinkToOriginalModelFile(archive, node);
            } else {
                if(!replaceOriginalModelFile(archive, node, true, node)){
                    // TODO: write the models as embeded models when the replacement fails
                    archive.reset();
                }
            }
            return archive;
        } else {
            pushToUriDirectoryStack(node->uri());
        }
    }

    currentArchive = archive;
    writeFunctions.dispatch(node);
    currentArchive.reset();

    if(node->isGroupNode()){
        auto group = node->toGroupNode();
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

    if(archive->empty()){
        archive.reset();
    }

    if(node->hasUri()){
        popFromUriDirectoryStack();
    }
    
    return archive;
}


void StdSceneWriter::Impl::makeLinkToOriginalModelFile(Mapping* archive, SgObject* sceneObject)
{
    string uri = sceneObject->absoluteUri();
    if(uri.find_first_of("file://") == 0){
        uri = getOrCreatePathVariableProcessor()->parameterize(uri.substr(7));
    }
    archive->write("uri", uri, DOUBLE_QUOTED);
}


/*
  TODO: Check if the scene can be written as the target model file formatd.
  If it cannot, give up the writing.
*/
bool StdSceneWriter::Impl::replaceOriginalModelFile
(Mapping* archive, SgNode* node, bool isAppearanceEnabled, SgObject* objectOfUri)
{
    // TODO: Consider the protocol header of the URI
    filesystem::path path(objectOfUri->uri());
    if(path.is_absolute()){
        path = filesystem::path("resource") / path.stem();
    } else {
        path.replace_extension("");
    }
    if(objectOfUri->hasUriFragment()){
        path += "-";
        path += objectOfUri->uriFragment();
    }
    if(extModelFileMode == ReplaceWithObjModelFiles){
        path += ".obj";
    } else {
        path += ".scen";
    }
    auto fullPath = filesystem::lexically_normal(baseDirPath / path);
    auto filename = fullPath.string();

    bool replaced = false;
    stdx::error_code ec;
    
    auto emplaced = extModelFiles.emplace(filename);
    if(!emplaced.second){ // Already replaced
        replaced = true;

    } else { // New reference
        if(objectOfUri->hasAbsoluteUri()){
            auto& absUri = objectOfUri->absoluteUri();
            if(absUri.find_first_of("file://") == 0){
                filesystem::path orgFilePath(absUri.substr(7));
                if(filesystem::equivalent(fullPath, orgFilePath, ec)){
                    os() << format(_("Model file \"{0}\" cannot be replaced with the same format file in the same directory"),
                                   filename) << endl;
                    return false;
                }
            }
        }
        filesystem::create_directories(fullPath.parent_path(), ec);
        if(!ec){
            if(extModelFileMode == ReplaceWithObjModelFiles){
                auto writer = getOrCreateObjSceneWriter();
                writer->setMaterialEnabled(isAppearanceEnabled);
                replaced = writer->writeScene(filename, node);
            } else {
                auto writer = getOrCreateSubSceneWriter();
                writer->setAppearanceEnabled(isAppearanceEnabled);
                replaced = writer->writeScene(filename, node);
            }
        }
    }

    if(replaced){
        archive->write("uri", path.generic_string(), DOUBLE_QUOTED);
        
    } else {
        if(ec){
            os() << format(_("Warning: Failed to replace model file \"{0}\" with \"{1}\". {2}"),
                           objectOfUri->uri(), filename, ec.message()) << endl;
        } else {
            os() << format(_("Warning: Failed to replace model file \"{0}\" with \"{1}\"."),
                           objectOfUri->uri(), filename) << endl;
        }
    }
    
    return replaced;
}


void StdSceneWriter::Impl::processUnknownNode(Mapping* archive, SgNode* node)
{
    ++numSkippedNode;
}
    
    
void StdSceneWriter::Impl::writeObjectHeader(Mapping* archive, const char* typeName, SgObject* object)
{
    if(typeName){
        auto typeNode = new ScalarNode(typeName);
        typeNode->setAsHeaderInMapping();
        archive->insert("type", typeNode);
    }
    if(!object->name().empty()){
        archive->write("name", object->name());
    }
}


void StdSceneWriter::Impl::writeGroup(Mapping* archive, SgGroup* group)
{
    writeObjectHeader(archive, "Group", group);
}


void StdSceneWriter::Impl::writePosTransform(Mapping* archive, SgPosTransform* transform)
{
    archive->setFloatingNumberFormat("%.12g");
    writeObjectHeader(archive, "Transform", transform);
    AngleAxis aa(transform->rotation());
    if(aa.angle() != 0.0){
        writeDegreeAngleAxis(archive, "rotation", aa);
    }
    Vector3 p(transform->translation());
    if(!p.isZero()){
        write(archive, "translation", p);
    }
}


void StdSceneWriter::Impl::writeScaleTransform(Mapping* archive, SgScaleTransform* transform)
{
    writeObjectHeader(archive, "Transform", transform);
    write(archive, "scale", transform->scale());
}


void StdSceneWriter::Impl::writeShape(Mapping* archive, SgShape* shape)
{
    writeObjectHeader(archive, "Shape", shape);

    if(isAppearanceEnabled){
        if(auto appearance = writeAppearance(shape)){
            archive->insert("appearance", appearance);
        }
    }
    if(auto geometry = writeGeometry(shape)){
        archive->insert("geometry", geometry);
    }
}


MappingPtr StdSceneWriter::Impl::writeGeometry(SgShape* shape)
{
    auto mesh = shape->mesh();
    if(!mesh){
        return nullptr;
    }

    MappingPtr archive = new Mapping;

    if(mesh->hasUri() && (extModelFileMode != EmbedModels)){
        archive->write("type", "Resource");
        if(extModelFileMode == LinkToOriginalModelFiles){
            makeLinkToOriginalModelFile(archive, mesh);
        } else {
            if(!replaceOriginalModelFile(archive, shape, false, mesh)){
                // TODO: write the models as embeded models when the replacement fails
                archive.reset();
            }
        }
        if(archive){
            writeMeshAttributes(archive, mesh);
        }
    } else {
        switch(mesh->primitiveType()){
        case SgMesh::MeshType:
            if(!writeMesh(archive, mesh)){
                archive.reset();
            }
            break;
        case SgMesh::BoxType:
            writeBox(archive, mesh);
            break;
        case SgMesh::SphereType:
            writeSphere(archive, mesh);
            break;
        case SgMesh::CylinderType:
            writeCylinder(archive, mesh);
            break;
        case SgMesh::ConeType:
            writeCone(archive, mesh);
            break;
        case SgMesh::CapsuleType:
            writeCapsule(archive, mesh);
            break;
        default:
            archive.reset();
            break;
        }
    }

    return archive;
}


void StdSceneWriter::Impl::writeMeshAttributes(Mapping* archive, SgMesh* mesh)
{
    if(mesh->creaseAngle() > 0.0f){
        archive->write("crease_angle", degree(mesh->creaseAngle()));
    }
    if(mesh->isSolid()){
        archive->write("solid", true);
    }
}


bool StdSceneWriter::Impl::writeMesh(Mapping* archive, SgMesh* mesh)
{
    bool isValid = false;
    int numTriangles = mesh->numTriangles();

    if(mesh->hasVertices() && numTriangles > 0){

        archive->write("type", "TriangleMesh");

        writeMeshAttributes(archive, mesh);

        auto vertices = archive->createFlowStyleListing("vertices");
        const auto srcVertices = mesh->vertices();
        const int scalarElementSize = srcVertices->size() * 3;
        vertices->reserve(scalarElementSize);
        for(auto& v : *srcVertices){
            vertices->append(v.x(), 12, scalarElementSize);
            vertices->append(v.y(), 12, scalarElementSize);
            vertices->append(v.z(), 12, scalarElementSize);
        }
        
        Listing& indexList = *archive->createFlowStyleListing("faces");
        const int numTriScalars = numTriangles * 3;
        indexList.reserve(numTriScalars);
        for(int i=0; i < numTriangles; ++i){
            auto triangle = mesh->triangle(i);
            indexList.append(triangle[0], 15, numTriScalars);
            indexList.append(triangle[1], 15, numTriScalars);
            indexList.append(triangle[2], 15, numTriScalars);
        }

        if(mesh->hasNormals() && mesh->creaseAngle() == 0.0f){
            auto normals = archive->createFlowStyleListing("normals");
            const auto srcNormals = mesh->normals();
            const int scalarElementSize = srcNormals->size() * 3;
            normals->reserve(scalarElementSize);
            for(auto& n : *srcNormals){
                normals->append(n.x(), 12, scalarElementSize);
                normals->append(n.y(), 12, scalarElementSize);
                normals->append(n.z(), 12, scalarElementSize);
            }
            if(mesh->hasNormalIndices()){
                const auto& srcNormalIndices = mesh->normalIndices();
                const int n = srcNormalIndices.size();
                Listing& indexList = *archive->createFlowStyleListing("normal_indices");
                indexList.reserve(n);
                for(auto& index : srcNormalIndices){
                    indexList.append(index, 15, n);
                }
            }
        }

        if(isAppearanceEnabled && mesh->hasTexCoords()){
            auto texCoords = archive->createFlowStyleListing("tex_coords");
            const auto srcTexCoords = mesh->texCoords();
            const int scalarElementSize = srcTexCoords->size() * 2;
            texCoords->reserve(scalarElementSize);
            for(auto& t : *srcTexCoords){
                texCoords->append(t.x(), 12, scalarElementSize);
                texCoords->append(t.y(), 12, scalarElementSize);
            }
            if(mesh->hasTexCoordIndices()){
                const auto& srcTexCoordIndices = mesh->texCoordIndices();
                const int n = srcTexCoordIndices.size();
                Listing& indexList = *archive->createFlowStyleListing("tex_coord_indices");
                indexList.reserve(n);
                for(auto& index : srcTexCoordIndices){
                    indexList.append(index, 15, n);
                }
            }
        }
        
        isValid = true;
    }
    
    return isValid;
}


void StdSceneWriter::Impl::writePrimitiveAttributes(Mapping* archive, SgMesh* mesh)
{
    if(mesh->isSolid()){
        archive->write("solid", true);
    }
    if(mesh->divisionNumber() >= 1){
        archive->write("division_number", mesh->divisionNumber());
    }
    if(mesh->extraDivisionNumber() >= 1){
        archive->write("extra_division_number", mesh->extraDivisionNumber());
        int mode = mesh->extraDivisionMode();
        if(mode != SgMesh::ExtraDivisionPreferred){
            string symbol;
            if(mode == SgMesh::ExtraDivisionX){
                symbol = "x";
            } else if(mode == SgMesh::ExtraDivisionY){
                symbol = "y";
            } else if(mode == SgMesh::ExtraDivisionZ){
                symbol = "z";
            }
            archive->write("extra_division_mode", symbol);
        }
    }
}


void StdSceneWriter::Impl::writeBox(Mapping* archive, SgMesh* mesh)
{
    const auto& box = mesh->primitive<SgMesh::Box>();
    archive->write("type", "Box");
    write(archive, "size", box.size);
    writePrimitiveAttributes(archive, mesh);
}


void StdSceneWriter::Impl::writeSphere(Mapping* archive, SgMesh* mesh)
{
    const auto& sphere = mesh->primitive<SgMesh::Sphere>();
    archive->write("type", "Sphere");
    archive->write("radius", sphere.radius);
    writePrimitiveAttributes(archive, mesh);
}


void StdSceneWriter::Impl::writeCylinder(Mapping* archive, SgMesh* mesh)
{
    const auto& cylinder = mesh->primitive<SgMesh::Cylinder>();
    archive->write("type", "Cylinder");
    archive->write("radius", cylinder.radius);
    archive->write("height", cylinder.height);
    if(!cylinder.top){
        archive->write("top", false);
    }
    if(!cylinder.bottom){
        archive->write("bottom", false);
    }
    writePrimitiveAttributes(archive, mesh);
}


void StdSceneWriter::Impl::writeCone(Mapping* archive, SgMesh* mesh)
{
    const auto& cone = mesh->primitive<SgMesh::Cone>();
    archive->write("type", "Cone");
    archive->write("radius", cone.radius);
    archive->write("height", cone.height);
    if(!cone.bottom){
        archive->write("bottom", false);
    }
    writePrimitiveAttributes(archive, mesh);
}


void StdSceneWriter::Impl::writeCapsule(Mapping* archive, SgMesh* mesh)
{
    const auto& capsule = mesh->primitive<SgMesh::Capsule>();
    archive->write("type", "Capsule");
    archive->write("radius", capsule.radius);
    archive->write("height", capsule.height);
    writePrimitiveAttributes(archive, mesh);
}


MappingPtr StdSceneWriter::Impl::writeAppearance(SgShape* shape)
{
    MappingPtr archive = new Mapping;

    if(auto materialInfo = writeMaterial(shape->material())){
        archive->insert("material", materialInfo);
    }
    auto texture = shape->texture();
    auto textureInfo = writeTexture(shape->texture());
    if(textureInfo){
        archive->insert("texture", textureInfo);
        if(auto transformInfo = writeTextureTransform(texture->textureTransform())){
            archive->insert("texture_transform", transformInfo);
        }
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

    if(material->ambientIntensity() != defaultMaterial->ambientIntensity()){
        archive->write("ambient", material->ambientIntensity());
    }
    if(material->diffuseColor() != defaultMaterial->diffuseColor()){
        write(archive, "diffuse", material->diffuseColor());
    }
    if(material->emissiveColor() != defaultMaterial->emissiveColor()){
        write(archive, "emissive", material->emissiveColor());
    }
    if(material->specularColor() != defaultMaterial->specularColor()){
        write(archive, "specular", material->specularColor());
    }
    if(material->specularExponent() != defaultMaterial->specularExponent()){
        archive->write("specular_exponent", material->specularExponent());
    }
    if(material->transparency() != defaultMaterial->transparency()){
        archive->write("transparency", material->transparency());
    }

    if(archive->empty()){
        archive = nullptr;
    }
    
    return archive;
}


MappingPtr StdSceneWriter::Impl::writeTexture(SgTexture* texture)
{
    if(!texture){
        return nullptr;
    }

    MappingPtr archive = new Mapping;
    bool isValid = false;

    if(auto image = texture->image()){
        if(image->hasUri()){
            string baseDir;
            if(uriDirectoryStack.empty()){
                baseDir = baseDirPath.string();
            } else {
                baseDir = (baseDirPath / uriDirectoryStack.back()).string();
            }
            if(self->findOrCopyImageFile(image, baseDir)){
                filesystem::path path(image->uri());
                if(!uriDirectoryStack.empty()){
                    path = uriDirectoryStack.back() / path;
                }
                archive->write("uri", path.generic_string(), DOUBLE_QUOTED);
                isValid = true;
                if(texture->repeatS() == texture->repeatT()){
                    archive->write("repeat", texture->repeatS());
                } else {
                    auto& repeat = *archive->createFlowStyleListing("repeat");
                    repeat.append(texture->repeatS());
                    repeat.append(texture->repeatT());
                }
            }
        }
    }
    if(!isValid){
        archive.reset();
    }
    return archive;
}


MappingPtr StdSceneWriter::Impl::writeTextureTransform(SgTextureTransform * transform)
{
    if(!transform){
        return nullptr;
    }

    MappingPtr archive = new Mapping;
    if(!transform->center().isZero()){
        write(archive, "center", transform->center());
    }
    if(!transform->scale().isOnes()){
        write(archive, "scale", transform->scale());
    }
    if(!transform->translation().isZero()){
        write(archive, "translation", transform->translation());
    }
    if(transform->rotation() != 0.0){
        archive->write("rotation", transform->rotation());
    }
    return archive;
}


void StdSceneWriter::Impl::writeLight(Mapping* archive, SgLight* light)
{
    write(archive, "color", light->color());
    archive->write("intensity", light->intensity());
    archive->write("ambient", light->ambientIntensity());
}
    

void StdSceneWriter::Impl::writeDirectionalLight(Mapping* archive, SgDirectionalLight* light)
{
    writeObjectHeader(archive, "DirectionalLight", light);
    writeLight(archive, light);
    write(archive, "direction", light->direction());
}


void StdSceneWriter::Impl::writePointLight(Mapping* archive, SgPointLight* light)
{
    writeObjectHeader(archive, "PointLight", light);
    writeLight(archive, light);
    write(archive, "attenuation",
          Vector3(light->constantAttenuation(),
                  light->linearAttenuation(),
                  light->quadraticAttenuation()));
}


void StdSceneWriter::Impl::writeSpotLight(Mapping* archive, SgSpotLight* light)
{
    writeObjectHeader(archive, "SpotLight", light);
    writePointLight(archive, light);
    write(archive, "direction", light->direction());
    archive->write("beam_width", degree(light->beamWidth()));
    archive->write("cut_off_angle", degree(light->cutOffAngle()));
    archive->write("cut_off_exponent", light->cutOffExponent());
}


void StdSceneWriter::Impl::writeCamera(Mapping* archive, SgCamera* camera)
{
    archive->write("near_clip_distance", camera->nearClipDistance());
    archive->write("far_clip_distance", camera->farClipDistance());
}


void StdSceneWriter::Impl::writePerspectiveCamera(Mapping* archive, SgPerspectiveCamera* camera)
{
    writeObjectHeader(archive, "PerspectiveCamera", camera);
    writeCamera(archive, camera);
    archive->write("field_of_view", camera->fieldOfView());
}


void StdSceneWriter::Impl::writeOrthographicCamera(Mapping* archive, SgOrthographicCamera* camera)
{
    writeObjectHeader(archive, "OrthographicCamera", camera);
    writeCamera(archive, camera);
    archive->write("height", camera->height());
}


void StdSceneWriter::Impl::writeFog(Mapping* archive, SgFog* fog)
{
    writeObjectHeader(archive, "Fog", fog);
    write(archive, "color", fog->color());
    archive->write("visibility_range", fog->visibilityRange());
}
