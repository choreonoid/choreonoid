#include "ObjSceneLoader.h"
#include "SimpleScanner.h"
#include "SceneDrawables.h"
#include "SceneLoader.h"
#include "NullOut.h"
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;
namespace filesystem = stdx::filesystem;

namespace {

struct Registration {
    Registration(){
        SceneLoader::registerLoader(
            "obj",
            []() -> shared_ptr<AbstractSceneLoader> {
                return make_shared<ObjSceneLoader>(); });
    }
} registration;

};

namespace cnoid {

class ObjSceneLoader::Impl
{
public:
    SimpleScanner scanner;
    SgGroupPtr group;
    SgShapePtr currentShape;
    SgMeshPtr currentMesh;
    SgVertexArrayPtr currentVertices;
    int vertexIndexOffset;
    SgNormalArrayPtr currentNormals;
    int normalIndexOffset;
    SgIndexArray* currentVertexIndices;
    SgIndexArray* currentNormalIndices;
    
    ostream* os_;
    ostream& os() { return *os_; }

    string fileBaseName;

    Impl();
    void clearBufObjects();
    SgNode* load(const string& filename);
    SgNodePtr loadScene();
    void createNewNode(const std::string& name);
    bool checkAndAddCurrentNode();
    void readVertex();
    void readNormal();
    void readFace();
    bool readFaceElement(int axis);
    void readMaterialTemplateLibrary(const std::string& name);
    void readMaterial(const std::string& name);
};

}


ObjSceneLoader::ObjSceneLoader()
{
    impl = new Impl;
}


ObjSceneLoader::Impl::Impl()
{
    os_ = &nullout();
}


ObjSceneLoader::~ObjSceneLoader()
{
    delete impl;
}


void ObjSceneLoader::setMessageSink(std::ostream& os)
{
    impl->os_ = &os;
}


void ObjSceneLoader::Impl::clearBufObjects()
{
    group.reset();
    currentShape.reset();
    currentMesh.reset();
    currentVertices.reset();
    currentNormals.reset();
    currentVertexIndices = nullptr;
    currentNormalIndices = nullptr;
}


SgNode* ObjSceneLoader::load(const std::string& filename)
{
    return impl->load(filename);
}


SgNode* ObjSceneLoader::Impl::load(const string& filename)
{
    if(!scanner.open(fromUTF8(filename).c_str())){
        os() << format(_("Unable to open file \"{}\"."), filename) << endl;
        return nullptr;
    }
    fileBaseName = filesystem::path(filename).stem().string();

    SgNodePtr scene;

    vertexIndexOffset = 1;
    normalIndexOffset = 1;
    
    try {
        scene = loadScene();
    }
    catch(const std::exception& ex){
        os() << ex.what() << endl;
    }

    if(scene){
        scene->setUri(filename);
    }

    scanner.close();
    clearBufObjects();

    return scene.retn();
}


SgNodePtr ObjSceneLoader::Impl::loadScene()
{
    group = new SgGroup;

    createNewNode(fileBaseName);

    string token;
        
    while(scanner.getLine()){

        switch(scanner.peekChar()){
            
        case 'v':
            scanner.moveForward();
            if(scanner.peekChar() == 'n'){
                scanner.moveForward();
                readNormal();
            } else {
                readVertex();
            }
            break;
            
        case 'f':
            scanner.moveForward();
            readFace();
            break;
            
        case 'm':
            if(scanner.checkStringAtCurrentPosition("mtlib ")){
                scanner.readString(token);
                readMaterialTemplateLibrary(token);
            } else {
                scanner.readString(token);
                scanner.throwEx(format("Unknown directive '{0}'", token));
            }
            break;

        case 'u':
            if(scanner.checkStringAtCurrentPosition("usemtl")){
                scanner.readString(token);
                readMaterial(token);
            } else {
                scanner.readString(token);
                scanner.throwEx(format("Unknown directive '{0}'", token));
            }
            break;

        case 'o':
            scanner.moveForward();
            scanner.readString(token);
            createNewNode(token);
            break;

        case 'g':
            scanner.moveForward();
            scanner.readString(token);
            createNewNode(token);
            break;

        case 's':
            continue;
            
        case '#':
            continue;

        default:
            scanner.checkLFEx();
            break;
        }
    }

    checkAndAddCurrentNode();

    SgNodePtr scene;

    if(!group->empty()){
        if(group->numChildren() == 1){
            scene = group->child(0);
        } else {
            scene = group;
        }
    }
    group.reset();
    return scene.retn();
}


void ObjSceneLoader::Impl::createNewNode(const std::string& name)
{
    if(!currentShape ||
       (currentMesh->hasVertices() || currentMesh->hasNormals() || currentMesh->hasTriangles())){

        if(currentShape){
            checkAndAddCurrentNode();
            vertexIndexOffset += currentVertices->size();
            normalIndexOffset += currentNormals->size();
        }
        
        currentShape = new SgShape;
        currentMesh = currentShape->getOrCreateMesh();
        currentVertices = currentMesh->getOrCreateVertices();
        currentNormals = currentMesh->getOrCreateNormals();
        currentVertexIndices = &currentMesh->triangleVertices();
        currentNormalIndices = &currentMesh->normalIndices();
    }
    currentShape->setName(name);
}


bool ObjSceneLoader::Impl::checkAndAddCurrentNode()
{
    if(!currentShape){
        return false;
    }

    bool isValid = true;

    if(currentVertices->empty() || currentVertexIndices->empty()){
        isValid = false;

    } else {
        if(currentNormals->empty()){
            if(!currentNormalIndices->empty()){
                throw std::runtime_error(
                    format("{0} has normal index data but it does not include normal vectors.",
                           currentShape->name()));
            }
        } else {
            if(currentNormalIndices->empty()){
                throw std::runtime_error(
                    format("Normal indices for faces are not specified in {0}.", currentShape->name()));
                
            } else if(currentVertexIndices->size() != currentNormalIndices->size()){
                throw std::runtime_error(
                    format("The number of the face normal indices is different from that of vertices in {0}.",
                           currentShape->name()));
            }
        }
    }

    if(isValid){
        group->addChild(currentShape);
    }

    return isValid;
}


void ObjSceneLoader::Impl::readVertex()
{
    currentVertices->emplace_back();
    auto& v = currentVertices->back();
    v.x() = scanner.readFloatEx();
    v.y() = scanner.readFloatEx();
    v.z() = scanner.readFloatEx();
}


void ObjSceneLoader::Impl::readNormal()
{
    currentNormals->emplace_back();
    auto& n = currentNormals->back();
    n.x() = scanner.readFloatEx();
    n.y() = scanner.readFloatEx();
    n.z() = scanner.readFloatEx();
}


void ObjSceneLoader::Impl::readFace()
{
    int axis = 0;
    while(true){
        if(!readFaceElement(axis)){
            break;
        }
        ++axis;
    }
    if(axis < 3){
        scanner.throwEx("The number of face elements is less than thrree");
    } else if(axis == 3){

    } else {
        scanner.throwEx("The number of face element is more than three");
    }
}


bool ObjSceneLoader::Impl::readFaceElement(int axis)
{
    int vertexIndex;
    if(!scanner.readInt(vertexIndex)){
        return false;
    }
        
    currentVertexIndices->push_back(vertexIndex - vertexIndexOffset);

    if(scanner.checkCharAtCurrentPosition('/')){
        int texCoordIndex;
        if(scanner.readInt(texCoordIndex)){
            // add texture coordinate here
        }
        if(scanner.checkCharAtCurrentPosition('/')){
            currentNormalIndices->push_back(scanner.readIntEx() - normalIndexOffset);
        }
    }

    return true;
}


void ObjSceneLoader::Impl::readMaterialTemplateLibrary(const std::string& name)
{

}


void ObjSceneLoader::Impl::readMaterial(const std::string& name)
{

}



                

