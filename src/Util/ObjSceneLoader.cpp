#include "ObjSceneLoader.h"
#include "SimpleScanner.h"
#include "SceneDrawables.h"
#include "SceneLoader.h"
#include "Triangulator.h"
#include "ImageIO.h"
#include "NullOut.h"
#include <unordered_map>
#include <algorithm>
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


void readVector2Ex(SimpleScanner& scanner, Vector2f& v)
{
    v.x() = scanner.readFloatEx();
    v.y() = scanner.readFloatEx();
}

void readVector3Ex(SimpleScanner& scanner, Vector3f& v)
{
    v.x() = scanner.readFloatEx();
    v.y() = scanner.readFloatEx();
    v.z() = scanner.readFloatEx();
}

};

namespace cnoid {

class ObjSceneLoader::Impl
{
public:
    SimpleScanner scanner;
    SimpleScanner subScanner;
    string token;
    SgGroupPtr group;
    SgVertexArrayPtr vertices;
    SgNormalArrayPtr normals;
    SgTexCoordArrayPtr texCoords;
    SgShapePtr currentShape;
    SgMeshPtr currentMesh;
    SgIndexArray* currentVertexIndices;
    SgIndexArray* currentNormalIndices;
    SgIndexArray* currentTexCoordIndices;
    Triangulator<SgVertexArray> triangulator;
    vector<int> polygon;

    struct MaterialInfo
    {
        SgMaterialPtr material;
        SgTexturePtr texture;
        MaterialInfo(){
            material = new SgMaterial;
        }
    };
    
    unordered_map<string, MaterialInfo> materialMap;
    MaterialInfo* currentMaterialInfo;
    MaterialInfo* currentMaterialDefInfo;
    SgMaterial* currentMaterialDef;
    MaterialInfo dummyMaterialInfo;
    ImageIO imageIO;
    
    ostream* os_;
    ostream& os() { return *os_; }

    string fileBaseName;
    filesystem::path directoryPath;

    Impl();
    void clearBufObjects();
    SgNode* load(const string& filename);
    SgNodePtr loadScene();
    void createNewNode(const std::string& name);
    bool checkAndAddCurrentNode();
    void readVertex();
    void readNormal();
    void readTextureCoordinate();
    void readFace();
    bool readFaceElement(int axis);
    bool loadMaterialTemplateLibrary(const std::string& name);
    void readMaterial(const std::string& name);
    void createNewMaterial(const string& name);
    void readAmbientColor();
    void readDiffuseColor();
    void readSpecularColor();
    void readEmissiveColor();
    void readSpecularExponent();
    void readOpticalDensity();
    void readDissolve();
    void readTransparency();
    void readIlluminationModel();
    void readTexture(const std::string& mapType);
};

}


ObjSceneLoader::ObjSceneLoader()
{
    impl = new Impl;
}


ObjSceneLoader::Impl::Impl()
{
    imageIO.setUpsideDown(true);
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
    vertices.reset();
    normals.reset();
    texCoords.reset();
    currentShape.reset();
    currentMesh.reset();
    currentVertexIndices = nullptr;
    currentNormalIndices = nullptr;
    currentTexCoordIndices = nullptr;
    materialMap.clear();
    currentMaterialInfo = nullptr;
    currentMaterialDefInfo = &dummyMaterialInfo;
    currentMaterialDef = dummyMaterialInfo.material;
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
    filesystem::path filePath(filename);
    fileBaseName = filePath.stem().string();
    directoryPath = filePath.parent_path();

    SgNodePtr scene;

    vertices = new SgVertexArray;
    triangulator.setVertices(*vertices);
    normals = new SgNormalArray;
    texCoords = new SgTexCoordArray;
    currentMaterialInfo = nullptr;
    currentMaterialDefInfo = &dummyMaterialInfo;
    currentMaterialDef = dummyMaterialInfo.material;
    
    try {
        scene = loadScene();
    }
    catch(const std::exception& ex){
        os() << ex.what() << endl;
    }

    if(scene){
        scene->setUriByFilePathAndCurrentDirectory(filename);
    }

    scanner.close();
    clearBufObjects();

    return scene.retn();
}


SgNodePtr ObjSceneLoader::Impl::loadScene()
{
    group = new SgGroup;

    createNewNode(fileBaseName);

    while(scanner.getLine()){

        switch(scanner.peekChar()){
            
        case 'v':
            scanner.moveForward();
            if(scanner.peekChar() == ' '){
                readVertex();
            } else if(scanner.peekChar() == 'n'){
                scanner.moveForward();
                readNormal();
            } else if(scanner.peekChar() == 't'){
                scanner.moveForward();
                readTextureCoordinate();
            } else {
                scanner.throwEx("Unsupported directive");
            }
            break;
            
        case 'f':
            scanner.moveForward();
            readFace();
            break;

        case 'l':
            break;
            
        case 'm':
            if(scanner.checkStringAtCurrentPosition("mtllib ")){
                scanner.readStringToEOL(token);
                loadMaterialTemplateLibrary(token);
            } else {
                scanner.readString(token);
                scanner.throwEx(format("Unsupported directive '{0}'", token));
            }
            break;

        case 'u':
            if(scanner.checkStringAtCurrentPosition("usemtl")){
                scanner.readStringToEOL(token);
                readMaterial(token);
            } else {
                scanner.readString(token);
                scanner.throwEx(format("Unsupported directive '{0}'", token));
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
            break;
            
        case '#':
            break;

        default:
            scanner.skipSpacesAndTabs();
            if(!scanner.checkLF()){
                scanner.throwEx("Unsupported directive");
            }
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
    if(currentShape && !currentMesh->hasTriangles()){
        if(!name.empty()){
            currentShape->setName(name);
        }
    } else {
        if(currentShape){
            checkAndAddCurrentNode();
        }

        if(name.empty() && currentShape){
            auto newShape = new SgShape;
            newShape->setName(currentShape->name());
            currentShape = newShape;
        } else {
            currentShape = new SgShape;
            currentShape->setName(name);
        }
        currentMesh = currentShape->getOrCreateMesh();
        currentMesh->setVertices(vertices);
        currentMesh->setNormals(normals);
        currentMesh->setTexCoords(texCoords);
        currentVertexIndices = &currentMesh->triangleVertices();
        currentNormalIndices = &currentMesh->normalIndices();
        currentTexCoordIndices = &currentMesh->texCoordIndices();
    }
}


bool ObjSceneLoader::Impl::checkAndAddCurrentNode()
{
    if(!currentShape){
        return false;
    }

    bool isValid = true;

    if(currentVertexIndices->empty()){
        isValid = false;

    } else if(!currentNormalIndices->empty() && (currentNormalIndices->size() != currentVertexIndices->size())){
        throw std::runtime_error(
            format("The number of the face normal indices is different from that of vertices in {0}.",
                   currentShape->name()));

    } else if(!currentTexCoordIndices->empty() && (currentTexCoordIndices->size() != currentVertexIndices->size())){
        throw std::runtime_error(
            format("The number of the tex coord indices is different from that of vertices in {0}.",
                   currentShape->name()));
    }

    if(isValid){
        if(currentNormalIndices->empty()){
            currentMesh->setNormals(nullptr);
        }
        if(currentTexCoordIndices->empty()){
            currentMesh->setTexCoords(nullptr);
        }

        if(currentMaterialInfo){
            currentShape->setMaterial(currentMaterialInfo->material);
            if(auto texture = currentMaterialInfo->texture){
                if(currentMesh->hasTexCoords() && currentMesh->hasTexCoordIndices()){
                    //texutre->setRepeat();
                    currentShape->setTexture(texture);
                }
            }
        }

        currentMesh->updateBoundingBox();
        group->addChild(currentShape);
    }

    return isValid;
}


void ObjSceneLoader::Impl::readVertex()
{
    vertices->emplace_back();
    readVector3Ex(scanner, vertices->back());
}


void ObjSceneLoader::Impl::readNormal()
{
    normals->emplace_back();
    readVector3Ex(scanner, normals->back());
}


void ObjSceneLoader::Impl::readTextureCoordinate()
{
    texCoords->emplace_back();
    readVector2Ex(scanner, texCoords->back());
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
    if(axis <= 2){
        scanner.throwEx("The number of face elements is less than thrree");

    } else if(axis >= 4){
        int index0 = currentVertexIndices->size() - axis;
        polygon.resize(axis);
        auto vpos = currentVertexIndices->begin() + index0;
        std::copy(vpos, vpos + axis, polygon.begin());
        int numTriangles = triangulator.apply(polygon);
        const auto& triangles = triangulator.triangles();

        bool needTofixNormalIndices =
            currentVertexIndices->size() == currentNormalIndices->size();
        bool needTofixTexCoordIndices =
            currentVertexIndices->size() == currentTexCoordIndices->size();
        
        currentVertexIndices->resize(index0);
        int localIndex = 0;
        for(int i=0; i < numTriangles; ++i){
            for(int j=0; j < 3; ++j){
                currentVertexIndices->push_back(polygon[triangles[localIndex++]]);
            }
        }
        if(needTofixNormalIndices){
            auto npos = currentNormalIndices->begin() + index0;
            std::copy(npos, npos + axis, polygon.begin());
            currentNormalIndices->resize(index0);
            int localIndex = 0;
            for(int i=0; i < numTriangles; ++i){
                for(int j=0; j < 3; ++j){
                    currentNormalIndices->push_back(polygon[triangles[localIndex++]]);
                }
            }
        }
        if(needTofixTexCoordIndices){
            auto npos = currentTexCoordIndices->begin() + index0;
            std::copy(npos, npos + axis, polygon.begin());
            currentTexCoordIndices->resize(index0);
            int localIndex = 0;
            for(int i=0; i < numTriangles; ++i){
                for(int j=0; j < 3; ++j){
                    currentTexCoordIndices->push_back(polygon[triangles[localIndex++]]);
                }
            }
        }
    }
}


bool ObjSceneLoader::Impl::readFaceElement(int axis)
{
    int index;
    if(!scanner.readInt(index)){
        return false;
    }
        
    currentVertexIndices->push_back(index - 1);

    if(scanner.checkCharAtCurrentPosition('/')){
        if(scanner.readInt(index)){
            currentTexCoordIndices->push_back(index - 1);
        }
        if(scanner.checkCharAtCurrentPosition('/')){
            currentNormalIndices->push_back(scanner.readIntEx() - 1);
        }
    }

    return true;
}


void ObjSceneLoader::Impl::readMaterial(const std::string& name)
{
    createNewNode("");
    
    auto p = materialMap.find(name);
    if(p != materialMap.end()){
        currentMaterialInfo = &p->second;
    } else {
        currentMaterialInfo = nullptr;
    }
}


bool ObjSceneLoader::Impl::loadMaterialTemplateLibrary(const std::string& filename)
{
    if(!subScanner.open((directoryPath / filename).string())){
        os() << format("Material template library file \"{0}\" cannot be open.", filename) << endl;
        return false;
    }

    while(subScanner.getLine()){

        bool isUnknownDirective = false;
        
        if(subScanner.checkStringAtCurrentPosition("newmtl")){
            subScanner.readStringToEOL(token);
            createNewMaterial(token);
        } else {
            subScanner.skipSpacesAndTabs();
            switch(subScanner.peekChar()){

            case 'K':
                subScanner.moveForward();
                switch(subScanner.readChar()){
                case 'a': // Ka
                    readAmbientColor();
                    break;
                case 'd': // Kd
                    readDiffuseColor();
                    break;
                case 's': // Ks
                    readSpecularColor();
                    break;
                case 'e': // Ke
                    readEmissiveColor();
                    break;
                default:
                    isUnknownDirective = true;
                    break;
                }
                break;
                
            case 'N':
                subScanner.moveForward();
                switch(subScanner.readChar()){
                case 's': // Ns
                    readSpecularExponent();
                    break;
                case 'i': // Ni
                    readOpticalDensity();
                    break;
                default:
                    isUnknownDirective = true;
                    break;
                }
                break;
                
            case 'd':
                subScanner.moveForward();
                readDissolve();
                break;
                
            case 'T':
                subScanner.moveForward();
                switch(subScanner.readChar()){
                case 'r': // Tr
                    readTransparency();
                    break;
                default:
                    isUnknownDirective = true;
                    break;
                }
                break;

            case 'i':
                if(subScanner.checkStringAtCurrentPosition("illum")){
                   readIlluminationModel();
                } else {
                    isUnknownDirective = true;
                }
                break;

            case 'm':
                if(subScanner.readStringAtCurrentPosition(token)){
                    if(token.find_first_of("map_") == 0){
                        readTexture(token.substr(4));
                    } else {
                        isUnknownDirective = true;
                    }
                } else {
                    isUnknownDirective = true;
                }
                break;

            case '#':
                break;
            }
        }
        if(isUnknownDirective){
            // subScanner.throwEx("Unsupported directive");
        }
    }

    subScanner.close();
    return true;
}


void ObjSceneLoader::Impl::createNewMaterial(const string& name)
{
    currentMaterialDefInfo = &materialMap[name];
    currentMaterialDefInfo->material->setName(name);
    currentMaterialDef = currentMaterialDefInfo->material;
}


void ObjSceneLoader::Impl::readAmbientColor()
{
    Vector3f a;
    readVector3Ex(subScanner, a);
    float intensity = std::max(a[0], std::max(a[1], a[2]));
    currentMaterialDef->setAmbientIntensity(intensity);
}


void ObjSceneLoader::Impl::readDiffuseColor()
{
    Vector3f c;
    readVector3Ex(subScanner, c);
    currentMaterialDef->setDiffuseColor(c);
}


void ObjSceneLoader::Impl::readSpecularColor()
{
    Vector3f c;
    readVector3Ex(subScanner, c);
    currentMaterialDef->setSpecularColor(c);
}


void ObjSceneLoader::Impl::readEmissiveColor()
{
    Vector3f c;
    readVector3Ex(subScanner, c);
    currentMaterialDef->setEmissiveColor(c);
}


void ObjSceneLoader::Impl::readSpecularExponent()
{
    currentMaterialDef->setSpecularExponent(subScanner.readFloatEx());
}


void ObjSceneLoader::Impl::readOpticalDensity()
{

}


void ObjSceneLoader::Impl::readDissolve()
{
    currentMaterialDef->setTransparency(1.0f - subScanner.readFloatEx());
}


void ObjSceneLoader::Impl::readTransparency()
{
    currentMaterialDef->setTransparency(subScanner.readFloatEx());
}


void ObjSceneLoader::Impl::readIlluminationModel()
{

}


void ObjSceneLoader::Impl::readTexture(const std::string& mapType)
{
    if(mapType == "Kd"){
        if(subScanner.readStringToEOL(token)){
            filesystem::path path(token);
            if(path.is_relative()){
                path = directoryPath / path;
            }
            auto filename = path.string();
            SgTexturePtr texture = new SgTexture;
            auto image = texture->getOrCreateImage();
            if(imageIO.load(image->image(), filename, os())){
                image->setUriByFilePathAndBaseDirectory(
                    token, directoryPath.generic_string());
                currentMaterialDefInfo->texture = texture;
            }
        }
    }
}
