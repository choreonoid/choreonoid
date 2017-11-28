/*!
  @author Shizuko Hattori, Shin'ichiro Nakaoka
*/

#include "AssimpSceneLoader.h"
#include <cnoid/SceneLoader>
#include <cnoid/SceneDrawables>
#include <cnoid/ImageIO>
#include <cnoid/FileUtil>
#include <cnoid/Exception>
#include <cnoid/NullOut>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <map>

using namespace std;
using namespace cnoid;

namespace {

struct Registration {
    Registration() {
        SceneLoader::registerLoader(
            "dae;blend;x;obj;dxf",
            []() -> shared_ptr<AbstractSceneLoader> { return std::make_shared<AssimpSceneLoader>(); });
    }
} registration;

}

namespace cnoid {

class AssimpSceneLoaderImpl
{
public:
    ostream* os_;
    ostream& os() { return *os_; }
    Assimp::Importer importer;
    const aiScene* scene;
    boost::filesystem::path directoryPath;
    ImageIO imageIO;

    typedef map<unsigned int, SgNodePtr> AiIndexToSgShapeMap;
    AiIndexToSgShapeMap aiIndexToSgShapeMap;
    typedef map<unsigned int, SgMaterialPtr> AiIndexToSgMaterialMap;
    AiIndexToSgMaterialMap  aiIndexToSgMaterialMap;
    typedef map<unsigned int, SgTexturePtr> AiIndexToSgTextureMap;
    AiIndexToSgTextureMap  aiIndexToSgTextureMap;
    typedef map<string, SgImagePtr> ImagePathToSgImageMap;
    ImagePathToSgImageMap imagePathToSgImageMap;

    AssimpSceneLoaderImpl();
    void clear();
    SgNode* load(const std::string& filename);
    SgTransform* convertAiNode(aiNode* node);
    SgNode* convertAiMesh(unsigned int);
    SgNode* convertAiMeshFaces(aiMesh* srcMesh);
    SgMaterial* convertAiMaterial(unsigned int);
    SgTexture* convertAiTexture(unsigned int index);
};

}


AssimpSceneLoader::AssimpSceneLoader()
{
    impl = new AssimpSceneLoaderImpl();
}


AssimpSceneLoaderImpl::AssimpSceneLoaderImpl()
{
#ifdef AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION
    importer.SetPropertyBool(AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION, true);
#endif

    imageIO.setUpsideDown(true);
    os_ = &nullout();
}


AssimpSceneLoader::~AssimpSceneLoader()
{
    delete impl;
}


void AssimpSceneLoader::setMessageSink(std::ostream& os)
{
    impl->os_ = &os;
}


void AssimpSceneLoaderImpl::clear()
{
    aiIndexToSgShapeMap.clear();
    aiIndexToSgMaterialMap.clear();
    aiIndexToSgTextureMap.clear();
    imagePathToSgImageMap.clear();
}


SgNode* AssimpSceneLoader::load(const std::string& filename)
{
    return impl->load(filename);
}


SgNode* AssimpSceneLoaderImpl::load(const std::string& filename)
{
    clear();

    scene = importer.ReadFile(filename, aiProcess_Triangulate | aiProcess_GenNormals);

    if(!scene){
        os() << importer.GetErrorString() << endl;
        return 0;
    }

    boost::filesystem::path path(filename);
    directoryPath = path.remove_filename();

    SgNode* node = convertAiNode(scene->mRootNode);

    importer.FreeScene();

    return node;
}


SgTransform* AssimpSceneLoaderImpl::convertAiNode(aiNode* node)
{
    const aiMatrix4x4& T = node->mTransformation;
    Affine3 M;
    M.translation() << T[0][3], T[1][3], T[2][3];
    M.linear() << 
        T[0][0], T[0][1], T[0][2],
        T[1][0], T[1][1], T[1][2],
        T[2][0], T[2][1], T[2][2];

    SgTransformPtr transform;
    if(M.linear().isUnitary(1.0e-6)){
        transform = new SgPosTransform(M);
    } else {
        transform = new SgAffineTransform(M);
    }
    transform->setName(node->mName.C_Str());
 
    for(unsigned int i=0; i < node->mNumMeshes; ++i){
        SgNode* shape = convertAiMesh(node->mMeshes[i]);
        if(shape){
            transform->addChild(shape);
        }
    }

    for(unsigned int i=0; i < node->mNumChildren; ++i){
        SgTransform* child = convertAiNode(node->mChildren[i]);
        if(child){
            transform->addChild(child);
        }
    }

    if(transform->empty()){
        transform = 0;
    }

    return transform.retn();
}


SgNode* AssimpSceneLoaderImpl::convertAiMesh(unsigned int index)
{
    AiIndexToSgShapeMap::iterator p = aiIndexToSgShapeMap.find(index);
    if(p != aiIndexToSgShapeMap.end()){
        return p->second;
    }

    SgNode* node = 0;
    aiMesh* srcMesh = scene->mMeshes[index];
    if(srcMesh->HasFaces()){
        node = convertAiMeshFaces(srcMesh);
    }
    aiIndexToSgShapeMap[index] = node;

    return node;;
}


SgNode* AssimpSceneLoaderImpl::convertAiMeshFaces(aiMesh* srcMesh)
{
    const unsigned int types = srcMesh->mPrimitiveTypes;
    SgGroupPtr group = new SgGroup;
    group->setName(srcMesh->mName.C_Str());

    const unsigned int numVertices = srcMesh->mNumVertices;
    const auto srcVertices = srcMesh->mVertices;
    SgVertexArrayPtr vertices = new SgVertexArray;
    vertices->resize(numVertices);
    for(unsigned int i=0; i < numVertices; ++i){
        const auto& v = srcVertices[i];
        vertices->at(i) << v.x, v.y, v.z;
    }

    SgNormalArrayPtr normals;
    if(srcMesh->HasNormals()){
        const auto srcNormals = srcMesh->mNormals;
        normals = new SgNormalArray;
        normals->resize(numVertices);
        for(unsigned int i=0; i < numVertices; ++i){
            const auto& n = srcNormals[i];
            normals->at(i) << n.x, n.y, n.z;
        }
    }

    SgColorArrayPtr colors;
    if(srcMesh->HasVertexColors(0)){
        const auto srcColors = srcMesh->mColors[0];
        colors = new SgColorArray;
        colors->resize(numVertices);
        for(unsigned int i=0; i < numVertices; ++i){
            const auto& c = srcColors[i];
            colors->at(i) << c.r, c.g, c.b;
        }
    }

    SgMaterialPtr material = convertAiMaterial(srcMesh->mMaterialIndex);

    const unsigned int numFaces = srcMesh->mNumFaces;
    const aiFace* srcFaces = srcMesh->mFaces;
    
    if(types & aiPrimitiveType_POINT){
        auto pointSet = new SgPointSet;
        pointSet->setVertices(vertices);
        pointSet->setNormals(normals);
        pointSet->setColors(colors);
        pointSet->setMaterial(material);
        pointSet->updateBoundingBox();
        group->addChild(pointSet);
    }
    
    if(types & aiPrimitiveType_LINE){
        auto lineSet = new SgLineSet;
        lineSet->setVertices(vertices);
        lineSet->setNormals(normals);
        lineSet->setColors(colors);
        lineSet->setMaterial(material);

        if(types == aiPrimitiveType_LINE){
            lineSet->reserveNumLines(numFaces);
        }
        for(unsigned int i = 0; i < numFaces; ++i){
            const aiFace& face = srcFaces[i];
            if(face.mNumIndices == 2){
                const unsigned int* indices = face.mIndices;
                lineSet->addLine(indices[0], indices[1]);
            }
        }
        lineSet->updateBoundingBox();
        
        group->addChild(lineSet);
    }
    
    if(types & aiPrimitiveType_TRIANGLE){
        auto shape = new SgShape;
        shape->setMaterial(material);
        auto mesh = shape->getOrCreateMesh();
        mesh->setVertices(vertices);
        mesh->setNormals(normals);
        mesh->setColors(colors);

        if(types == aiPrimitiveType_TRIANGLE){
            mesh->reserveNumTriangles(numFaces);
        }
        for(unsigned int i = 0; i < numFaces; ++i){
            const aiFace& face = srcFaces[i];
            if(face.mNumIndices == 3){
                const unsigned int* indices = face.mIndices;
                mesh->addTriangle(indices[0], indices[1], indices[2]);
            }
        }

        if(srcMesh->HasTextureCoords(0)){
            const auto srcTexCoords = srcMesh->mTextureCoords[0];
            auto& texCoords = *mesh->getOrCreateTexCoords();
            texCoords.resize(numVertices);
            for(unsigned int i=0; i < numVertices; ++i){
                const auto& p = srcTexCoords[i];
                texCoords[i] << p.x, p.y;
            }
        }
        SgTexture* texture = convertAiTexture(srcMesh->mMaterialIndex);
        if(texture){
            shape->setTexture(texture);
        }

        mesh->updateBoundingBox();

        group->addChild(shape);
    }

    if(group->empty()){
        return 0;
    } else if(group->numChildren() == 1){
        SgNodePtr node = group->child(0);
        node->setName(group->name());
        group->clearChildren();
        return node.retn();
    }
    return group.retn();
}


SgMaterial* AssimpSceneLoaderImpl::convertAiMaterial(unsigned int index)
{
    AiIndexToSgMaterialMap::iterator p = aiIndexToSgMaterialMap.find(index);
    if (p != aiIndexToSgMaterialMap.end()){
        return p->second;
    }

    SgMaterial* material = new SgMaterial();
    aiMaterial* srcMaterial = scene->mMaterials[index];

    aiString name;
    if(AI_SUCCESS == srcMaterial->Get(AI_MATKEY_NAME, name)){
        material->setName(name.C_Str());
    }
    
    aiColor3D color(0.f, 0.f, 0.f);
    float diffuse;
    if(AI_SUCCESS == srcMaterial->Get(AI_MATKEY_COLOR_DIFFUSE, color)){
        material->setDiffuseColor(Vector3(color.r, color.g, color.b));
        diffuse = color.r+color.g+color.b;
    }
    if(AI_SUCCESS == srcMaterial->Get(AI_MATKEY_COLOR_SPECULAR, color)){
        material->setSpecularColor(Vector3(color.r, color.g, color.b));
    }
    if(AI_SUCCESS == srcMaterial->Get(AI_MATKEY_COLOR_EMISSIVE, color)){
        material->setEmissiveColor(Vector3(color.r, color.g, color.b));
    }
    float s;
    if(AI_SUCCESS == srcMaterial->Get(AI_MATKEY_SHININESS, s)){
        s = std::min(128.0f, s);
        material->setShininess(s / 128.0f);
    }
    if(AI_SUCCESS == srcMaterial->Get(AI_MATKEY_COLOR_AMBIENT, color)){
        float c = diffuse==0? 0 : (color.r+color.g+color.b)/diffuse;
        if(c>1)
            c=1;
        material->setAmbientIntensity(c);
    }

    float o;
    if (AI_SUCCESS == srcMaterial->Get(AI_MATKEY_OPACITY, o)){
        if(!o){ // This is temporary processing. Is there anything with the opposite setting value?
            material->setTransparency(o);
        } else {
            material->setTransparency(1.0f - o);
        }
    }

    aiIndexToSgMaterialMap[index] = material;

    return material;

}


SgTexture* AssimpSceneLoaderImpl::convertAiTexture(unsigned int index)
{
    AiIndexToSgTextureMap::iterator p = aiIndexToSgTextureMap.find(index);
    if(p != aiIndexToSgTextureMap.end()){
        return p->second;
    }

    SgTexture* texture = 0;
    aiMaterial* srcMaterial = scene->mMaterials[index];

    if(srcMaterial->GetTextureCount(aiTextureType_DIFFUSE) > 0){
        aiString path;
        if(srcMaterial->GetTexture(aiTextureType_DIFFUSE, 0, &path, NULL, NULL, NULL, NULL, NULL) == AI_SUCCESS){
            boost::filesystem::path filepath(path.data);
            if(!checkAbsolute(filepath)){
                filepath = directoryPath / filepath;
                filepath.normalize();
            }
            string textureFile = getAbsolutePathString(filepath);

            SgImagePtr image;
            ImagePathToSgImageMap::iterator p = imagePathToSgImageMap.find(textureFile);
            if(p != imagePathToSgImageMap.end()){
                image = p->second;
            } else {
                try {
                    image = new SgImage;
                    imageIO.load(image->image(), textureFile);
                    imagePathToSgImageMap[textureFile] = image;
                } catch(const exception_base& ex){
                    os() << *boost::get_error_info<error_info_message>(ex) << endl;
                    image = 0;
                }
            }
            if(image){
                texture = new SgTexture;
                texture->setImage(image);
            }
        }
    }

    aiIndexToSgTextureMap[index] = texture;

    return texture;
}
