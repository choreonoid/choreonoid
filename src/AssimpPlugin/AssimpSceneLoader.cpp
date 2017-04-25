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

    typedef map<unsigned int, SgShapePtr> AiIndexToSgShapeMap;
    AiIndexToSgShapeMap aiIndexToSgShapeMap;
    typedef map<unsigned int, SgMaterialPtr> AiIndexToSgMaterialMap;
    AiIndexToSgMaterialMap  aiIndexToSgMaterialMap;
    typedef map<unsigned int, SgTexturePtr> AiIndexToSgTextureMap;
    AiIndexToSgTextureMap  aiIndexToSgTextureMap;
    typedef map<string, SgImagePtr> ImagePathToSgImageMap;
    ImagePathToSgImageMap imagePathToSgImageMap;

    AssimpSceneLoaderImpl();
    void clear();
    SgNodePtr load(const std::string& filename);
    SgPosTransformPtr convertAiNode(aiNode* node);
    SgShape* convertAiMesh(unsigned int);
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
    importer.SetPropertyBool(AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION, true);
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


SgNodePtr AssimpSceneLoader::load(const std::string& filename)
{
    return impl->load(filename);
}


SgNodePtr AssimpSceneLoaderImpl::load(const std::string& filename)
{
    clear();

    scene = importer.ReadFile(
        filename,
        aiProcess_Triangulate                //三角面へ変換
        //| aiProcess_JoinIdenticalVertices  //重複する頂点座標の削除
        | aiProcess_GenNormals               //面法線を生成
        //| aiProcess_FindInstances          //重複するメッシュを探し初出のメッシュに置き換える
        //| aiProcess_SortByPType            //異なる複数のプリミティブ型からなるメッシュをサブメッシュに分割
        );

    if(!scene){
        os() << importer.GetErrorString() << endl;
        return 0;
    }

    boost::filesystem::path path(filename);
    directoryPath = path.remove_filename();

    SgPosTransformPtr transform = convertAiNode(scene->mRootNode);

    importer.FreeScene();

    return transform;
}


SgPosTransformPtr AssimpSceneLoaderImpl::convertAiNode(aiNode* node)
{
    SgPosTransformPtr transform = new SgPosTransform;

    transform->setName(node->mName.C_Str());
    
    const aiMatrix4x4& T = node->mTransformation;
    transform->translation() << T[0][3], T[1][3], T[2][3];
    transform->rotation() <<
        T[0][0], T[0][1], T[0][2],
        T[1][0], T[1][1], T[1][2],
        T[2][0], T[2][1], T[2][2];

    for(unsigned int i=0; i < node->mNumMeshes; ++i){
        SgShape* shape = convertAiMesh(node->mMeshes[i]);
        if(shape){
            transform->addChild(shape);
        }
    }

    for(unsigned int i=0; i < node->mNumChildren; ++i){
        SgPosTransformPtr child = convertAiNode(node->mChildren[i]);
        if(child){
            transform->addChild(child);
        }
    }

    if(transform->empty()){
        transform = 0;
    }

    return transform;
}


SgShape* AssimpSceneLoaderImpl::convertAiMesh(unsigned int index)
{
    AiIndexToSgShapeMap::iterator p = aiIndexToSgShapeMap.find(index);
    if(p != aiIndexToSgShapeMap.end()){
        return p->second;
    }

    SgShape* shape = 0;
    aiMesh* srcMesh = scene->mMeshes[index];
    
    if(srcMesh->HasFaces()){
        shape = new SgShape;
        shape->setName(srcMesh->mName.C_Str());
        auto mesh = shape->getOrCreateMesh();
        mesh->setName(shape->name());

        const unsigned int numVertices = srcMesh->mNumVertices;
        const auto srcVertices = srcMesh->mVertices;
        auto& vertices = *mesh->getOrCreateVertices();
        vertices.resize(numVertices);
        for(unsigned int i=0; i < numVertices; ++i){
            const auto& v = srcVertices[i];
            vertices[i] << v.x, v.y, v.z;
        }
        if(srcMesh->HasNormals()){
            const auto srcNormals = srcMesh->mNormals;
            auto& normals = *mesh->getOrCreateNormals();
            normals.resize(numVertices);
            for(unsigned int i=0; i < numVertices; ++i){
                const auto& n = srcNormals[i];
                normals[i] << n.x, n.y, n.z;
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
        if(srcMesh->HasVertexColors(0)){
            const auto srcColors = srcMesh->mColors[0];
            auto& colors = *mesh->getOrCreateColors();
            colors.resize(numVertices);
            for(unsigned int i=0; i < numVertices; ++i){
                const auto& c = srcColors[i];
                colors[i] << c.r, c.g, c.b;
            }
        }

        const unsigned int numFaces = srcMesh->mNumFaces;
        const auto srcFaces = srcMesh->mFaces;
        mesh->setNumTriangles(numFaces);
        for(unsigned int i = 0; i < numFaces; ++i){
            const auto& indices = srcFaces[i].mIndices;
            mesh->setTriangle(i, indices[0], indices[1], indices[2]);
        }

        SgMaterial* material = convertAiMaterial(srcMesh->mMaterialIndex);
        shape->setMaterial(material);

        SgTexture* texture = convertAiTexture(srcMesh->mMaterialIndex);
        if(texture){
            shape->setTexture(texture);
        }
    }
    
    aiIndexToSgShapeMap[index] = shape;

    return shape;
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
    if (AI_SUCCESS == srcMaterial->Get(AI_MATKEY_COLOR_DIFFUSE, color)){
        material->setDiffuseColor(Vector3(color.r, color.g, color.b));
        diffuse = color.r+color.g+color.b;
    }
    if (AI_SUCCESS == srcMaterial->Get(AI_MATKEY_COLOR_SPECULAR, color)){
        material->setSpecularColor(Vector3(color.r, color.g, color.b));
    }
    if (AI_SUCCESS == srcMaterial->Get(AI_MATKEY_COLOR_EMISSIVE, color)){
        material->setEmissiveColor(Vector3(color.r, color.g, color.b));
    }
    float s;
    if (AI_SUCCESS == srcMaterial->Get(AI_MATKEY_SHININESS, s)){
        s = std::min(128.0f, s);
        material->setShininess(s / 128.0f);
    }
    if (AI_SUCCESS == srcMaterial->Get(AI_MATKEY_COLOR_AMBIENT, color)){
        float c = diffuse==0? 0 : (color.r+color.g+color.b)/diffuse;
        if(c>1)
            c=1;
        material->setAmbientIntensity(c);
    }

    float o;
    if (AI_SUCCESS == srcMaterial->Get(AI_MATKEY_OPACITY, o)){
        if(!o){    //設定値が逆のものがある？暫定処理 TODO
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
    if (p != aiIndexToSgTextureMap.end()){
        return p->second;
    }

    SgTexture* texture = 0;
    aiMaterial* srcMaterial = scene->mMaterials[index];

    if (srcMaterial->GetTextureCount(aiTextureType_DIFFUSE) > 0) {
        aiString path;
        if (srcMaterial->GetTexture(aiTextureType_DIFFUSE, 0, &path, NULL, NULL, NULL, NULL, NULL) == AI_SUCCESS) {
            boost::filesystem::path filepath(path.data);
            if(!checkAbsolute(filepath)){
                filepath = directoryPath / filepath;
                filepath.normalize();
            }
            string textureFile = getAbsolutePathString(filepath);

            SgImage* image=0;
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
