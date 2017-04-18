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
#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <map>

using namespace std;
using namespace cnoid;
using namespace Assimp;

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
    AssimpSceneLoaderImpl();
    SgNodePtr load(const std::string& filename);
    SgPosTransformPtr convertAinode(aiNode* ainode);
    SgShape* convertAiMesh(unsigned int);
    SgMaterial* convertAiMaterial(unsigned int);
    SgTexture* convertAiTexture(unsigned int index);
    void clear();

private:
    boost::filesystem::path directoryPath;
    const aiScene* scene;
    ImageIO imageIO;

    typedef map<unsigned int, SgShapePtr> AiIndexToSgShapeMap;
    AiIndexToSgShapeMap aiIndexToSgShapeMap;
    typedef map<unsigned int, SgMaterialPtr> AiIndexToSgMaterialMap;
    AiIndexToSgMaterialMap  aiIndexToSgMaterialMap;
    typedef map<unsigned int, SgTexturePtr> AiIndexToSgTextureMap;
    AiIndexToSgTextureMap  aiIndexToSgTextureMap;
    typedef map<string, SgImagePtr> ImagePathToSgImageMap;
    ImagePathToSgImageMap imagePathToSgImageMap;
};

}

AssimpSceneLoader::AssimpSceneLoader()
{
    impl = new AssimpSceneLoaderImpl();
}


AssimpSceneLoader::~AssimpSceneLoader()
{
    delete impl;
}


AssimpSceneLoaderImpl::AssimpSceneLoaderImpl()
{
    imageIO.setUpsideDown(true);
    os_ = &nullout();
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

    Importer importer;

    scene = importer.ReadFile(
        filename,
        aiProcess_Triangulate                //三角面へ変換
        //| aiProcess_JoinIdenticalVertices  //重複する頂点座標の削除
        | aiProcess_GenNormals               //面法線を生成
        //| aiProcess_FindInstances          //重複するメッシュを探し初出のメッシュに置き換える
        //| aiProcess_SortByPType            //異なる複数のプリミティブ型からなるメッシュをサブメッシュに分割
        );

    if( !scene )
    {
        os() << importer.GetErrorString() << endl;
        return 0;
    }

    boost::filesystem::path path(filename);
    directoryPath = path.remove_filename();

    SgPosTransformPtr transform = convertAinode(scene->mRootNode);

    return transform;

}


SgPosTransformPtr AssimpSceneLoaderImpl::convertAinode(aiNode* ainode)
{
    const aiMatrix4x4& aiT = ainode->mTransformation;
    Affine3 T;
    T.linear() << aiT[0][0], aiT[0][1], aiT[0][2],
                  aiT[1][0], aiT[1][1], aiT[1][2],
                  aiT[2][0], aiT[2][1], aiT[2][2];
    T.translation() << aiT[0][3], aiT[1][3], aiT[2][3];

    SgPosTransformPtr transform = new SgPosTransform(T);

    for(int i=0; i < ainode->mNumMeshes; ++i){
        SgShape* shape = convertAiMesh(ainode->mMeshes[i]);
        if(shape){
            transform->addChild(shape);
        }
    }

    for(int i=0; i < ainode->mNumChildren; ++i){
        SgPosTransformPtr child = convertAinode(ainode->mChildren[i]);
        if(child){
            transform->addChild(child);
        }
    }

    if(transform->numChildren() == 0){
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

    aiMesh* aimesh = scene->mMeshes[index];
    if(aimesh->HasFaces()){
        shape = new SgShape();
        SgVertexArray* vertices = new SgVertexArray;
        SgNormalArray* normal = new SgNormalArray;
        SgTexCoordArray* texCoord = 0;
        if(aimesh->HasTextureCoords(0)){
            texCoord = new SgTexCoordArray;
        }
        SgColorArray* colors = 0;
        if(aimesh->HasVertexColors(0)){
            colors = new SgColorArray;
        }

        SgMesh* mesh = new SgMesh;
        for (unsigned int i = 0 ; i < aimesh->mNumVertices ; i++) {
            const aiVector3D& pos = aimesh->mVertices[i];
            const aiVector3D& n_ = aimesh->mNormals[i];
            vertices->push_back(Vector3f(pos.x, pos.y, pos.z));
            normal->push_back(Vector3f(n_.x, n_.y, n_.z));
            if(texCoord){
                const aiVector3D& tc_ = aimesh->mTextureCoords[0][i];
                texCoord->push_back(Vector2f(tc_.x, tc_.y));
            }
            if(colors){
                const aiColor4D& co = aimesh->mColors[0][i];
                colors->push_back(Vector3f(co.r, co.g, co.b));
            }
        }
        mesh->setVertices(vertices);
        mesh->setNormals(normal);
        if(texCoord)
            mesh->setTexCoords(texCoord);
        if(colors)
            mesh->setColors(colors);

        for (unsigned int i = 0 ; i < aimesh->mNumFaces ; i++) {
            const aiFace& face = aimesh->mFaces[i];
            mesh->addTriangle(face.mIndices[0], face.mIndices[1], face.mIndices[2]);
        }

        shape->setMesh(mesh);

        SgMaterial* material = convertAiMaterial(aimesh->mMaterialIndex);
        shape->setMaterial(material);

        SgTexture* texture = convertAiTexture(aimesh->mMaterialIndex);
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
    aiMaterial* aimaterial = scene->mMaterials[index];
    
    aiColor3D color(0.f, 0.f, 0.f);
    float diffuse;
    if (AI_SUCCESS == aimaterial->Get(AI_MATKEY_COLOR_DIFFUSE, color)){
        material->setDiffuseColor(Vector3(color.r, color.g, color.b));
        diffuse = color.r+color.g+color.b;
    }
    if (AI_SUCCESS == aimaterial->Get(AI_MATKEY_COLOR_SPECULAR, color)){
        material->setSpecularColor(Vector3(color.r, color.g, color.b));
    }
    if (AI_SUCCESS == aimaterial->Get(AI_MATKEY_COLOR_EMISSIVE, color)){
        material->setEmissiveColor(Vector3(color.r, color.g, color.b));
    }
    float w;
    if (AI_SUCCESS == aimaterial->Get(AI_MATKEY_SHININESS, w)){
        material->setShininess(w);
    }
    if (AI_SUCCESS == aimaterial->Get(AI_MATKEY_COLOR_AMBIENT, color)){
        float c = diffuse==0? 0 : (color.r+color.g+color.b)/diffuse;
        if(c>1)
            c=1;
        material->setAmbientIntensity(c);
    }
    if (AI_SUCCESS == aimaterial->Get(AI_MATKEY_OPACITY, w)){
        if(!w)    //設定値が逆のものがある？暫定処理 TODO
            material->setTransparency(w);
        else
            material->setTransparency(1-w);
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
    aiMaterial* aimaterial = scene->mMaterials[index];

    if (aimaterial->GetTextureCount(aiTextureType_DIFFUSE) > 0) {
        aiString path;
        if (aimaterial->GetTexture(aiTextureType_DIFFUSE, 0, &path, NULL, NULL, NULL, NULL, NULL) == AI_SUCCESS) {
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
