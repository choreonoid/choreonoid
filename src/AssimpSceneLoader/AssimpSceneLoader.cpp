#include "AssimpSceneLoader.h"
#include <cnoid/SceneLoader>
#include <cnoid/SceneDrawables>
#include <cnoid/MeshFilter>
#include <cnoid/ImageIO>
#include <cnoid/Exception>
#include <cnoid/NullOut>
#include <cnoid/UTF8>
#include <cnoid/stdx/filesystem>
#include <cnoid/stdx/optional>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <fmt/format.h>
#include <map>
#include <iostream>

using namespace std;
using namespace cnoid;
using fmt::format;
namespace filesystem = cnoid::stdx::filesystem;

namespace {

const bool USE_AFFINE_TRANSFORM = false;

/*
  If the following option is true, transforms with coordinate flip
  are applied to the original mesh vertices so that flipped transforms
  can be excluded from the scene graph.
  However, this conversion may cause some troubles in operating the scene graph,
  so the conversion should be disabled unless there is a special reason.
  Anyway, transformations with flipping should not be included in the scene graph.
  Such transformations can be warned by setting true to the next option.
 */
const bool ENABLE_FLIPPED_COORDINATE_EXPANSION = false;
const bool ENABLE_WARNING_FOR_FLIPPED_COORDINATE = false;

}

namespace cnoid {

class AssimpSceneLoader::Impl
{
public:
    ostream* os_;
    ostream& os() { return *os_; }
    Assimp::Importer importer;
    const aiScene* scene;
    filesystem::path directoryPath;
    ImageIO imageIO;

    stdx::optional<Affine3f> T_local;
    
    typedef map<unsigned int, SgNodePtr> AiIndexToSgShapeMap;
    AiIndexToSgShapeMap aiIndexToSgShapeMap;
    typedef map<unsigned int, SgMaterialPtr> AiIndexToSgMaterialMap;
    AiIndexToSgMaterialMap  aiIndexToSgMaterialMap;
    typedef map<unsigned int, SgTexturePtr> AiIndexToSgTextureMap;
    AiIndexToSgTextureMap  aiIndexToSgTextureMap;
    typedef map<string, SgImagePtr> ImagePathToSgImageMap;
    ImagePathToSgImageMap imagePathToSgImageMap;

    MeshFilter meshFilter;

    Impl();
    void clear();
    SgNode* load(const std::string& filename);
    SgGroup* convertAiNode(aiNode* node);
    SgNode* convertAiMesh(unsigned int);
    SgNode* convertAiMeshFaces(aiMesh* srcMesh);
    SgMaterial* convertAiMaterial(unsigned int);
    SgTexture* convertAiTexture(unsigned int index);
};

}


void AssimpSceneLoader::initializeClass()
{
    SceneLoader::registerLoader(
        { "dae", "blend", "x", "dxf" },
        []() -> shared_ptr<AbstractSceneLoader> { return std::make_shared<AssimpSceneLoader>(); });
}
    

AssimpSceneLoader::AssimpSceneLoader()
{
    impl = nullptr;
}


AssimpSceneLoader::Impl* AssimpSceneLoader::getOrCreateImpl()
{
    if(!impl){
        impl = new Impl;
    }
    return impl;
}

void AssimpSceneLoader::setMessageSinkStdErr()
{
    setMessageSink(std::cerr);
}

AssimpSceneLoader::Impl::Impl()
{
#ifdef AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION
    importer.SetPropertyBool(AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION, true);
#endif

    imageIO.setUpsideDown(true);
    os_ = &nullout();
}


AssimpSceneLoader::~AssimpSceneLoader()
{
    if(impl){
        delete impl;
    }
}


void AssimpSceneLoader::setMessageSink(std::ostream& os)
{
    getOrCreateImpl()->os_ = &os;
}


void AssimpSceneLoader::Impl::clear()
{
    aiIndexToSgShapeMap.clear();
    aiIndexToSgMaterialMap.clear();
    aiIndexToSgTextureMap.clear();
    imagePathToSgImageMap.clear();
}


SgNode* AssimpSceneLoader::load(const std::string& filename)
{
    return insertTransformNodesToAdjustLengthUnitAndUpperAxis(
        getOrCreateImpl()->load(filename));
}


SgNode* AssimpSceneLoader::Impl::load(const std::string& filename)
{
    clear();

    scene = importer.ReadFile(
        filename,
        aiProcess_Triangulate | aiProcess_JoinIdenticalVertices);

    if(!scene || !scene->mRootNode){
        auto error = importer.GetErrorString();
        if(error && *error != '\0'){
            os() << error << endl;
        }
        return nullptr;
    }

    filesystem::path path(fromUTF8(filename));
    directoryPath = path.remove_filename();

    T_local = stdx::nullopt;

    SgNode* node = convertAiNode(scene->mRootNode);

    importer.FreeScene();

    return node;
}


SgGroup* AssimpSceneLoader::Impl::convertAiNode(aiNode* node)
{
    static const bool USE_AFFINE_TRANSFORM = false;
    
    const aiMatrix4x4& S = node->mTransformation;
    Affine3 T;
    T.translation() << S[0][3], S[1][3], S[2][3];
    T.linear() << 
        S[0][0], S[0][1], S[0][2],
        S[1][0], S[1][1], S[1][2],
        S[2][0], S[2][1], S[2][2];

    stdx::optional<Affine3f> prev_T_local = T_local;

    if(ENABLE_FLIPPED_COORDINATE_EXPANSION || ENABLE_WARNING_FOR_FLIPPED_COORDINATE){
        double d = T.linear().determinant();
        if(ENABLE_WARNING_FOR_FLIPPED_COORDINATE){
            if(d < 0){
                os() << format("Warning: Flip transform is included in node \"{0}\".\n", node->mName.C_Str());
            }
        }
        if(ENABLE_FLIPPED_COORDINATE_EXPANSION){
            if(T_local || d < 0){ // include coordinate reflection
                if(T_local){
                    T_local = (*T_local) * T.cast<Matrix3f::Scalar>();
                } else {
                    T_local = T.cast<Matrix3f::Scalar>();
                }
                T.setIdentity();
            }
        }
    }

    SgGroupPtr group;
    SgGroup* groupToAddChildren = nullptr;

    if(T.isApprox(Affine3::Identity())){
        group = new SgGroup;

    } else if(T.linear().isUnitary(1.0e-6)){
        group = new SgPosTransform(Isometry3(T.matrix()));

    } else {
        if(USE_AFFINE_TRANSFORM){
            group = new SgAffineTransform(T);
        } else {
            Vector3 scale;
            for(int i=0; i < 3; ++i){
                double s = T.linear().col(i).norm();
                scale[i] = s;
                T.linear().col(i) /= s;
            }
            SgScaleTransformPtr scaleTransform = new SgScaleTransform(scale);

            if(T.isApprox(Affine3::Identity())){
                group = scaleTransform;
            } else {
                group = new SgPosTransform(Isometry3(T.matrix()));
                group->addChild(scaleTransform);
                groupToAddChildren = scaleTransform;
            }
        }
    }

    group->setName(node->mName.C_Str());
 
    if(!groupToAddChildren){
        groupToAddChildren = group;
    }
    for(unsigned int i=0; i < node->mNumMeshes; ++i){
        SgNode* shape = convertAiMesh(node->mMeshes[i]);
        if(shape){
            groupToAddChildren->addChild(shape);
        }
    }

    for(unsigned int i=0; i < node->mNumChildren; ++i){
        SgGroup* child = convertAiNode(node->mChildren[i]);
        if(child){
            groupToAddChildren->addChild(child);
        }
    }

    if(ENABLE_FLIPPED_COORDINATE_EXPANSION){
        T_local = prev_T_local;
    }
    
    if(group->empty()){
        group = nullptr;
    }

    return group.retn();
}


SgNode* AssimpSceneLoader::Impl::convertAiMesh(unsigned int index)
{
    if(!ENABLE_FLIPPED_COORDINATE_EXPANSION || !T_local){
        AiIndexToSgShapeMap::iterator p = aiIndexToSgShapeMap.find(index);
        if(p != aiIndexToSgShapeMap.end()){
            return p->second;
        }
    }

    SgNode* node = nullptr;
    aiMesh* srcMesh = scene->mMeshes[index];
    if(srcMesh->HasFaces()){
        node = convertAiMeshFaces(srcMesh);
    }
    if(!ENABLE_FLIPPED_COORDINATE_EXPANSION || !T_local){
        aiIndexToSgShapeMap[index] = node;
    }

    return node;;
}


SgNode* AssimpSceneLoader::Impl::convertAiMeshFaces(aiMesh* srcMesh)
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
    if(ENABLE_FLIPPED_COORDINATE_EXPANSION && T_local){
        for(auto& v : *vertices){
            v = (*T_local) * v;
        }
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
        if(ENABLE_FLIPPED_COORDINATE_EXPANSION && T_local){
            const Matrix3f R = (*T_local).linear();
            for(auto& n : *normals){
                n = R * n;
            }
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

        // The following operation takes much time to execute for a large mesh.
        // Texture coordinates must be modified in the following operation.
        /*
        meshFilter.removeRedundantVertices(mesh);
        if(normals){
            meshFilter.removeRedundantNormals(mesh);
        } else {
            meshFilter.generateNormals(mesh);
        }
        */
        if(!normals){
            meshFilter.generateNormals(mesh);
        }

        mesh->updateBoundingBox();

        group->addChild(shape);
    }

    if(group->empty()){
        return nullptr;
    } else if(group->numChildren() == 1){
        SgNodePtr node = group->child(0);
        node->setName(group->name());
        group->clearChildren();
        return node.retn();
    }
    return group.retn();
}


SgMaterial* AssimpSceneLoader::Impl::convertAiMaterial(unsigned int index)
{
    AiIndexToSgMaterialMap::iterator p = aiIndexToSgMaterialMap.find(index);
    if (p != aiIndexToSgMaterialMap.end()){
        return p->second;
    }

    SgMaterial* material = new SgMaterial;
    aiMaterial* srcMaterial = scene->mMaterials[index];

    aiString name;
    if(AI_SUCCESS == srcMaterial->Get(AI_MATKEY_NAME, name)){
        material->setName(name.C_Str());
    }
    
    aiColor3D color(0.f, 0.f, 0.f);
    float diffuse{};
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
        material->setSpecularExponent(s);
    }
    if(AI_SUCCESS == srcMaterial->Get(AI_MATKEY_COLOR_AMBIENT, color)){
        float c = (diffuse == 0.0f ? 0.0f : (color.r + color.g + color.b) / diffuse);
        if(c > 1.0f){
            c = 1.0f;
        }
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


SgTexture* AssimpSceneLoader::Impl::convertAiTexture(unsigned int index)
{
    AiIndexToSgTextureMap::iterator p = aiIndexToSgTextureMap.find(index);
    if(p != aiIndexToSgTextureMap.end()){
        return p->second;
    }

    SgTexture* texture = nullptr;
    aiMaterial* srcMaterial = scene->mMaterials[index];

    if(srcMaterial->GetTextureCount(aiTextureType_DIFFUSE) > 0){
        aiString path;
        if(srcMaterial->GetTexture(aiTextureType_DIFFUSE, 0, &path, NULL, NULL, NULL, NULL, NULL) == AI_SUCCESS){
            filesystem::path filepath(fromUTF8(path.data));
            if(!filepath.is_absolute()){
                filepath = filesystem::lexically_normal(directoryPath / filepath);
            }
            string textureFile = toUTF8(stdx::filesystem::absolute(filepath).string());

            SgImagePtr image;
            ImagePathToSgImageMap::iterator p = imagePathToSgImageMap.find(textureFile);
            if(p != imagePathToSgImageMap.end()){
                image = p->second;
            } else {
                image = new SgImage;
                if(imageIO.load(image->image(), textureFile, os())){
                    image->setUri(path.data, textureFile);
                    imagePathToSgImageMap[textureFile] = image;
                } else {
                    image.reset();
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
