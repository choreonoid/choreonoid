#include "AssimpSceneLoader.h"
#include <cnoid/SceneLoader>
#include <cnoid/SceneDrawables>
#include <cnoid/MeshFilter>
#include <cnoid/ImageIO>
#include <cnoid/NullOut>
#include <cnoid/UTF8>
#include <cnoid/Format>
#include <filesystem>
#include <optional>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <Eigen/Dense>
#include <map>
#include <cmath>

using namespace std;
using namespace cnoid;

namespace {

const bool USE_AFFINE_TRANSFORM = false;

/*
  Collada and similar source formats sometimes embed a reflection (a linear
  transform whose determinant is negative) into a node, typically when an
  authoring tool mirrors a left-side mesh to obtain its right-side counterpart.
  Such a reflection cannot be carried on the Choreonoid scene graph: only
  SgAffineTransform can hold it, and several downstream subsystems (renderer
  normal matrix, back-face culling, MeshExtractor winding, physics-engine
  collision-shape transforms) assume rotation-only transforms and silently
  produce wrong results when a reflection passes through. See the SgAffineTransform
  comment in SceneGraph.h for details.

  When the following option is true, any reflection found on an aiNode is baked
  into the descendant mesh vertices (with the normals and, if necessary, the
  triangle winding adjusted accordingly), so that the scene graph itself stays
  free of flips.
  This is the recommended behavior and the option should normally be left on.
 */
const bool ENABLE_FLIPPED_COORDINATE_EXPANSION = true;
const bool ENABLE_WARNING_FOR_FLIPPED_COORDINATE = false;

// Registers the loader when this library is loaded so that it can be used independently of
// the Choreonoid GUI (i.e. without the AssimpPlugin calling initializeClass()). The
// registration is idempotent because SceneLoader::registerLoader just overwrites the
// extension-to-loader mapping, so calling initializeClass() again from the plugin is harmless.
struct Registration {
    Registration(){
        AssimpSceneLoader::initializeClass();
    }
} registration;

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

    std::optional<Affine3f> T_local;
    
    typedef map<unsigned int, SgNodePtr> AiIndexToSgShapeMap;
    AiIndexToSgShapeMap aiIndexToSgShapeMap;
    typedef map<unsigned int, SgMaterialPtr> AiIndexToSgMaterialMap;
    AiIndexToSgMaterialMap  aiIndexToSgMaterialMap;
    typedef map<unsigned int, SgTexturePtr> AiIndexToSgTextureMap;
    AiIndexToSgTextureMap  aiIndexToSgTextureMap;
    typedef map<string, SgImagePtr> ImagePathToSgImageMap;
    ImagePathToSgImageMap imagePathToSgImageMap;
    // Additional directories searched for texture image files when they are not found next to
    // the scene file. Populated via AssimpSceneLoader::addImageSearchDirectory() and reset by
    // clearImageSearchDirectories(). Tried in registration order.
    vector<filesystem::path> imageSearchDirectories;

    MeshFilter meshFilter;

    Impl();
    void clear();
    SgNode* load(const std::string& filename);
    SgGroup* convertAiNode(aiNode* node);
    SgGroup* decomposeTransformation(const Affine3& T);
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


AssimpSceneLoader::Impl::Impl()
{
#ifdef AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION
    importer.SetPropertyBool(AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION, true);
#endif

#ifdef AI_CONFIG_IMPORT_COLLADA_USE_COLLADA_NAMES
    // Collada nodes carry both an 'id' (a file-unique identifier, often decorated by the
    // exporter, e.g. "node-Ranger") and a 'name' (the original object name from the
    // authoring tool, e.g. "Ranger"). By default Assimp puts the id into aiNode::mName.
    // This option flips that so the human-meaningful name is used, which is what callers
    // expect when looking nodes up by name (e.g. SDF's <mesh><submesh><name>).
    importer.SetPropertyBool(AI_CONFIG_IMPORT_COLLADA_USE_COLLADA_NAMES, true);
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


void AssimpSceneLoader::addImageSearchDirectory(const std::string& directory)
{
    if(directory.empty()) return;
    getOrCreateImpl()->imageSearchDirectories.emplace_back(fromUTF8(directory));
}


void AssimpSceneLoader::clearImageSearchDirectories()
{
    if(impl){
        impl->imageSearchDirectories.clear();
    }
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

    T_local = std::nullopt;

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

    std::optional<Affine3f> prev_T_local = T_local;

    if(ENABLE_FLIPPED_COORDINATE_EXPANSION || ENABLE_WARNING_FOR_FLIPPED_COORDINATE){
        double d = T.linear().determinant();
        if(ENABLE_WARNING_FOR_FLIPPED_COORDINATE){
            if(d < 0){
                os() << formatC("Warning: Flip transform is included in node \"{0}\".\n", node->mName.C_Str());
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
    if(T.isApprox(Affine3::Identity())){
        group = new SgGroup;
    } else if(T.linear().isUnitary(1.0e-6)){
        group = new SgPosTransform(Isometry3(T.matrix()));
    } else {
        if(!USE_AFFINE_TRANSFORM){
            group = decomposeTransformation(T);
        }
        if(!group){
            group = new SgAffineTransform(T);
        }
    }
    group->setName(node->mName.C_Str());

    SgGroup* groupToAddChildren = group;
    while(!groupToAddChildren->empty()){
        if(auto childGroup = groupToAddChildren->child(0)->toGroupNode()){
            groupToAddChildren = childGroup;
        } else {
            break;
        }
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


SgGroup* AssimpSceneLoader::Impl::decomposeTransformation(const Affine3& T)
{
    Matrix3 Q = T.linear();

    // With ENABLE_FLIPPED_COORDINATE_EXPANSION on, any reflection has already
    // been baked into the descendant mesh vertices in convertAiNode, so a
    // negative determinant should not reach here. Defensive bail-out: if it
    // ever does, return nullptr so the caller falls back to SgAffineTransform
    // rather than producing a broken rotation/scale decomposition.
    if(Q.determinant() < 0.0){
        return nullptr;
    }

    // Step 1: Use polar decomposition to decompose Q into U (rotation) and P (symmetric positive definite matrix)
    // Polar decomposition using SVD
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(Q, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();
    Eigen::Matrix3d P = svd.matrixV() * svd.singularValues().asDiagonal() * svd.matrixV().transpose();
    
    // Step 2: Decompose P as R*S*R^T
    // Perform eigenvalue decomposition on P (= R*S*R^T)
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(P);
    
    if(eigensolver.info() != Eigen::Success){
        return nullptr;
    }
    
    // Get eigenvalues (scaling factors)
    const Vector3& eigenvalues = eigensolver.eigenvalues();
    
    SgGroup* transformGroup = nullptr;
    SgScaleTransform* scaleTransform = nullptr;

    if(!eigenvalues.isApprox(Vector3::Zero())){
        scaleTransform = new SgScaleTransform;
        scaleTransform->setScale(eigenvalues);
            
        // Get rotation matrix R (eigenvectors)
        Matrix3 SR = eigensolver.eigenvectors().template cast<double>();

        // Ensure SR is a proper rotation matrix (determinant = +1)
        if(SR.determinant() < 0){
            // Flip the sign of one eigenvector
            SR.col(0) = -SR.col(0);
        }
        if(!SR.isIdentity()){
            scaleTransform->setScaleOrientation(SR);
        }
    }

    if(R.isIdentity() && T.translation().isApprox(Vector3::Zero())){
        transformGroup = scaleTransform;
    } else {
        SgPosTransform* posTransform = new SgPosTransform;
        posTransform->setRotation(R);
        posTransform->setTranslation(T.translation());
        if(scaleTransform){
            posTransform->addChild(scaleTransform);
        }
        transformGroup = posTransform;
    }

    if(!transformGroup){
        transformGroup = new SgGroup;
    }

    return transformGroup;
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

    // If a non-trivial transform is being baked into the mesh by
    // ENABLE_FLIPPED_COORDINATE_EXPANSION, decide once whether it contains a
    // reflection. Some Collada files already compensate the triangle winding
    // for a reflected node, so the actual winding flip is decided after the
    // transformed normals are available.
    const bool isFlipped =
        ENABLE_FLIPPED_COORDINATE_EXPANSION && T_local &&
        (*T_local).linear().determinant() < 0.0f;

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
            // Normals transform by the inverse transpose of the linear part.
            // This is identical to the linear part itself when the transform
            // is a rotation (possibly with a uniform positive scale), but it
            // matters here precisely because T_local may contain a reflection
            // or a non-uniform scale.
            const Matrix3f N = (*T_local).linear().inverse().transpose();
            for(auto& n : *normals){
                n = (N * n).normalized();
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

    bool flipTriangleWinding = isFlipped;
    if(isFlipped && normals){
        double dotSum = 0.0;
        int numCheckedFaces = 0;
        for(unsigned int i = 0; i < numFaces; ++i){
            const aiFace& face = srcFaces[i];
            if(face.mNumIndices == 3){
                const unsigned int* indices = face.mIndices;
                const Vector3f& v0 = vertices->at(indices[0]);
                const Vector3f& v1 = vertices->at(indices[1]);
                const Vector3f& v2 = vertices->at(indices[2]);
                Vector3f faceNormal = (v1 - v0).cross(v2 - v0);
                if(faceNormal.squaredNorm() > 0.0f){
                    faceNormal.normalize();
                    Vector3f normal =
                        normals->at(indices[0]) + normals->at(indices[1]) + normals->at(indices[2]);
                    if(normal.squaredNorm() > 0.0f){
                        dotSum += faceNormal.dot(normal.normalized());
                        ++numCheckedFaces;
                    }
                }
            }
        }
        if(numCheckedFaces > 0){
            flipTriangleWinding = (dotSum < 0.0);
        }
    }
    
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
                if(flipTriangleWinding){
                    mesh->addTriangle(indices[0], indices[2], indices[1]);
                } else {
                    mesh->addTriangle(indices[0], indices[1], indices[2]);
                }
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
            filesystem::path rawPath(fromUTF8(path.data));
            const bool isAbsolute = rawPath.is_absolute();
            filesystem::path primaryPath =
                isAbsolute ? rawPath : (directoryPath / rawPath).lexically_normal();
            string primaryTextureFile = toUTF8(std::filesystem::absolute(primaryPath).string());

            // First try the path as referenced by the scene file (relative paths resolved
            // against the scene file's directory).
            SgImagePtr image;
            string usedTextureFile = primaryTextureFile;
            ImagePathToSgImageMap::iterator p = imagePathToSgImageMap.find(primaryTextureFile);
            if(p != imagePathToSgImageMap.end()){
                image = p->second;
            } else {
                image = new SgImage;
                // For a relative path, additional image search directories may be tried as a
                // fallback, so suppress diagnostics on the first attempt and let the final
                // attempt report the failure.
                const bool hasFallback = !isAbsolute && !imageSearchDirectories.empty();
                std::ostream& firstSink = hasFallback ? nullout() : os();
                if(imageIO.load(image->image(), primaryTextureFile, firstSink)){
                    image->setUri(path.data, primaryTextureFile);
                    imagePathToSgImageMap[primaryTextureFile] = image;
                } else if(hasFallback){
                    // Try each search directory with just the filename portion.
                    const filesystem::path filenameOnly = rawPath.filename();
                    image.reset();
                    for(size_t i = 0; i < imageSearchDirectories.size(); ++i){
                        filesystem::path fallback =
                            (imageSearchDirectories[i] / filenameOnly).lexically_normal();
                        string fallbackTextureFile =
                            toUTF8(std::filesystem::absolute(fallback).string());
                        auto fp = imagePathToSgImageMap.find(fallbackTextureFile);
                        if(fp != imagePathToSgImageMap.end()){
                            image = fp->second;
                            usedTextureFile = fallbackTextureFile;
                            break;
                        }
                        SgImagePtr trial = new SgImage;
                        const bool isLastTry = (i + 1 == imageSearchDirectories.size());
                        std::ostream& sink = isLastTry ? os() : nullout();
                        if(imageIO.load(trial->image(), fallbackTextureFile, sink)){
                            trial->setUri(path.data, fallbackTextureFile);
                            imagePathToSgImageMap[fallbackTextureFile] = trial;
                            image = trial;
                            usedTextureFile = fallbackTextureFile;
                            break;
                        }
                    }
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
