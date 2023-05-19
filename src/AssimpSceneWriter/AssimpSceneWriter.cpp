#include <assimp/Exporter.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include "AssimpSceneWriter.h"

#include <cnoid/NullOut>
#include <cnoid/SceneDrawables>
#include <cnoid/MeshExtractor>
#include <cnoid/MeshGenerator>

#include <vector>
#include <iostream>
#include <sstream>
using namespace std;
using namespace cnoid;
//using fmt::format;
//namespace filesystem = cnoid::stdx::filesystem;

#if 0
#define DEBUG_STREAM(args) \
    std::cerr << "[" << __PRETTY_FUNCTION__ << "]" << args << std::endl
#else
#define DEBUG_STREAM(args) \
    if(verbose) { self->os() << "[" << __PRETTY_FUNCTION__ << "]" << args << std::endl; }
#endif

namespace {

const bool USE_AFFINE_TRANSFORM = false;

}

namespace cnoid {

class AssimpSceneWriter::Impl
{
public:
    AssimpSceneWriter *self;

    Impl(AssimpSceneWriter *_self);

    void copyConfigurations(const Impl* org);

    std::string output_type;
    bool generate_primitive_mesh;
    bool verbose;
    std::vector<aiMesh*> vec_mesh;
    std::vector<aiMaterial*> vec_material;
    int material_counter;
    //// assimp
    MeshExtractor meshExtractor;
    void callbackMesh();
    void addMesh(SgMesh *mesh);
    aiScene *createScene();
    bool storeScene(aiScene *_pscene, const std::string &_filename, std::string &_id);

    void extractMeshSgNode(SgNode *_geometry)
    {
        vec_mesh.resize(0);
        vec_material.resize(0);
        meshExtractor.extract(_geometry,  [this](){ callbackMesh(); });
    }
    void clear() {
        vec_mesh.resize(0);
        vec_material.resize(0);
    }
};

}

////
AssimpSceneWriter::AssimpSceneWriter()
{
    impl = new Impl(this);
}
AssimpSceneWriter::AssimpSceneWriter(const AssimpSceneWriter& org)
    : AssimpSceneWriter()
{
    impl->copyConfigurations(org.impl);
}
AssimpSceneWriter::~AssimpSceneWriter()
{
    delete impl;
}
void AssimpSceneWriter::setOutputType(const std::string& _type)
{
    impl->output_type = _type;
}
const std::string &AssimpSceneWriter::getOutputType()
{
    return impl->output_type;
}
void AssimpSceneWriter::setVerbose(bool on)
{
    impl->verbose = on;
}
void AssimpSceneWriter::generatePrimitiveMesh(bool on)
{
    impl->generate_primitive_mesh = on;
}
void AssimpSceneWriter::setMessageSinkStdErr()
{
    setMessageSink(std::cerr);
}
bool AssimpSceneWriter::writeScene(const std::string& filename, SgNode* node)
{
    impl->extractMeshSgNode(node);
    aiScene *res_ = impl->createScene();
    if(!res_) {
        return false;
    }
    bool ret_ = impl->storeScene(res_, filename, impl->output_type);
    delete res_;
    return ret_;
}

//// Impl
AssimpSceneWriter::Impl::Impl(AssimpSceneWriter *_self) : self(_self)
{
    generate_primitive_mesh = false;
    verbose = false;
    material_counter = 0;
}
void AssimpSceneWriter::Impl::copyConfigurations(const Impl* org)
{
    output_type = org->output_type;
    generate_primitive_mesh = org->generate_primitive_mesh;
    verbose = org->verbose;
}
void AssimpSceneWriter::Impl::callbackMesh()
{
    SgMesh* mesh = meshExtractor.currentMesh();
    if(mesh->primitiveType() == SgMesh::MeshType && mesh->hasTriangles()) {
        addMesh(mesh);
    } else if (generate_primitive_mesh) {
        MeshGenerator mg_;
        SgMesh* gmesh;
        switch(mesh->primitiveType()) {
        case SgMesh::BoxType:
        {
            SgMesh::Box prim = mesh->primitive<SgMesh::Box>();
            gmesh = mg_.generateBox(prim.size);
        }
        break;
        case SgMesh::SphereType:
        {
            SgMesh::Sphere prim = mesh->primitive<SgMesh::Sphere>();
            gmesh = mg_.generateSphere(prim.radius);
        }
        break;
        case SgMesh::CylinderType:
        {
            SgMesh::Cylinder prim = mesh->primitive<SgMesh::Cylinder>();
            gmesh = mg_.generateCylinder(prim.radius, prim.height);
        }
        break;
        case SgMesh::ConeType:
        {
            SgMesh::Cone prim = mesh->primitive<SgMesh::Cone>();
            gmesh = mg_.generateCylinder(prim.radius, prim.height);
        }
        break;
        case SgMesh::CapsuleType:
        {
            SgMesh::Capsule prim = mesh->primitive<SgMesh::Capsule>();
            gmesh = mg_.generateCylinder(prim.radius, prim.height);
        }
        break;
        default:
        {
            self->os() << "not supported type" << std::endl;
        }
        break;
        }
        addMesh(gmesh);
    }
}
////////
//// normal
// choreonoid vertex_array, normal_array, normal_index, index_array(triangle x 3)
//   index = index_array[n] ( 0 <= n < triangle * 3 )
//   vertex = vertex_array[index]
//   normal = normal_array[normal_index[index]]
// assimp vertex_array, normal_array, index_array(triangle * 3)
//   index = index_array[n] ( 0 <= n < triangle * 3 )
//   vertex = vertex_array[index]
//   normal = normal_array[index]
void AssimpSceneWriter::Impl::addMesh(SgMesh *mesh)
{
    DEBUG_STREAM(" addMesh");
    //SgMesh* mesh = meshExtractor.currentMesh();
    SgShape* shape = meshExtractor.currentShape();
    const Affine3& T = meshExtractor.currentTransform();
    const Isometry3 &Ti = meshExtractor.currentTransformWithoutScaling();
    const Matrix3 Rot(Ti.linear());

    aiMesh *pMesh = new aiMesh();
    pMesh->mPrimitiveTypes = aiPrimitiveType_TRIANGLE;
    const SgVertexArray& vertices_ = *mesh->vertices();
    const size_t numVertices = vertices_.size();
    pMesh->mNumVertices = numVertices;
    pMesh->mVertices = new aiVector3D [numVertices];

    DEBUG_STREAM(" numVertices : " << numVertices);
    //bool hasVertices() const { return (vertices_ && !vertices_->empty()); }
    for(size_t i = 0; i < numVertices; i++) {
        const Vector3 v = T * vertices_[i].cast<Isometry3::Scalar>();
        pMesh->mVertices[i].x = v.x();
        pMesh->mVertices[i].y = v.y();
        pMesh->mVertices[i].z = v.z();
    }
    if (mesh->hasNormals()) {
        const SgNormalArray& normals_ = *mesh->normals();
        const size_t numNormals = normals_.size();
        DEBUG_STREAM(" numNormals : " << numNormals);
        if(mesh->hasNormalIndices()) {
            const SgIndexArray& nIndices = mesh->normalIndices();
            DEBUG_STREAM(" nIndices.size() : " << nIndices.size());
            const SgIndexArray& vIndices = mesh->triangleVertices();
            DEBUG_STREAM(" vIndices.size() : " << vIndices.size());
            if (nIndices.size() == vIndices.size()) {
                pMesh->mNormals  = new aiVector3D [pMesh->mNumVertices];
                for (size_t k = 0; k < nIndices.size(); k++) {
                    Vector3 n = Rot * normals_[nIndices[k]].cast<Isometry3::Scalar>();
                    size_t idx = vIndices[k];
                    n.normalize();
                    pMesh->mNormals[idx].x = n.x();
                    pMesh->mNormals[idx].y = n.y();
                    pMesh->mNormals[idx].z = n.z();
                }
            } else {
                self->os() << "AssimpSceneWriter: parse error! (normals/indices)" << std::endl;
            }
        } else {
            if(numNormals == pMesh->mNumVertices) {
                pMesh->mNormals  = new aiVector3D [numNormals];
                for (size_t k = 0; k < numNormals; k++) {
                    Vector3 n = Rot * normals_[k].cast<Isometry3::Scalar>();
                    n.normalize();
                    pMesh->mNormals[k].x = n.x();
                    pMesh->mNormals[k].y = n.y();
                    pMesh->mNormals[k].z = n.z();
                }
            } else {
                self->os() << "AssimpSceneWriter: parse error! (normals)" << std::endl;
            }
        }
    }
    if (mesh->hasColors()) {
        const SgColorArray& colors_ = *mesh->colors();
        const size_t numColors = colors_.size();
        DEBUG_STREAM(" numColors : " << numColors);
        if(mesh->hasColorIndices()) {
            const SgIndexArray& cIndices = mesh->colorIndices();
            DEBUG_STREAM(" cIndices.size() : " << cIndices.size());
            const SgIndexArray& vIndices = mesh->triangleVertices();
            DEBUG_STREAM(" vIndices.size() : " << vIndices.size());
            if (cIndices.size() == vIndices.size()) {
                pMesh->mColors[0]  = new aiColor4D [pMesh->mNumVertices];
                for (size_t k = 0; k < cIndices.size(); k++) {
                    Vector3f col_ = colors_[cIndices[k]];
                    size_t idx = vIndices[k];
                    aiColor4D &acol_ = pMesh->mColors[0][idx];
                    acol_.r = col_[0];
                    acol_.g = col_[1];
                    acol_.b = col_[2];
                    acol_.a = 1.0;
                }
            } else {
                self->os() << "AssimpSceneWriter: parse error! (colors/indices)" << std::endl;
            }
        } else {
            if(numColors == pMesh->mNumVertices) {
                pMesh->mColors[0]  = new aiColor4D [numColors];
                for (size_t k = 0; k < numColors; k++) {
                    Vector3f col_ = colors_[k];
                    aiColor4D &acol_ = pMesh->mColors[0][k];
                    acol_.r = col_[0];
                    acol_.g = col_[1];
                    acol_.b = col_[2];
                    acol_.a = 1.0;
                }
            } else {
                self->os() << "AssimpSceneWriter: parse error! (colors)" << std::endl;
            }
        }
    }
    if (mesh->hasTexCoords()) {
        const SgTexCoordArray& texcds_ = *mesh->texCoords();
        const size_t numTexCoords = texcds_.size();
        DEBUG_STREAM(" numTexCoords : " << numTexCoords);
        if(mesh->hasTexCoordIndices()) {
            const SgIndexArray& tIndices = mesh->texCoordIndices();
            DEBUG_STREAM(" tIndices.size() : " << tIndices.size());
            const SgIndexArray& vIndices = mesh->triangleVertices();
            DEBUG_STREAM(" vIndices.size() : " << vIndices.size());
            if (tIndices.size() == vIndices.size()) {
                pMesh->mTextureCoords[0] = new aiVector3D [pMesh->mNumVertices];
                for (size_t k = 0; k < tIndices.size(); k++) {
                    Vector2f tx_ = texcds_[tIndices[k]];
                    size_t idx = vIndices[k];
                    pMesh->mTextureCoords[0][idx].x = tx_[0];
                    pMesh->mTextureCoords[0][idx].y = tx_[1];
                }
            } else {
                self->os() << "AssimpSceneWriter: parse error! (texcoords/indices)" << std::endl;
            }
        } else {
            if(numTexCoords == pMesh->mNumVertices) {
                pMesh->mTextureCoords[0] = new aiVector3D [numTexCoords];
                for (size_t k = 0; k < numTexCoords; k++) {
                    Vector2f tx_ = texcds_[k];
                    pMesh->mTextureCoords[0][k].x = tx_[0];
                    pMesh->mTextureCoords[0][k].y = tx_[1];
                }
            } else {
                self->os() << "AssimpSceneWriter: parse error! (texcoords)" << std::endl;
            }
        }
    }
    const int numTriangles = mesh->numTriangles();
    DEBUG_STREAM(" numTriangles : " << numTriangles);
    DEBUG_STREAM(" faceVertexIndices : " << mesh->faceVertexIndices().size());
    pMesh->mNumFaces = numTriangles;
    pMesh->mFaces = new aiFace[numTriangles];
    for(size_t i = 0; i < numTriangles; i++) {
        SgMesh::TriangleRef tri = mesh->triangle(i);
        pMesh->mFaces[i].mNumIndices = 3;
        pMesh->mFaces[i].mIndices = new unsigned int [3];

        pMesh->mFaces[i].mIndices[0] = tri[0];
        pMesh->mFaces[i].mIndices[1] = tri[1];
        pMesh->mFaces[i].mIndices[2] = tri[2];
    }
    vec_mesh.push_back(pMesh);
    { // Material
        SgMaterial *material_ = shape->material();
        aiMaterial* pcMat = new aiMaterial();
        aiString s;
	if(material_->name().empty()) {
	  std::ostringstream oss;
	  oss << AI_DEFAULT_MATERIAL_NAME << material_counter++;
	  s.Set (oss.str().c_str());
	} else {
	  s.Set (material_->name().c_str());
	}
        pcMat->AddProperty (&s, AI_MATKEY_NAME);
        const Vector3f &col_dif = material_->diffuseColor();
        aiColor4D colorDiffuse(col_dif[0], col_dif[1], col_dif[2], 1.0f);
        pcMat->AddProperty (&colorDiffuse, 1, AI_MATKEY_COLOR_DIFFUSE);
        const Vector3f &col_spe = material_->specularColor();
        aiColor4D colorSpecular(col_spe[0], col_spe[1], col_spe[2], 1.0f);
        pcMat->AddProperty (&colorSpecular, 1, AI_MATKEY_COLOR_SPECULAR);
        const Vector3f &col_emi = material_->emissiveColor();
        aiColor4D colorEmissive(col_emi[0], col_emi[1], col_emi[2], 1.0f);
        pcMat->AddProperty (&colorEmissive, 1, AI_MATKEY_COLOR_EMISSIVE);
        float ambi = material_->ambientIntensity();
        aiColor4D colorAmbient(ambi*col_dif[0], ambi*col_dif[1], ambi*col_dif[2], 1.0f);
        pcMat->AddProperty (&colorAmbient, 1, AI_MATKEY_COLOR_AMBIENT);
        float transp = material_->transparency();
        pcMat->AddProperty (&transp, 1, AI_MATKEY_COLOR_TRANSPARENT);

        vec_material.push_back(pcMat);
        pMesh->mMaterialIndex = vec_material.size() - 1;
    }
}
aiScene *AssimpSceneWriter::Impl::createScene()
{
    int num_mesh = vec_mesh.size();
    DEBUG_STREAM(" num_mesh: " << num_mesh);
    if (num_mesh == 0) {
        return nullptr;
    }
    aiScene *pScene = new aiScene();

    // store meshes
    pScene->mNumMeshes = num_mesh;
    pScene->mMeshes = new aiMesh*[num_mesh];
    for(int i = 0; i < num_mesh; i++) {
        pScene->mMeshes[i] = vec_mesh[i];
    }
    // create root node
    pScene->mRootNode = new aiNode();
    pScene->mRootNode->mNumMeshes = num_mesh;
    pScene->mRootNode->mMeshes = new unsigned int[num_mesh];
    for(int i = 0; i < num_mesh; i++) {
        pScene->mRootNode->mMeshes[i] = i;
    }
    pScene->mRootNode->mName.Set("root");

    // material
    int num_mat = vec_material.size();
    if (num_mat == 0) {
        pScene->mNumMaterials = 1;
        pScene->mMaterials = new aiMaterial*[1];
        // make default material
        aiMaterial* pcMat = new aiMaterial();
        aiString s; s.Set (AI_DEFAULT_MATERIAL_NAME);
        pcMat->AddProperty (&s, AI_MATKEY_NAME);
        aiColor4D colorDiffuse(0.8f, 0.8f, 0.8f, 1.0f);
        pcMat->AddProperty (&colorDiffuse, 1, AI_MATKEY_COLOR_DIFFUSE);
        pcMat->AddProperty (&colorDiffuse, 1, AI_MATKEY_COLOR_SPECULAR);
        aiColor4D colorAmbient(0.2f, 0.2f, 0.2f, 1.0f);
        pcMat->AddProperty (&colorAmbient, 1, AI_MATKEY_COLOR_AMBIENT);

        pScene->mMaterials[0] = pcMat;
    } else {
        pScene->mNumMaterials = num_mat;
        pScene->mMaterials = new aiMaterial*[num_mat];
        for(int i = 0; i < num_mat; i++) {
            pScene->mMaterials[i] = vec_material[i];
        }
    }
    return pScene;
}
#define SIZE_T_MAX (std::numeric_limits<size_t>::max())
bool AssimpSceneWriter::Impl::storeScene(aiScene *_pscene, const std::string &_filename, std::string &_id)
{
    std::string outf = _filename;
    std::string outext;
    if(_id.size() > 0) {
        outext = _id;
    } else {
        const std::string::size_type s = outf.find_last_of('.');
        if (s != std::string::npos) {
            outext = outf.substr (s+1);
        } else {
            self->os() << ";; Can not find extention: " << outf << std::endl;
        }
    }
    Assimp::Exporter exporter;
    size_t outf_idx = SIZE_T_MAX;
    for(size_t i = 0, end = exporter.GetExportFormatCount(); i < end; ++i) {
        const aiExportFormatDesc* const e = exporter.GetExportFormatDescription(i);
        DEBUG_STREAM(" e->id = " << e->id << ", e->fileExtension = " << e->fileExtension);
        if (outext == e->id) {
            outf_idx = i; break;
        } else if (outext == e->fileExtension) {
            outf_idx = i; break;
        }
    }
    if (outf_idx == SIZE_T_MAX) { // unknown extention
        outext = "stl";
        for(size_t i = 0, end = exporter.GetExportFormatCount(); i < end; ++i) {
            const aiExportFormatDesc* const e = exporter.GetExportFormatDescription(i);
            if (outext == e->fileExtension) {
                outf_idx = i; break;
            }
        }
    }
    if (outf_idx == SIZE_T_MAX) outf_idx = 0;
    //
    const aiExportFormatDesc* const e = exporter.GetExportFormatDescription(outf_idx);
    aiReturn ret = exporter.Export (_pscene, e->id, outf);

    if (ret == AI_SUCCESS) {
        return true;
    }
    return false;
}
