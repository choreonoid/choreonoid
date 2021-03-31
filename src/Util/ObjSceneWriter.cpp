#include "ObjSceneWriter.h"
#include "SceneGraph.h"
#include "SceneDrawables.h"
#include "PolymorphicSceneNodeFunctionSet.h"
#include "IdPair.h"
#include "UTF8.h"
#include "NullOut.h"
#include <cnoid/stdx/filesystem>
#include <fmt/format.h>
#include <fstream>
#include <map>
#include <set>
#include <exception>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;
namespace filesystem = cnoid::stdx::filesystem;

namespace cnoid {

class ObjSceneWriter::Impl
{
public:
    ObjSceneWriter* self;

    // File stream to output the main geometry part
    std::ofstream gfs;
    // File stream to output the material template library
    std::ofstream mfs;

    vector<SgObject*> namedObjectStack;
    map<string, int> objectNameCounterMap;
    int objectIdCounter;
    
    int vertexIndexOffset;
    int normalIndexOffset;
    int texCoordIndexOffset;

    bool isAppearanceEnabled;
    typedef IdPair<SgObject*> MaterialPair;
    map<MaterialPair, string> materialLabelMap;
    set<string> materialLabelSet;
    MaterialPair lastMaterialPair;
    int materialLabelIdCounter;

    filesystem::path filepath;
    filesystem::path baseDirPath;

    ostream* os_;
    ostream& os() { return *os_; }

    Impl(ObjSceneWriter* self);
    void clear();
    bool writeScene(const std::string& filename, SgNode* node);
    bool writeNode(SgNode* node, const Affine3& T_parent);
    bool writeShape(SgShape* shape, const Affine3& T);
    bool findOrWriteMaterial(SgShape* shape);
    void writeMaterial(SgMaterial* material, SgTexture* texture, string& out_label);
    bool writeMaterial(SgShape* shape);
    void writeMesh(SgMesh* mesh, const Affine3& T);
};

}

ObjSceneWriter::ObjSceneWriter()
{
    impl = new Impl(this);
}


ObjSceneWriter::Impl::Impl(ObjSceneWriter* self)
    : self(self)
{
    isAppearanceEnabled = true;
    os_ = &nullout();    
}


ObjSceneWriter::~ObjSceneWriter()
{
    delete impl;
}


void ObjSceneWriter::setMessageSink(std::ostream& os)
{
    AbstractSceneWriter::setMessageSink(os);
    impl->os_ = &os;
}


void ObjSceneWriter::setMaterialEnabled(bool on)
{
    impl->isAppearanceEnabled = on;
}


bool ObjSceneWriter::isMaterialEnabled() const
{
    return impl->isAppearanceEnabled;
}


void ObjSceneWriter::Impl::clear()
{
    namedObjectStack.clear();
    objectNameCounterMap.clear();
    objectIdCounter = 0;
    vertexIndexOffset = 1;
    normalIndexOffset = 1;
    texCoordIndexOffset = 1;

    materialLabelMap.clear();
    materialLabelSet.clear();
    lastMaterialPair = MaterialPair(nullptr, nullptr);
    materialLabelIdCounter = 0;
}    


bool ObjSceneWriter::writeScene(const std::string& filename, SgNode* node)
{
    return impl->writeScene(filename, node);
}


bool ObjSceneWriter::Impl::writeScene(const std::string& filename, SgNode* node)
{
    filepath = filesystem::path(filename);
    baseDirPath = filesystem::absolute(filepath).parent_path();
    
    try {
        gfs.open(fromUTF8(filename).c_str(), ios_base::out | ios_base::binary);
        gfs.exceptions(std::ios_base::failbit);
    }
    catch(const std::exception& ex){
        os() << format(_("\"{0}\" cannot be open. {1}."), filename, ex.what()) << endl;
        return false;
    }

    clear();

    bool result = writeNode(node, Affine3::Identity());

    if(!result){
        os() << format(_("Failed to write a scene into \"{0}\"."), filename) << endl;
    }

    gfs.close();

    if(mfs.is_open()){
        mfs.close();
    }

    clear();
    
    return result;
}


bool ObjSceneWriter::Impl::writeNode(SgNode* node, const Affine3& T_parent)
{
    if(!node->isGroupNode()){
        if(auto shape = dynamic_cast<SgShape*>(node)){
            if(!shape->name().empty()){
                namedObjectStack.push_back(shape);
            }
            bool result = writeShape(shape, T_parent);
            if(!shape->name().empty()){
                namedObjectStack.pop_back();
            }
            if(!result){
                return false;
            }
        }
    } else {
        auto group = node->toGroupNode();
        Affine3 T = T_parent;
        if(auto transform = group->toTransformNode()){
            Affine3 T0;
            transform->getTransform(T0);
            T = T * T0;
        }
        if(!group->name().empty()){
            namedObjectStack.push_back(group);
        }
        for(auto& child : *group){
            writeNode(child, T);
        }
        if(!group->name().empty()){
            namedObjectStack.pop_back();
        }
    }
    return true;
}


bool ObjSceneWriter::Impl::writeShape(SgShape* shape, const Affine3& T)
{
    if(isAppearanceEnabled){
        if(!findOrWriteMaterial(shape)){
            return false;
        }
    }
    auto mesh = shape->mesh();
    if(mesh->hasVertices() && mesh->numTriangles() > 0){
        writeMesh(mesh, T);
    }
    return true;
}


bool ObjSceneWriter::Impl::findOrWriteMaterial(SgShape* shape)
{
    auto material = shape->material();
    auto texture = shape->texture();
    MaterialPair materialPair(material, texture);

    if(materialPair == lastMaterialPair){
        return true;
    }
    
    if(!mfs.is_open()){
        string mtlFilename;
        try {
            auto mtlFilepath = filepath;
            mtlFilename = mtlFilepath.replace_extension(".mtl").string();
            mfs.open(fromUTF8(mtlFilename).c_str(), ios_base::out | ios_base::binary);
            mfs.exceptions(std::ios_base::failbit);

            gfs << "mtllib " << mtlFilepath.filename().generic_string() << "\n";
        }
        catch(const std::exception& ex){
            os() << format(_("\"{0}\" cannot be open. {1}."), mtlFilename, ex.what()) << endl;
            return false;
        }
    }

    auto& label = materialLabelMap[materialPair];
    if(label.empty()){
        writeMaterial(material, texture, label);
    }

    gfs << "usemtl " << label << "\n";

    return true;
}


void ObjSceneWriter::Impl::writeMaterial(SgMaterial* material, SgTexture* texture, string& out_label)
{
    string name;
    if(material){
        name = material->name();
    }
    if(!name.empty()){
        auto inserted = materialLabelSet.insert(name);
        if(inserted.second){
            out_label = name;
        }
    }
    if(out_label.empty()){
        int idCounter = 0;
        int* pLabelIdCounter;

        if(name.empty()){
            name = "Material";
            pLabelIdCounter = &materialLabelIdCounter;
        } else {
            pLabelIdCounter = &idCounter;
        }
        string label;
        while(true){
            label = format("{0}_{1:03d}", name, (*pLabelIdCounter)++);
            auto inserted = materialLabelSet.insert(label);
            if(inserted.second){
                out_label = label;
                break;
            }
        }
    }
    
    mfs << "newmtl " << out_label << "\n";

    if(material){
        float Ka = material->ambientIntensity();
        mfs << "Ka " << Ka << " " << Ka << " " << Ka << "\n";
        auto& Kd = material->diffuseColor();
        mfs << "Kd " << Kd[0] << " " << Kd[1] << " " << Kd[2] << "\n";
        auto& Ks = material->specularColor();
        mfs << "Ks " << Ks[0] << " " << Ks[1] << " " << Ks[2] << "\n";
        mfs << "Ns " << material->specularExponent() << "\n";
        auto& Ke = material->emissiveColor();
        mfs << "Ke " << Ke[0] << " " << Ke[1] << " " << Ke[2] << "\n";
        mfs << "d " << (1.0f - material->transparency()) << "\n";
        mfs << "Tr " << material->transparency() << "\n";
        mfs << "illum 2\n";
    }

    if(texture){
        auto image = texture->image();
        if(image && image->hasUri()){
            if(self->findOrCopyImageFile(image, baseDirPath.generic_string())){
                mfs << "map_Kd " << image->uri() << "\n";
            }
        }
    }

    mfs << "\n";
}


void ObjSceneWriter::Impl::writeMesh(SgMesh* mesh, const Affine3& T)
{
    string name;
    if(!mesh->name().empty()){
        name = mesh->name();
    } else if(!namedObjectStack.empty()){
        name = namedObjectStack.back()->name();
        auto inserted = objectNameCounterMap.emplace(name, 1);
        if(!inserted.second){
            auto& counter = inserted.first->second;
            string nameWithId;
            while(true){
                nameWithId = format("{0}_{1}", name, counter++);
                if(objectNameCounterMap.find(nameWithId) == objectNameCounterMap.end()){
                    name = nameWithId;
                    break;
                }
            }
        }
    }
    if(!name.empty()){
        gfs << "g " << name << "\n";
    } else {
        gfs << "g " << objectIdCounter++ << "\n";
    }

    const auto vertices = mesh->vertices();
    for(auto& v : *vertices){
        Vector3f vt = (T * v.cast<double>()).cast<float>();
        gfs << "v " << vt.x() << " " << vt.y() << " " << vt.z() << "\n";
    }

    const auto& vertexIndices = mesh->faceVertexIndices();

    bool hasValidNormals = false;
    const auto normals = mesh->normals();
    const auto& normalIndices = mesh->normalIndices();
    if(mesh->hasNormals()){
        if(!mesh->hasNormalIndices()){
            if(normals->size() == vertices->size()){
                hasValidNormals = true;
            }
        } else {
            if(normalIndices.size() == vertexIndices.size()){
                hasValidNormals = true;
            }
        }
    }
    if(hasValidNormals){
        Matrix3 R = T.linear();
        for(auto& n : *normals){
            Vector3f vn = (R * n.cast<double>()).cast<float>();
            gfs << "vn " << vn.x() << " " << vn.y() << " " << vn.z() << "\n";
        }
    }

    bool hasValidTexCoords = false;
    const auto texCoords = mesh->texCoords();
    const auto& texCoordIndices = mesh->texCoordIndices();
    if(mesh->hasTexCoords()){
        if(!mesh->hasTexCoordIndices()){
            if(texCoords->size() == vertices->size()){
                hasValidTexCoords = true;
            }
        } else {
            if(texCoordIndices.size() == vertexIndices.size()){
                hasValidTexCoords = true;
            }
        }
    }
    if(hasValidTexCoords){
        for(auto& vt : *texCoords){
            gfs << "vt " << vt.x() << " " << vt.y() << " 0\n";
        }
    }

    const int numTriangles = mesh->numTriangles();
    for(int i=0; i < numTriangles; ++i){
        gfs << "f ";
        for(int j=0; j < 3; ++j){
            int localIndex = i * 3 + j;
            int vertexIndex = vertexIndices[localIndex];
            gfs << (vertexIndex + vertexIndexOffset) << "/";
            if(hasValidTexCoords){
                if(texCoordIndices.empty()){
                    gfs << (vertexIndex + texCoordIndexOffset);
                } else {
                    gfs << (texCoordIndices[localIndex] + texCoordIndexOffset);
                }
            }
            gfs << "/";
            if(hasValidNormals){
                if(normalIndices.empty()){
                    gfs << (vertexIndex + normalIndexOffset);
                } else {
                    gfs << (normalIndices[localIndex] + normalIndexOffset);
                }
            }
            if(j == 2){
                gfs << "\n";
            } else {
                gfs << " ";
            }
        }
    }
                    
    vertexIndexOffset += vertices->size();
    if(hasValidNormals){
        normalIndexOffset += normals->size();
    }
    if(hasValidTexCoords){
        texCoordIndexOffset += texCoords->size();
    }
}
