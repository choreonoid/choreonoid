#include "ObjSceneWriter.h"
#include "SceneGraph.h"
#include "SceneDrawables.h"
#include "PolymorphicSceneNodeFunctionSet.h"
#include "UTF8.h"
#include "NullOut.h"
#include <cnoid/stdx/filesystem>
#include <fmt/format.h>
#include <fstream>
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
    std::ofstream ofs;
    filesystem::path directory;
    int objectIdCounter;
    int vertexIndexOffset;
    int normalIndexOffset;

    ostream* os_;
    ostream& os() { return *os_; }

    Impl(ObjSceneWriter* self);
    bool writeScene(const std::string& filename, SgNode* node);
    void writeNode(SgNode* node, const Affine3& T_parent);
    void writeShape(SgShape* shape, const Affine3& T);
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
    os_ = &nullout();    
}


ObjSceneWriter::~ObjSceneWriter()
{
    delete impl;
}


void ObjSceneWriter::setMessageSink(std::ostream& os)
{
    impl->os_ = &os;
}


bool ObjSceneWriter::writeScene(const std::string& filename, SgNode* node)
{
    return impl->writeScene(filename, node);
}


bool ObjSceneWriter::Impl::writeScene(const std::string& filename, SgNode* node)
{
    try {
        ofs.open(fromUTF8(filename).c_str(), ios_base::out | ios_base::binary);
        ofs.exceptions(std::ios_base::failbit);
    }
    catch(const std::exception& ex){
        os() << format(_("\"{0}\" cannot be open. {1}."), filename, ex.what()) << endl;
        return false;
    }

    directory = filesystem::path(filename).parent_path();
    objectIdCounter = 0;
    vertexIndexOffset = 1;
    normalIndexOffset = 1;

    writeNode(node, Affine3::Identity());

    ofs.close();
    
    return true;
}


void ObjSceneWriter::Impl::writeNode(SgNode* node, const Affine3& T_parent)
{
    if(node->isGroupNode()){
        auto group = node->toGroupNode();
        Affine3 T = T_parent;
        if(auto transform = group->toTransformNode()){
            Affine3 T0;
            transform->getTransform(T0);
            T = T * T0;
        }
        for(auto& child : *group){
            writeNode(child, T);
        }
    } else if(auto shape = dynamic_cast<SgShape*>(node)){
        writeShape(shape, T_parent);
    }
}


void ObjSceneWriter::Impl::writeShape(SgShape* shape, const Affine3& T)
{
    auto mesh = shape->mesh();
    if(mesh->hasVertices() && mesh->numTriangles() > 0){
        writeMesh(mesh, T);
    }
}


void ObjSceneWriter::Impl::writeMesh(SgMesh* mesh, const Affine3& T)
{
    ofs << "g " << objectIdCounter++ << "\n";

    const auto vertices = mesh->vertices();
    for(auto& v : *vertices){
        Vector3f vt = (T * v.cast<double>()).cast<float>();
        ofs << "v " << vt.x() << " " << vt.y() << " " << vt.z() << "\n";
    }

    bool hasValidNormals = false;
    const auto normals = mesh->normals();
    if(mesh->hasNormals()){
        if(!mesh->hasNormalIndices()){
            if(normals->size() == vertices->size()){
                hasValidNormals = true;
            }
        } else {
            if(mesh->normalIndices().size() == mesh->triangleVertices().size()){
                hasValidNormals = true;
            }
        }
    }
    if(hasValidNormals){
        Matrix3 R = T.linear();
        for(auto& n : *normals){
            Vector3f nt = (R * n.cast<double>()).cast<float>();
            ofs << "vn " << nt.x() << " " << nt.y() << " " << nt.z() << "\n";
        }
    }

    const int numTriangles = mesh->numTriangles();

    const int vio = vertexIndexOffset;
    const int nio = normalIndexOffset;
    
    if(!hasValidNormals){
        for(int i=0; i < numTriangles; ++i){
            auto tri = mesh->triangle(i);
            ofs << "f " << (tri[0] + vio)
                << " "  << (tri[1] + vio)
                << " "  << (tri[2] + vio) << "\n";
        }
    } else {
        const auto& normalIndices = mesh->normalIndices();
        if(normalIndices.empty()){
            for(int i=0; i < numTriangles; ++i){
                auto tri = mesh->triangle(i);
                ofs << "f ";
                ofs << (tri[0] + vio) << "//" << (tri[0] + nio) << " ";
                ofs << (tri[1] + vio) << "//" << (tri[1] + nio) << " ";
                ofs << (tri[2] + vio) << "//" << (tri[2] + nio) << "\n";
            }
        } else {
            for(int i=0; i < numTriangles; ++i){
                auto tri = mesh->triangle(i);
                ofs << "f ";
                ofs << (tri[0] + vio) << "//" << (normalIndices[i * 3]     + nio) << " ";
                ofs << (tri[1] + vio) << "//" << (normalIndices[i * 3 + 1] + nio) << " ";
                ofs << (tri[2] + vio) << "//" << (normalIndices[i * 3 + 2] + nio) << "\n";
            }
        }
    }

    vertexIndexOffset += vertices->size();
    if(hasValidNormals){
        normalIndexOffset += normals->size();
    }
}
