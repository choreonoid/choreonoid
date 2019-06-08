
#include "STLSceneLoader.h"
#include "SceneDrawables.h"
#include "SceneLoader.h"
#include "NullOut.h"
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <fmt/format.h>
#include <fstream>
#include "gettext.h"

using namespace std;
using namespace boost::algorithm;
using fmt::format;
using namespace cnoid;

namespace {

static const bool ENABLE_COMPACTION = true;

struct Registration {
    Registration(){
        SceneLoader::registerLoader(
            "stl",
            []() -> shared_ptr<AbstractSceneLoader> { return make_shared<STLSceneLoader>(); });
    }
} registration;


Vector3f readVector3(string text)
{
    trim(text);
    Vector3f value;
    int i = 0;
    split_iterator<string::iterator> iter = make_split_iterator(text, token_finder(is_space(), token_compress_on));
    while(iter != split_iterator<string::iterator>()){
        value[i++] = boost::lexical_cast<float>(*iter++);
        if(i == 3){
            break;
        }
    }
    return value;
}

class CompactMeshArranger
{
public:
    SgMeshPtr mesh;
    SgVertexArray& vertices;
    SgNormalArray& normals;
    SgIndexArray& triangleVertices;
    SgIndexArray& normalIndices;
    BoundingBoxf bbox;

    CompactMeshArranger();
    void addVertex(const Vector3f& vertex);
    void addNormal(const Vector3f& normal);
    SgShape* finalize();
};

}


CompactMeshArranger::CompactMeshArranger()
    : mesh(new SgMesh),
      vertices(*mesh->getOrCreateVertices()),
      normals(*mesh->getOrCreateNormals()),
      triangleVertices(mesh->triangleVertices()),
      normalIndices(mesh->normalIndices())
{

}


void CompactMeshArranger::addVertex(const Vector3f& vertex)
{
    static const int SearchLength = 27;

    bool found = false;
    int index = vertices.size() - 1;

    if(ENABLE_COMPACTION){
        int minIndex = std::max(0, index - (SearchLength - 1));
        while(index >= minIndex){
            if(vertex.isApprox(vertices[index])){
                found = true;
                break;
            }
            --index;
        }
    }

    if(found){
        triangleVertices.push_back(index);
    } else {
        triangleVertices.push_back(vertices.size());
        vertices.push_back(vertex);
        bbox.expandBy(vertex);
    }
}


void CompactMeshArranger::addNormal(const Vector3f& normal)
{
    static const int SearchLength = 12;

    bool found = false;
    int index = normals.size() - 1;

    if(ENABLE_COMPACTION){
        int minIndex = std::max(0, index - (SearchLength - 1));
        while(index >= minIndex){
            if(normal.isApprox(normals[index])){
                found = true;
                break;
            }
            --index;
        }
    }

    if(!found){
        index = normals.size();
        normals.push_back(normal);
    }        
    normalIndices.push_back(index);
    normalIndices.push_back(index);
    normalIndices.push_back(index);
}


SgShape* CompactMeshArranger::finalize()
{
    if(vertices.empty()){
        return nullptr;
    }

    auto shape = new SgShape;
    shape->setMesh(mesh);
    vertices.shrink_to_fit();
    normals.shrink_to_fit();
    triangleVertices.shrink_to_fit();
    normalIndices.shrink_to_fit();
    mesh->setBoundingBox(bbox);

    return shape;
}


STLSceneLoader::STLSceneLoader()
{
    os_ = &nullout();
}


void STLSceneLoader::setMessageSink(std::ostream& os)
{
    os_ = &os;
}


SgNode* STLSceneLoader::load(const std::string& filename)
{
    std::ifstream ifs(filename.c_str(), std::ios::in | std::ios::binary);
    if(!ifs.is_open()){
        os() << format(_("Unable to open file \"{}\"."), filename) << endl;
        return 0;
    }
    
    ifs.seekg(0, fstream::end);
    unsigned int fileSize = ifs.tellg();
    ifs.seekg(0, fstream::beg);

    CompactMeshArranger arranger;

    bool isBinary = false;
    unsigned int numFaces = 0;
    uint8_t buf[84];
    ifs.read((char*)buf, 84);
    if(ifs.gcount() == 84){
        numFaces = buf[80] + (buf[81] << 8) + (buf[82] << 16) + (buf[83] << 24);
        unsigned int expectedSize = numFaces * 50 + 84;
        if(expectedSize == fileSize){
            isBinary = true;
        }
    }

    if(isBinary){
        for(size_t i = 0; i < numFaces; ++i){
            Vector3f normal;
            for(size_t j = 0; j < 3; ++j){
                ifs.read((char*)&normal[j], 4);
            }
            arranger.addNormal(normal);

            for(size_t j = 0; j < 3; ++j){
                Vector3f vertex;
                for(size_t k = 0; k < 3; ++k){
                    ifs.read((char*)&vertex[k], 4);
                }
                arranger.addVertex(vertex);
            }
            uint16_t attrib;
            ifs.read((char *)&attrib, 2);
        }
    } else {
        // text format
        std::ifstream ifs(filename.c_str(), std::ios::in);
        std::string line;
        while(!ifs.eof() && getline(ifs, line)){
            trim(line);
            if(boost::istarts_with(line, "vertex")){
                arranger.addVertex(readVector3(line.substr(6)));
            } else if(boost::istarts_with(line, "facet normal")){
                arranger.addNormal(readVector3(line.substr(12)));
            }
        }
    }

    auto shape = arranger.finalize();
    
    if(!shape){
        os() << "Empty vertices." << endl;
    }
    
    return shape; 
}
