
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

struct Registration {
    Registration(){
        SceneLoader::registerLoader(
            "stl",
            []() -> shared_ptr<AbstractSceneLoader> { return make_shared<STLSceneLoader>(); });
    }
} registration;

}

static void readVector3(string text, SgVectorArray<Vector3f>* array)
{
    trim(text);
    Vector3f value;
    int i = 0;
    split_iterator<string::iterator> iter = make_split_iterator(text, token_finder(is_space(), token_compress_on));
    while(iter != split_iterator<string::iterator>()){
        value[i++] = boost::lexical_cast<float>(*iter++);
        if(i == 3){
            array->push_back(value);
            break;
        }
    }
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

    SgVertexArrayPtr vertices = new SgVertexArray;
    SgNormalArrayPtr normals = new SgNormalArray;

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
        for(size_t i = 0; i < numFaces; i++){
            Vector3f value;
            for(size_t j = 0; j < 3; j++){
                float v;
                ifs.read((char *)&v, 4);
                value[j] = v;
            }
            normals->push_back(value);
            for(size_t k = 0; k < 3; k++){
                Vector3f value;
                for(size_t j = 0; j < 3; j++){
                    float v;
                    ifs.read((char *)&v, 4);
                    value[j] = v;
                }
                vertices->push_back(value);
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
                readVector3(line.substr(6), vertices);
            } else if(boost::istarts_with(line, "facet normal")){
                readVector3(line.substr(12), normals);
            }
        }
    }
    
    SgShape* shape = 0;
    
    if(vertices->empty()){
        os() << "Empty vertices." << endl;
            
    } else {
        shape = new SgShape;
        SgMesh* mesh = shape->getOrCreateMesh();
        mesh->setVertices(vertices);
        const int numTriangles = vertices->size() / 3;
        mesh->reserveNumTriangles(numTriangles);
        for(int i = 0; i < numTriangles; ++i){
            const int j = i * 3;
            mesh->addTriangle(j, j + 1, j + 2);
        }
        if(!normals->empty()){
            mesh->setNormals(normals);
            SgIndexArray& indices = mesh->normalIndices();
            indices.reserve(normals->size() * 3);
            for(size_t i = 0; i < normals->size(); ++i) {
                indices.push_back(i);
                indices.push_back(i);
                indices.push_back(i);
            }
        }
        mesh->updateBoundingBox();
    }
    
    return shape; 
}
