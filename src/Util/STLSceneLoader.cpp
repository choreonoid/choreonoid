
#include "STLSceneLoader.h"
#include "SceneDrawables.h"
#include "SceneLoader.h"
#include "NullOut.h"
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <fmt/format.h>
#include <fstream>
#include <thread>
#include <iostream>
#include "gettext.h"

using namespace std;
using namespace boost::algorithm;
using fmt::format;
using namespace cnoid;

namespace {

const bool ENABLE_DEBUG = false;

const bool ENABLE_COMPACTION = true;

typedef ifstream::pos_type pos_type;

const size_t STL_BINARY_HEADER_SIZE = 84;

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

class MeshLoader
{
public:
    SgMeshPtr sharedMesh;
    size_t numTriangles;
    SgVertexArrayPtr vertices;
    SgIndexArray* triangleVertices;
    size_t triangleVerticesIndex;
    SgNormalArrayPtr normals;
    SgIndexArray* normalIndices;
    size_t normalIndicesIndex;
    BoundingBoxf bbox;
    thread loaderThread;

    // For the mesh integration
    int vertexArrayOffset;
    int normalArrayOffset;
    size_t indexArrayOffset;

    MeshLoader(size_t numTriangles);
    MeshLoader(const MeshLoader& org);
    void initializeArrays();
    void initializeArrays(size_t triangleOffset, size_t numTriangles);
    template<typename AddIndexFunction>
    void addNormal(const Vector3f& normal, AddIndexFunction addIndex);
    template<typename AddIndexFunction>
    void addVertex(const Vector3f& vertex, AddIndexFunction addIndex);
    void join();
    void initializeIntegration(size_t totalNumVertices, size_t totalNumNormals);
    void integrate(MeshLoader& loader);
    SgMeshPtr completeMesh(bool doShrin);
};

class BinaryMeshLoader : public MeshLoader
{
public:
    BinaryMeshLoader(size_t numTriangles) : MeshLoader(numTriangles) { }
    BinaryMeshLoader(const BinaryMeshLoader& org) : MeshLoader(org) { }
    void addNormal(const Vector3f& normal);
    void addVertex(const Vector3f& vertex);
    void load(const string& filename, size_t triangleOffset, size_t numTriangles);
    void load(ifstream& ifs, size_t triangleOffset, size_t numTriangles);
};

class AsciiMeshLoader : public MeshLoader
{
public:
    AsciiMeshLoader() : MeshLoader(0) { }
    void addNormal(const Vector3f& normal);
    void addVertex(const Vector3f& vertex);
    SgMeshPtr load(const string& filename);
};

}

namespace cnoid {

class STLSceneLoaderImpl
{
public:
    ostream* os_;
    ostream& os() { return *os_; }

    STLSceneLoaderImpl();
    SgNode* load(const string& filename);
    SgMeshPtr loadBinaryFormat(const string& filename, ifstream& ifs, size_t numTriangles);
    SgMeshPtr loadAsciiFormat(const string& filename);
};

}


MeshLoader::MeshLoader(size_t numTriangles)
    : sharedMesh(new SgMesh),
      numTriangles(numTriangles)
{
    triangleVertices = &sharedMesh->triangleVertices();
    normalIndices = &sharedMesh->normalIndices();
    
    sharedMesh->setNumTriangles(numTriangles);
    normalIndices->resize(numTriangles * 3);
}


MeshLoader::MeshLoader(const MeshLoader& org)
    : sharedMesh(org.sharedMesh)
{
    numTriangles = 0;
    triangleVertices = nullptr;
    normalIndices = nullptr;
}


void MeshLoader::initializeArrays()
{
    vertices = new SgVertexArray;
    normals = new SgNormalArray;
    triangleVertices = &sharedMesh->triangleVertices();
    normalIndices = &sharedMesh->normalIndices();
}
    

void MeshLoader::initializeArrays(size_t triangleOffset, size_t numTriangles)
{
    this->numTriangles = numTriangles;
    
    vertices = new SgVertexArray;
    vertices->reserve(numTriangles * 3 / 3);

    normals = new SgNormalArray;
    normals->reserve(numTriangles / 5);
    
    triangleVertices = &sharedMesh->triangleVertices();
    triangleVerticesIndex = triangleOffset * 3;

    normalIndices = &sharedMesh->normalIndices();
    normalIndicesIndex = triangleOffset * 3;
}


template<typename AddIndexFunction>
void MeshLoader::addNormal(const Vector3f& normal, AddIndexFunction addIndex)
{
    static const int SearchLength = 12;

    bool found = false;
    int index = normals->size() - 1;

    if(ENABLE_COMPACTION){
        int minIndex = std::max(0, index - (SearchLength - 1));
        while(index >= minIndex){
            if(normal.isApprox((*normals)[index])){
                found = true;
                break;
            }
            --index;
        }
    }

    if(!found){
        index = normals->size();
        normals->push_back(normal);
    }
    addIndex(index);
}


template<typename AddIndexFunction>
void MeshLoader::addVertex(const Vector3f& vertex, AddIndexFunction addIndex)
{
    static const int SearchLength = 27;

    bool found = false;
    int index = vertices->size() - 1;

    if(ENABLE_COMPACTION){
        int minIndex = std::max(0, index - (SearchLength - 1));
        while(index >= minIndex){
            if(vertex.isApprox((*vertices)[index])){
                found = true;
                break;
            }
            --index;
        }
    }

    if(found){
        addIndex(index);
    } else {
        addIndex(vertices->size());
        vertices->push_back(vertex);
        bbox.expandBy(vertex);
    }
}


void MeshLoader::join()
{
    if(loaderThread.joinable()){
        loaderThread.join();
    }
}


void MeshLoader::initializeIntegration(size_t totalNumVertices, size_t totalNumNormals)
{
    vertices = new SgVertexArray(totalNumVertices);
    normals = new SgNormalArray(totalNumNormals);
       
    vertexArrayOffset = 0;
    normalArrayOffset = 0;
    indexArrayOffset = 0;
}


void MeshLoader::integrate(MeshLoader& loader)
{
    std::copy(loader.vertices->begin(), loader.vertices->end(),
              vertices->begin() + vertexArrayOffset);

    std::copy(loader.normals->begin(), loader.normals->end(),
              normals->begin() + normalArrayOffset);

    const size_t indexArrayEnd = indexArrayOffset + loader.numTriangles * 3;
    
    if(vertexArrayOffset > 0 || normalArrayOffset > 0){
        for(size_t i = indexArrayOffset; i < indexArrayEnd; ++i){
            (*triangleVertices)[i] += vertexArrayOffset;
            (*normalIndices)[i] += normalArrayOffset;
        }
    }
            
    vertexArrayOffset += loader.vertices->size();
    normalArrayOffset += loader.normals->size();
    indexArrayOffset = indexArrayEnd;

    bbox.expandBy(loader.bbox);
}


SgMeshPtr MeshLoader::completeMesh(bool doShrink)
{
    if(vertices->empty()){
        return nullptr;
    }

    sharedMesh->setVertices(vertices);
    sharedMesh->setNormals(normals);
    sharedMesh->setBoundingBox(bbox);

    if(doShrink){
        vertices->shrink_to_fit();
        normals->shrink_to_fit();
        sharedMesh->triangleVertices().shrink_to_fit();
        sharedMesh->normalIndices().shrink_to_fit();
    }

    return sharedMesh;
}


STLSceneLoader::STLSceneLoader()
{
    impl = new STLSceneLoaderImpl;
}


STLSceneLoaderImpl::STLSceneLoaderImpl()
{
    os_ = &nullout();
}


STLSceneLoader::~STLSceneLoader()
{
    delete impl;
}


void STLSceneLoader::setMessageSink(std::ostream& os)
{
    impl->os_ = &os;
}


SgNode* STLSceneLoader::load(const std::string& filename)
{
    return impl->load(filename);
}


SgNode* STLSceneLoaderImpl::load(const string& filename)
{
    ifstream ifs(filename.c_str(), std::ios::in | std::ios::binary);
    if(!ifs.is_open()){
        os() << format(_("Unable to open file \"{}\"."), filename) << endl;
        return nullptr;
    }
    
    ifs.seekg(0, fstream::end);
    pos_type fileSize = ifs.tellg();
    ifs.seekg(0, fstream::beg);

    bool isBinary = false;
    size_t numTriangles = 0;
    uint8_t buf[STL_BINARY_HEADER_SIZE];
    ifs.read((char*)buf, STL_BINARY_HEADER_SIZE);
    if(ifs.gcount() == STL_BINARY_HEADER_SIZE){
        numTriangles = buf[80] + (buf[81] << 8) + (buf[82] << 16) + (buf[83] << 24);
        pos_type expectedSize = numTriangles * 50 + STL_BINARY_HEADER_SIZE;
        if(expectedSize == fileSize){
            isBinary = true;
        }
    }

    SgMeshPtr mesh;
    if(isBinary){
        mesh = loadBinaryFormat(filename, ifs, numTriangles);
    } else {
        mesh = loadAsciiFormat(filename);
    }

    if(!mesh){
        os() << "Empty vertices." << endl;
    }

    auto shape = new SgShape;
    shape->setMesh(mesh);
    
    return shape; 
}


SgMeshPtr STLSceneLoaderImpl::loadBinaryFormat(const string& filename, ifstream& ifs, size_t numTriangles)
{
    BinaryMeshLoader mainLoader(numTriangles);

    const size_t NumTrianglesPerThread = 250000;

    auto maxNumThreads = std::max((unsigned)1, std::min(thread::hardware_concurrency(), (unsigned)6));
    if(ENABLE_DEBUG){
        cout << "maxNumThreads: " << maxNumThreads << endl;
    }
    
    size_t numThreads = std::min(size_t(maxNumThreads), std::max(size_t(1), numTriangles / NumTrianglesPerThread));

    if(ENABLE_DEBUG){
        cout << "numThreads: " << numThreads << endl;
    }

    if(numThreads == 1){
        mainLoader.load(ifs, 0, numTriangles);
        return mainLoader.completeMesh(true);
    }

    vector<BinaryMeshLoader> loaders(numThreads, mainLoader);
    
    int index = 0;
    size_t triangleOffset = 0;
    size_t numTrianglesPerThread = numTriangles / numThreads;
    while(index < numThreads - 1){
        loaders[index].load(filename, triangleOffset, numTrianglesPerThread);
        ++index;
        triangleOffset += numTrianglesPerThread;
    }
    loaders[index].load(ifs, triangleOffset, numTriangles - triangleOffset);

    size_t totalNumVertices = 0;
    size_t totalNumNormals = 0;
    for(auto& loader : loaders){
        loader.join();
        totalNumVertices += loader.vertices->size();
        totalNumNormals += loader.normals->size();
    }

    mainLoader.initializeIntegration(totalNumVertices, totalNumNormals);

    for(auto& loader : loaders){
        mainLoader.integrate(loader);
    }

    return mainLoader.completeMesh(false);
}


void BinaryMeshLoader::addNormal(const Vector3f& normal)
{
    MeshLoader::addNormal(
        normal,
        [&](int index){
            (*normalIndices)[normalIndicesIndex++] = index;
            (*normalIndices)[normalIndicesIndex++] = index;
            (*normalIndices)[normalIndicesIndex++] = index;
        });
}


void BinaryMeshLoader::addVertex(const Vector3f& vertex)
{
    MeshLoader::addVertex(
        vertex,
        [&](int index){
            (*triangleVertices)[triangleVerticesIndex++] = index;
        });
}


void BinaryMeshLoader::load(const string& filename, size_t triangleOffset, size_t numTriangles)
{
    loaderThread = thread(
        [this,filename,triangleOffset,numTriangles](){
            ifstream ifs(filename.c_str(), std::ios::in | std::ios::binary);
            load(ifs, triangleOffset, numTriangles);
        });
}


void BinaryMeshLoader::load(ifstream& ifs, size_t triangleOffset, size_t numTriangles)
{
    ifs.seekg(STL_BINARY_HEADER_SIZE + triangleOffset * 50);

    initializeArrays(triangleOffset, numTriangles);
    
    for(size_t i = 0; i < numTriangles; ++i){
        Vector3f normal;
        for(size_t j = 0; j < 3; ++j){
            ifs.read((char*)&normal[j], 4);
        }
        addNormal(normal);

        for(size_t j = 0; j < 3; ++j){
            Vector3f vertex;
            for(size_t k = 0; k < 3; ++k){
                ifs.read((char*)&vertex[k], 4);
            }
            addVertex(vertex);
        }
        uint16_t attrib;
        ifs.read((char *)&attrib, 2);
    }
}


SgMeshPtr STLSceneLoaderImpl::loadAsciiFormat(const string& filename)
{
    AsciiMeshLoader loader;
    return loader.load(filename);
}


void AsciiMeshLoader::addNormal(const Vector3f& normal)
{
    MeshLoader::addNormal(
        normal,
        [&](int index){
            normalIndices->push_back(index);
            normalIndices->push_back(index);
            normalIndices->push_back(index);
        });
}


void AsciiMeshLoader::addVertex(const Vector3f& vertex)
{
    MeshLoader::addVertex(
        vertex,
        [&](int index){
            triangleVertices->push_back(index);
        });
}


SgMeshPtr AsciiMeshLoader::load(const string& filename)
{
    initializeArrays();
    
    ifstream ifs(filename.c_str(), std::ios::in);
    string line;
    while(!ifs.eof() && getline(ifs, line)){
        trim(line);
        if(boost::istarts_with(line, "vertex")){
            addVertex(readVector3(line.substr(6)));
        } else if(boost::istarts_with(line, "facet normal")){
            addNormal(readVector3(line.substr(12)));
        }
    }

    return completeMesh(true);
}
