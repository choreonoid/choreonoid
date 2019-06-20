#include "STLSceneLoader.h"
#include "SceneDrawables.h"
#include "SceneLoader.h"
#include "NullOut.h"
#include "FileUtil.h"
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <fmt/format.h>
#include <fstream>
#include <thread>
#include <stdexcept>
#include <cstdlib>
#include "gettext.h"

using namespace std;
using namespace boost::algorithm;
using fmt::format;
using namespace cnoid;

namespace {

const bool ENABLE_COMPACTION = true;
const int VertexSearchLength = 27;
const int NormalSearchLength = 15;
const size_t NumTrianglesPerThread = 100000;

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
    size_t triangleOffset;
    size_t numTriangles;
    SgVertexArrayPtr vertices;
    SgIndexArray* triangleVertices;
    SgNormalArrayPtr normals;
    SgIndexArray* normalIndices;
    BoundingBoxf bbox;
    thread loaderThread;

    // For the mesh integration
    int vertexArrayOffset;
    int numActualVertices;
    int numMergedVertices;
    vector<int> mergedVertexIndexMap;

    int normalArrayOffset;
    int numActualNormals;
    int numMergedNormals;
    vector<int> mergedNormalIndexMap;

    MeshLoader(size_t numTriangles);
    MeshLoader(const MeshLoader& org);
    void initializeArrays();
    template<typename AddIndexFunction>
    void addNormal(const Vector3f& normal, AddIndexFunction addIndex);
    template<typename AddIndexFunction>
    void addVertex(const Vector3f& vertex, AddIndexFunction addIndex);
    void join();
    int findElement(
        const Vector3f& element, const SgVectorArray<Vector3f>& prevElements, int searchLength);
    void findRedundantElementsBetweenLoaders(
        SgVectorArray<Vector3f>& elements, SgVectorArray<Vector3f>& prevElements,
        vector<int>& mergedElementIndexMap, int& numMergedElements, int searchLength);
    void initializeIntegration(MeshLoader* prevLoader);
    void integrateElements(
        SgVectorArray<Vector3f>& elements, SgVectorArray<Vector3f>& orgElements, int elementArrayOffset,
        SgIndexArray& elementIndices, vector<int>& mergedIndexMap, int numMergedElements, int searchLength);
    void integrateTo(MeshLoader& mainLoader);
    void integrateConcurrentlyTo(MeshLoader& mainLoader);
    
    SgMeshPtr completeMesh(bool doShrin);
};

class BinaryMeshLoader : public MeshLoader
{
public:
    size_t triangleVerticesIndex;
    size_t normalIndicesIndex;
    
    BinaryMeshLoader(size_t numTriangles) : MeshLoader(numTriangles) { }
    BinaryMeshLoader(const BinaryMeshLoader& org) : MeshLoader(org) { }
    void initializeArrays(size_t triangleOffset, size_t numTriangles);
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
    triangleOffset = 0;

    triangleVertices = &sharedMesh->triangleVertices();
    normalIndices = &sharedMesh->normalIndices();
    
    sharedMesh->setNumTriangles(numTriangles);
    normalIndices->resize(numTriangles * 3);
}


MeshLoader::MeshLoader(const MeshLoader& org)
    : sharedMesh(org.sharedMesh)
{
    triangleOffset = 0;
    numTriangles = 0;
    triangleVertices = nullptr;
    normalIndices = nullptr;
}


void MeshLoader::initializeArrays()
{
    vertices = new SgVertexArray;
    normals = new SgNormalArray;
}
    

template<typename AddIndexFunction>
void MeshLoader::addNormal(const Vector3f& normal, AddIndexFunction addIndex)
{
    bool found = false;
    int index = normals->size() - 1;

    if(ENABLE_COMPACTION){
        int minIndex = std::max(0, index - (NormalSearchLength - 1));
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
    bool found = false;
    int index = vertices->size() - 1;

    if(ENABLE_COMPACTION){
        int minIndex = std::max(0, index - (VertexSearchLength - 1));
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


int MeshLoader::findElement
(const Vector3f& element, const SgVectorArray<Vector3f>& prevElements, int searchLength)
{
    bool found = false;
    int index = prevElements.size() - 1;
    int minIndex = std::max(0, index - (searchLength - 1));
    while(index >= minIndex){
        if(element.isApprox(prevElements[index])){
            found = true;
            break;
        }
        --index;
    }
    return found ? (index - prevElements.size()) : 0;
}


void MeshLoader::findRedundantElementsBetweenLoaders
(SgVectorArray<Vector3f>& elements, SgVectorArray<Vector3f>& prevElements,
 vector<int>& mergedElementIndexMap, int& numMergedElements, int searchLength)
{
    mergedElementIndexMap.clear();
    int index = 0;
    while(searchLength > 0){
        const auto& element = elements[index];
        int foundIndex = findElement(element, prevElements, searchLength);
        if(foundIndex < 0){
            mergedElementIndexMap.push_back(foundIndex);
            ++numMergedElements;
        } else {
            mergedElementIndexMap.push_back(index - numMergedElements);
            --searchLength;
        }
        ++index;
    }
}


void MeshLoader::initializeIntegration(MeshLoader* prevLoader)
{
    vertexArrayOffset = 0;
    numActualVertices = vertices->size();
    numMergedVertices = 0;
 
    normalArrayOffset = 0;
    numActualNormals = normals->size();
    numMergedNormals = 0;

    if(prevLoader){
        if(ENABLE_COMPACTION){
            findRedundantElementsBetweenLoaders(
                *vertices, *prevLoader->vertices,
                mergedVertexIndexMap, numMergedVertices, VertexSearchLength);

            numActualVertices -= numMergedVertices;

            findRedundantElementsBetweenLoaders(
                *normals, *prevLoader->normals,
                mergedNormalIndexMap, numMergedNormals, NormalSearchLength);

            numActualNormals -= numMergedNormals;
        }
        vertexArrayOffset = prevLoader->vertexArrayOffset + prevLoader->numActualVertices;
        normalArrayOffset = prevLoader->normalArrayOffset + prevLoader->numActualNormals;
    }
}


void MeshLoader::integrateElements
(SgVectorArray<Vector3f>& elements, SgVectorArray<Vector3f>& orgElements, int elementArrayOffset,
 SgIndexArray& elementIndices, vector<int>& mergedIndexMap, int numMergedElements, int searchLength)
{
    auto newElementIter = elements.begin() + elementArrayOffset;

    for(size_t i=0; i < mergedIndexMap.size(); ++i){
        auto mappedIndex = mergedIndexMap[i];
        if(mappedIndex >= 0){
            *newElementIter++ = orgElements[i];
        }
        mergedIndexMap[i] += elementArrayOffset; // Conver to the global index
    }
        
    std::copy(orgElements.begin() + mergedIndexMap.size(), orgElements.end(), newElementIter);

    const int indexArrayOffset = triangleOffset * 3;
    const int end = indexArrayOffset + numTriangles * 3;
    const int unmappedElementIndexOffset = elementArrayOffset - numMergedElements;
    int pos = indexArrayOffset;
    while(pos < end){
        auto& index = elementIndices[pos++];
        if(index < mergedIndexMap.size()){
            index = mergedIndexMap[index];
        } else {
            index += unmappedElementIndexOffset;
        }
    }
 }


void MeshLoader::integrateTo(MeshLoader& mainLoader)
{
    integrateElements(
        *mainLoader.vertices, *vertices, vertexArrayOffset,
        *triangleVertices, mergedVertexIndexMap, numMergedVertices, VertexSearchLength);

    integrateElements(
        *mainLoader.normals, *normals, normalArrayOffset,
        *normalIndices, mergedNormalIndexMap, numMergedNormals, NormalSearchLength);
}


void MeshLoader::integrateConcurrentlyTo(MeshLoader& mainLoader)
{
    loaderThread = thread([this, &mainLoader](){ integrateTo(mainLoader); });
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
        return nullptr;
    }

    auto shape = new SgShape;
    shape->setMesh(mesh);
    return shape; 
}


SgMeshPtr STLSceneLoaderImpl::loadBinaryFormat(const string& filename, ifstream& ifs, size_t numTriangles)
{
    if(numTriangles == 0){
        os() << format(_("No triangles in \"{1}\"."), filename) << endl;
        return nullptr;
    }
    if(numTriangles > (1 << 31) / 3){
        os() << format(_("Unable to load \"{1}\". Its file size is too large."), filename) << endl;
        return nullptr;
    }
    
    BinaryMeshLoader mainLoader(numTriangles);

    auto maxNumThreads = std::max((unsigned)1, std::min(thread::hardware_concurrency(), (unsigned)6));
    size_t numThreads = std::min(size_t(maxNumThreads), std::max(size_t(1), numTriangles / NumTrianglesPerThread));

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

    for(auto& loader : loaders){
        loader.join();
    }    

    size_t totalNumVertices = 0;
    size_t totalNumNormals = 0;
    BinaryMeshLoader* prevLoader = nullptr;
    for(auto& loader : loaders){
        loader.initializeIntegration(prevLoader);
        totalNumVertices += loader.numActualVertices;
        totalNumNormals += loader.numActualNormals;
        mainLoader.bbox.expandBy(loader.bbox);
        prevLoader = &loader;
    }

    mainLoader.vertices = new SgVertexArray(totalNumVertices);
    mainLoader.normals = new SgNormalArray(totalNumNormals);

    for(int i=0; i < loaders.size() - 1; ++i){
        loaders[i].integrateConcurrentlyTo(mainLoader);
    }
    loaders.back().integrateTo(mainLoader);
            
    for(auto& loader : loaders){
        loader.join();
    }

    return mainLoader.completeMesh(false);
}


void BinaryMeshLoader::initializeArrays(size_t triangleOffset, size_t numTriangles)
{
    MeshLoader::initializeArrays();
    
    this->triangleOffset = triangleOffset;
    this->numTriangles = numTriangles;
    
    vertices->reserve(ENABLE_COMPACTION ? (numTriangles * 3 * 0.4) : (numTriangles * 3));
    triangleVertices = &sharedMesh->triangleVertices();
    triangleVerticesIndex = triangleOffset * 3;

    normals->reserve(ENABLE_COMPACTION ? (numTriangles * 0.25) : numTriangles);
    normalIndices = &sharedMesh->normalIndices();
    normalIndicesIndex = triangleOffset * 3;
}


void BinaryMeshLoader::addNormal(const Vector3f& normal)
{
    MeshLoader::addNormal(
        normal,
        [this](int index){
            (*normalIndices)[normalIndicesIndex++] = index;
            (*normalIndices)[normalIndicesIndex++] = index;
            (*normalIndices)[normalIndicesIndex++] = index;
        });
}


void BinaryMeshLoader::addVertex(const Vector3f& vertex)
{
    MeshLoader::addVertex(
        vertex,
        [this](int index){
            (*triangleVertices)[triangleVerticesIndex++] = index;
        });
}


void BinaryMeshLoader::load(const string& filename, size_t triangleOffset, size_t numTriangles)
{
    loaderThread = thread(
        [this, filename, triangleOffset, numTriangles](){
            ifstream ifs(filename.c_str(), std::ios::in | std::ios::binary);
            load(ifs, triangleOffset, numTriangles);
        });
}


void BinaryMeshLoader::load(ifstream& ifs, size_t triangleOffset, size_t numTriangles)
{
    ifs.seekg(STL_BINARY_HEADER_SIZE + triangleOffset * 50);

    /**
       Buffer size can be specified by the following code.
       It is currently disabled becasue it does not especially obtain a better result
       on Ubuntu 18.04 and GCC 8.3.0.
    */
    /*
    vector<char> buf(20 * 1024 * 1024);
    ifs.rdbuf()->pubsetbuf(&buf.front(), buf.size());
    */
    
    initializeArrays(triangleOffset, numTriangles);

    size_t datasize = sizeof(float) * 3 * 4 + 2;
    char data[datasize];
    for(size_t i = 0; i < numTriangles; ++i){
        ifs.read(data, datasize);
        addNormal(Vector3f(reinterpret_cast<float*>(data)));
        addVertex(Vector3f(reinterpret_cast<float*>(&data[12])));
        addVertex(Vector3f(reinterpret_cast<float*>(&data[24])));
        addVertex(Vector3f(reinterpret_cast<float*>(&data[36])));
    }
}


SgMeshPtr STLSceneLoaderImpl::loadAsciiFormat(const string& filename)
{
    AsciiMeshLoader loader;

    SgMeshPtr mesh;
    try {
        mesh = loader.load(filename);
    }
    catch(const std::exception& ex){
        os() << ex.what() << endl;
    }
    return mesh;
}


void AsciiMeshLoader::addNormal(const Vector3f& normal)
{
    MeshLoader::addNormal(
        normal,
        [this](int index){
            normalIndices->push_back(index);
            normalIndices->push_back(index);
            normalIndices->push_back(index);
        });
}


void AsciiMeshLoader::addVertex(const Vector3f& vertex)
{
    MeshLoader::addVertex(
        vertex,
        [this](int index){
            triangleVertices->push_back(index);
        });
}


class Scanner
{
public:
    ifstream ifs;
    static const size_t bufsize = 256;
    char buf[bufsize];
    char* pos;
    int lineNumber;
    string filename;
    
    Scanner(const string& filename);
    bool getline();
    void skipSpaces();
    bool checkString(const char* str);
    void checkStringEx(const char* str);
    bool readString(string& out_string);
    void readFloatEx(float& out_value);
    void checkLFEx();
    bool checkEOF();
    void throwEx(const string& error);
};


Scanner::Scanner(const string& filename)
    : ifs(filename, std::ios::in),
      filename(filename)
{
    lineNumber = 0;
    buf[0] = '\0';
    pos = buf;
    if(!getline()){
        throwEx("No data");
    }
}


bool Scanner::getline()
{
    pos = buf;
    if(ifs.getline(buf, bufsize)){
        ++lineNumber;
        return true;
    }
    if(!ifs.eof()){
        if(ifs.gcount() > 0){
            throwEx("Too long line");
        } else {
            throwEx("I/O error");
        }
    }
    buf[0] = '\0';
    return !ifs.eof();
}
        

void Scanner::skipSpaces()
{
    while(*pos == ' ' && *pos != '\0'){
        ++pos;
    }
}


bool Scanner::checkString(const char* str)
{
    skipSpaces();

    char* pos0 = pos;
    while(*str != '\0'){
        if(*str++ != *pos++){
            pos = pos0;
            return false;
        }
    }
    return true;
}


void Scanner::checkStringEx(const char* str)
{
    const char* org = str;
    if(!checkString(str)){
        throwEx(format("\"{}\" is expected", org));
    }
}


bool Scanner::readString(string& out_string)
{
    skipSpaces();
    
    char* pos0 = pos;
    while(*pos != '\r' && *pos != '\0'){
        ++pos;
    }
    out_string.assign(pos0, pos - pos0);

    return !out_string.empty();
}    
    

void Scanner::readFloatEx(float& out_value)
{
    skipSpaces();
    
    char* tail;
    out_value = strtof(pos, &tail);
    if(tail != pos){
        pos = tail;
    } else {
        throwEx("Invalid value");
    }
}


void Scanner::checkLFEx()
{
    skipSpaces();

    if(*pos == '\r' || *pos == '\0'){
        getline();
    } else {
        throwEx("Invalid value");
    }
}


bool Scanner::checkEOF()
{
    if(ifs.eof()){
        return true;
    }
    do{
        skipSpaces();
    } while((*pos == '\r' || *pos == '\0') && getline());

    return ifs.eof();
}


void Scanner::throwEx(const string& error)
{
    boost::filesystem::path path(filename);
    throw std::runtime_error(
        format(_("{0} at line {1} of \"{2}\"."),
               error, lineNumber, path.filename().string()));
}


SgMeshPtr AsciiMeshLoader::load(const string& filename)
{
    initializeArrays();

    Scanner scanner(filename);

    scanner.checkStringEx("solid");
    string name;
    if(scanner.readString(name)){
        sharedMesh->setName(name);
    }
    scanner.checkLFEx();

    do {
        scanner.checkStringEx("facet normal");
        Vector3f v;
        scanner.readFloatEx(v.x());
        scanner.readFloatEx(v.y());
        scanner.readFloatEx(v.z());
        addNormal(v);
        scanner.checkLFEx();

        scanner.checkStringEx("outer loop");
        scanner.checkLFEx();
        for(int i=0; i < 3; ++i){
            scanner.checkStringEx("vertex");
            scanner.readFloatEx(v.x());
            scanner.readFloatEx(v.y());
            scanner.readFloatEx(v.z());
            addVertex(v);
            scanner.checkLFEx();
        }
        scanner.checkStringEx("endloop");
        scanner.checkLFEx();

        scanner.checkStringEx("endfacet");
        scanner.checkLFEx();

        if(scanner.checkEOF()){
            scanner.throwEx("\"endsolid\" is not found");
        }

    } while(!scanner.checkString("endsolid"));

    return completeMesh(true);
}
