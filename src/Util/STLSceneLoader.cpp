/*!
  @author Shin'ichiro Nakaoka
*/

#include "STLSceneLoader.h"
#include "SceneDrawables.h"
#include "SceneLoader.h"
#include "NullOut.h"
#include "FileUtil.h"
#include "strtofloat.h"
#include <fmt/format.h>
#include <fstream>
#include <thread>
#include <stdexcept>
#include <cstdlib>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

typedef ifstream::pos_type pos_type;

const bool ENABLE_COMPACTION = true;
const int VertexSearchLength = 27;
const int NormalSearchLength = 15;
const size_t NumTrianglesPerThread = 100000;
const pos_type AsciiSizePerThread = 1024 * 1024;
const size_t STL_BINARY_HEADER_SIZE = 84;

struct Registration {
    Registration(){
        SceneLoader::registerLoader(
            "stl",
            []() -> shared_ptr<AbstractSceneLoader> {
                return make_shared<STLSceneLoader>(); });
    }
} registration;

class Scanner
{
public:
    ifstream ifs;
    static const size_t bufsize = 256;
    char buf[bufsize];
    char* pos;
    size_t lineNumber;
    string filename;
    
    void open(const string& filename)
    {
        // The binary mode is faster on Windows
        ifs.open(filename, std::ios::in | std::ios::binary);
        this->filename = filename;
        clear();
    }

    void clear()
    {
        lineNumber = 0;
        buf[0] = '\0';
        pos = buf;
    }        

    bool getLine()
    {
        pos = buf;
        bool result = false;

#ifndef _WIN32
        if(ifs.getline(buf, bufsize)){
            result = true;
        }
#else
        // The following code is faster on Windows
        if(ifs.get(buf, bufsize, '\n')){
            ifs.ignore();
            result = true;
        } else if(!ifs.eof() && ifs.fail()){
            ifs.clear();
            if(ifs.peek() == '\n'){
                ifs.ignore();
                result = true;
            }
        }
#endif
        if(result){
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

    void skipSpaces()
    {
        while(*pos == ' '){
            ++pos;
        }
    }

    bool checkLF()
    {
        skipSpaces();
        if(*pos == '\0'){
            return true;
        } else if(*pos == '\r'){
            ++pos;
            return true;
        }
        return false;
    }

    void checkLFEx()
    {
        if(!checkLF()){
            throwEx("Invalid value");
        }
    }

    bool checkEOF()
    {
        if(checkLF()){
            return ifs.eof();
        }
        return false;
    }

    bool checkString(const char* str)
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

    void checkStringEx(const char* str)
    {
        const char* org = str;
        if(!checkString(str)){
            throwEx(format("\"{}\" is expected", org));
        }
    }

    bool seekToString(pos_type initialSeekPos, const char* str, size_t maxLength, pos_type& out_seekPos)
    {
        ifs.seekg(initialSeekPos);
        getLine();
        
        pos_type currentLinePos = initialSeekPos;
        pos_type nextLinePos = ifs.tellg();
        const char* str0 = str;
        char* found = nullptr;
        while(true){
            while(*pos != '\0'){
                ++pos;
            }
            if(!getLine()){
                break;
            }
            currentLinePos = nextLinePos;
            nextLinePos = ifs.tellg();
            if(currentLinePos - initialSeekPos > maxLength){
                break;
            }
            skipSpaces();

            if(*pos == *str){
                ++pos;
                ++str;
                while(true){
                    if(*str == '\0'){
                        out_seekPos = currentLinePos;
                        ifs.seekg(out_seekPos);
                        clear();
                        return true; // found
                    }
                    if(*pos++ != *str++){
                        str = str0;
                        break;
                    }
                }
            }
        }
        out_seekPos = initialSeekPos;
        return false;
    }
            
    bool readString(string& out_string)
    {
        skipSpaces();
        char* pos0 = pos;
        while(*pos != '\r' && *pos != '\0'){
            ++pos;
        }
        out_string.assign(pos0, pos - pos0);
        return !out_string.empty();
    }
    
    void readFloatEx(float& out_value)
    {
        skipSpaces();
        char* tail;
        out_value = cnoid::strtof(pos, &tail);
        if(tail != pos){
            pos = tail;
        } else {
            throwEx("Invalid value");
        }
    }

    void throwEx(const string& error)
    {
        stdx::filesystem::path path(filename);
        throw std::runtime_error(
            format(_("{0} at line {1} of \"{2}\"."),
                   error, lineNumber, path.filename().string()));
    }
};

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

    /**
       The following two variables are only used in BinaryMeshLoader.
       Defining them here instead of BinaryMeshLoader can improve the loading speed.
       It seems that this is because the cache hit ratio is improved.
    */
    size_t triangleVerticesIndex;
    size_t normalIndicesIndex;
    
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

    string errorMessage;

    MeshLoader(size_t numTriangles);
    MeshLoader(const MeshLoader& mainLoader);
    template<typename AddIndexFunction>
    void addNormal(const Vector3f& normal, AddIndexFunction addIndex);
    template<typename AddIndexFunction>
    void addVertex(const Vector3f& vertex, AddIndexFunction addIndex);
    bool join();
    int findElement(
        const Vector3f& element, const SgVectorArray<Vector3f>& prevElements, int searchLength);
    void findRedundantElementsBetweenLoaders(
        SgVectorArray<Vector3f>& elements, SgVectorArray<Vector3f>& prevElements,
        vector<int>& mergedElementIndexMap, int& numMergedElements, int searchLength);
    virtual void initializeIntegration(MeshLoader* prevLoader);
    void integrateElements(
        SgVectorArray<Vector3f>& elements, SgVectorArray<Vector3f>& subElements, int elementArrayOffset,
        SgIndexArray& elementIndices, SgIndexArray* subElementIndices,
        vector<int>& mergedIndexMap, int numMergedElements);
    void integrate();
    void integrateConcurrently();
    
    SgMeshPtr completeMesh(bool doShrin);
};

class BinaryMeshLoader : public MeshLoader
{
public:
    BinaryMeshLoader(size_t numTriangles) : MeshLoader(numTriangles) { }
    BinaryMeshLoader(const BinaryMeshLoader& mainLoader) : MeshLoader(mainLoader) { }
    void initializeArrays(size_t triangleOffset, size_t numTriangles);
    void addNormal(const Vector3f& normal);
    void addVertex(const Vector3f& vertex);
    void load(ifstream& ifs, size_t triangleOffset, size_t numTriangles);
    void loadConcurrently(const string& filename, size_t triangleOffset, size_t numTriangles);
};

class AsciiMeshLoader : public MeshLoader
{
public:
    Scanner scanner;
    pos_type seekOffset;
    pos_type seekEnd;
    SgIndexArray subTriangleVertices;
    SgIndexArray subNormalIndices;
    string filename;
    bool isSuccessfullyLoaded;
    
    AsciiMeshLoader(const string& filename, bool doOpen);
    AsciiMeshLoader(const AsciiMeshLoader& mainLoader);
    void addNormal(const Vector3f& normal);
    void addVertex(const Vector3f& vertex);
    bool seekToTriangleBorderPosition(pos_type position);
    bool load();
    void loadConcurrently();
    void loadTriangles();
    virtual void initializeIntegration(MeshLoader* prevLoader) override;
};

template<class MeshLoaderType>
vector<MeshLoader*> getMeshLoaderPointers(vector<MeshLoaderType>& loaders)
{
    vector<MeshLoader*> converted(loaders.size());
    for(size_t i=0; i < loaders.size(); ++i){
        converted[i] = &loaders[i];
    }
    return converted;
}

}

namespace cnoid {

class STLSceneLoaderImpl
{
public:
    size_t maxNumThreads;
    ostream* os_;
    ostream& os() { return *os_; }

    STLSceneLoaderImpl();
    SgNode* load(const string& filename);
    SgMeshPtr loadBinaryFormat(const string& filename, ifstream& ifs, size_t numTriangles);
    SgMeshPtr loadBinaryFormatConcurrently(
        const string& filename, ifstream& ifs, size_t numTriangles, size_t numThreads, BinaryMeshLoader& mainLoader);
    SgMeshPtr loadAsciiFormat(const string& filename, pos_type fileSize);
    SgMeshPtr loadAsciiFormatConcurrently(
        const string& filename, AsciiMeshLoader& mainLoader, pos_type fileSize, size_t numThreads);
    SgMeshPtr integrateSubLoaderMeshes(MeshLoader& mainLoader, vector<MeshLoader*> loaders);
};

}


MeshLoader::MeshLoader(size_t numTriangles)
    : sharedMesh(new SgMesh),
      numTriangles(numTriangles)
{
    triangleOffset = 0;

    vertices = sharedMesh->getOrCreateVertices();
    normals = sharedMesh->getOrCreateNormals();

    triangleVertices = &sharedMesh->triangleVertices();
    normalIndices = &sharedMesh->normalIndices();
    
    sharedMesh->setNumTriangles(numTriangles);
    normalIndices->resize(numTriangles * 3);
}


MeshLoader::MeshLoader(const MeshLoader& mainLoader)
    : sharedMesh(mainLoader.sharedMesh)
{
    triangleOffset = 0;
    numTriangles = 0;
    vertices = new SgVertexArray;
    triangleVertices = nullptr;
    normals = new SgNormalArray;
    normalIndices = nullptr;
}


/**
   The following determination can be performed in Eigen originally as
   v1.isApprox(v2), but the binary generated by Visual C++ is much slower
   than that of GCC on Ubuntu Linux. By using the following implementation,
   the similar performance can be obtained with Visual C++. This implementation
   is slightly faster than the original Eigen expression even on Ubuntu Linux
   with GCC, so the implementation is used by default.
*/
static inline bool isApprox(const Vector3f& v1, const Vector3f& v2)
{
    float l2 = 0.0f;
    for(int i=0; i < 3; ++i){
        auto d = v1[i] - v2[i];
        l2 += d * d;
    }
    const float e = 1e-5f;
    return (l2 < e * e);
}
    

template<typename AddIndexFunction>
void MeshLoader::addNormal(const Vector3f& normal, AddIndexFunction addIndex)
{
    bool found = false;
    int index = normals->size() - 1;

    if(ENABLE_COMPACTION){
        int minIndex = std::max(0, index - (NormalSearchLength - 1));
        while(index >= minIndex){
            if(isApprox(normal, (*normals)[index])){
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
            if(isApprox(vertex, (*vertices)[index])){
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


bool MeshLoader::join()
{
    if(loaderThread.joinable()){
        loaderThread.join();
        return true;
    }
    return false;
}


int MeshLoader::findElement
(const Vector3f& element, const SgVectorArray<Vector3f>& prevElements, int searchLength)
{
    bool found = false;
    int index = prevElements.size() - 1;
    int minIndex = std::max(0, index - (searchLength - 1));
    while(index >= minIndex){
        if(isApprox(element, prevElements[index])){
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
(SgVectorArray<Vector3f>& elements, SgVectorArray<Vector3f>& subElements, int elementArrayOffset,
 SgIndexArray& elementIndices, SgIndexArray* subElementIndices,
 vector<int>& mergedIndexMap, int numMergedElements)
{
    auto newElementIter = elements.begin() + elementArrayOffset;

    for(size_t i=0; i < mergedIndexMap.size(); ++i){
        auto mappedIndex = mergedIndexMap[i];
        if(mappedIndex >= 0){
            *newElementIter++ = subElements[i];
        }
        mergedIndexMap[i] += elementArrayOffset; // Conver to the global index
    }
        
    std::copy(subElements.begin() + mergedIndexMap.size(), subElements.end(), newElementIter);

    int pos = triangleOffset * 3;
    const int unmappedElementIndexOffset = elementArrayOffset - numMergedElements;

    if(subElementIndices){
        for(auto& index : *subElementIndices){
            if(index < mergedIndexMap.size()){
                elementIndices[pos++] = mergedIndexMap[index];
            } else {
                elementIndices[pos++] = index + unmappedElementIndexOffset;
            }
        }
    } else {
        const int end = pos + numTriangles * 3;
        while(pos < end){
            auto& index = elementIndices[pos++];
            if(index < mergedIndexMap.size()){
                index = mergedIndexMap[index];
            } else {
                index += unmappedElementIndexOffset;
            }
        }
    }
 }


void MeshLoader::integrate()
{
    SgIndexArray* subTriangleVertices = nullptr;
    if(triangleVertices != &sharedMesh->triangleVertices()){
        subTriangleVertices = triangleVertices;
    }
    
    integrateElements(
        *sharedMesh->vertices(), *vertices, vertexArrayOffset,
        sharedMesh->triangleVertices(), subTriangleVertices,
        mergedVertexIndexMap, numMergedVertices);

    SgIndexArray* subNormalIndices = nullptr;
    if(normalIndices != &sharedMesh->normalIndices()){
        subNormalIndices = normalIndices;
    }

    integrateElements(
        *sharedMesh->normals(), *normals, normalArrayOffset,
        sharedMesh->normalIndices(), subNormalIndices,
        mergedNormalIndexMap, numMergedNormals);
}


void MeshLoader::integrateConcurrently()
{
    loaderThread = thread([this](){ integrate(); });
}
 

SgMeshPtr MeshLoader::completeMesh(bool doShrink)
{
    if(vertices->empty()){
        return nullptr;
    }

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
    maxNumThreads = std::max((unsigned)1, thread::hardware_concurrency());

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
        mesh = loadAsciiFormat(filename, fileSize);
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

    size_t numThreads = std::min(maxNumThreads, std::max(size_t(1), numTriangles / NumTrianglesPerThread));

    /**
       The number of threads is limited to 4 at maximum because just increasing
       the number of threds accessing the same file will slow down overall file
       reading speed.
    */
    if(numThreads > 4){
        numThreads = 4;
    }

    if(numThreads == 1){
        mainLoader.load(ifs, 0, numTriangles);
        return mainLoader.completeMesh(true);
    } else {
        return loadBinaryFormatConcurrently(filename, ifs, numTriangles, numThreads, mainLoader);
    }
}


SgMeshPtr STLSceneLoaderImpl::loadBinaryFormatConcurrently
(const string& filename, ifstream& ifs, size_t numTriangles, size_t numThreads, BinaryMeshLoader& mainLoader)
{
    vector<BinaryMeshLoader> loaders(numThreads, mainLoader);
    
    int index = 0;
    size_t triangleOffset = 0;
    size_t numTrianglesPerThread = numTriangles / numThreads;
    while(index < numThreads - 1){
        loaders[index].loadConcurrently(filename, triangleOffset, numTrianglesPerThread);
        ++index;
        triangleOffset += numTrianglesPerThread;
    }
    loaders[index].load(ifs, triangleOffset, numTriangles - triangleOffset);

    for(auto& loader : loaders){
        loader.join();
    }

    return integrateSubLoaderMeshes(mainLoader, getMeshLoaderPointers(loaders));
}


void BinaryMeshLoader::initializeArrays(size_t triangleOffset, size_t numTriangles)
{
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


void BinaryMeshLoader::load(ifstream& ifs, size_t triangleOffset, size_t numTriangles)
{
    ifs.seekg(STL_BINARY_HEADER_SIZE + triangleOffset * 50);

    /**
       Buffer size can be specified by the following code.
       It is currently disabled becasue it does not especially obtain a better result
       on both Ubuntu 18.04 + GCC 8.3.0 and Visual C++ 2017.
    */
    /*
    vector<char> buf(1 * 1024 * 1024);
    ifs.rdbuf()->pubsetbuf(&buf.front(), buf.size());
    */
    
    initializeArrays(triangleOffset, numTriangles);

    const size_t datasize = sizeof(float) * 3 * 4 + 2;
    char data[datasize];
    for(size_t i = 0; i < numTriangles; ++i){
        ifs.read(data, datasize);
        addNormal(Vector3f(reinterpret_cast<float*>(data)));
        addVertex(Vector3f(reinterpret_cast<float*>(&data[12])));
        addVertex(Vector3f(reinterpret_cast<float*>(&data[24])));
        addVertex(Vector3f(reinterpret_cast<float*>(&data[36])));
    }
}


void BinaryMeshLoader::loadConcurrently(const string& filename, size_t triangleOffset, size_t numTriangles)
{
    loaderThread = thread(
        [this, filename, triangleOffset, numTriangles](){
            ifstream ifs(filename.c_str(), std::ios::in | std::ios::binary);
            load(ifs, triangleOffset, numTriangles);
        });
}


SgMeshPtr STLSceneLoaderImpl::loadAsciiFormat(const string& filename, pos_type fileSize)
{
    size_t numThreads = std::min(maxNumThreads, std::max(size_t(1), size_t(fileSize / AsciiSizePerThread)));

#ifndef _WIN32
    if(numThreads > 4){
        numThreads = 4;
    }
#endif
    
    SgMeshPtr mesh;
    
    bool doOpen = (numThreads == 1);
    AsciiMeshLoader mainLoader(filename, doOpen);
    
    if(numThreads == 1){
        if(mainLoader.load()){
            mesh = mainLoader.completeMesh(true);
        }
    } else {
        mesh =loadAsciiFormatConcurrently(filename, mainLoader, fileSize, numThreads);
    }
    
    return mesh;
}


SgMeshPtr STLSceneLoaderImpl::loadAsciiFormatConcurrently
(const string& filename, AsciiMeshLoader& mainLoader, pos_type fileSize, size_t numThreads)
{
    vector<AsciiMeshLoader> loaders(numThreads, mainLoader);
    pos_type fragmentSize = fileSize / numThreads;
    bool ready = true;
    
    try {
        AsciiMeshLoader* prevLoader = &loaders.front();
        for(size_t i=1; i < loaders.size(); ++i){
            auto& loader = loaders[i];
            if(loader.seekToTriangleBorderPosition(i * fragmentSize)){
                prevLoader->seekEnd = loader.seekOffset;
                prevLoader->loadConcurrently();
                prevLoader = &loader;
            }
        }
    }
    catch(const std::exception& ex){
        os() << ex.what() << endl;
        ready = false;
    }

    if(ready){
        auto& lastLoader = loaders.back();
        lastLoader.seekEnd = fileSize;
        lastLoader.load();
    }

    for(auto& loader : loaders){
        if(loader.join()){
            if(!loader.isSuccessfullyLoaded){
                if(!loader.errorMessage.empty()){
                    os() << loader.errorMessage << endl;
                }
                ready = false;
            }
        }
    }

    SgMeshPtr mesh;
    if(ready){
        mesh = integrateSubLoaderMeshes(mainLoader, getMeshLoaderPointers(loaders));
    }
    return mesh;
}


AsciiMeshLoader::AsciiMeshLoader(const string& filename, bool doOpen)
    : MeshLoader(0),
      filename(filename)
{
    if(doOpen){
        scanner.open(filename);
    }
    seekOffset = 0;
    seekEnd = 0;
}


AsciiMeshLoader::AsciiMeshLoader(const AsciiMeshLoader& mainLoader)
    : MeshLoader(mainLoader),
      filename(mainLoader.filename)
{
    triangleVertices = &subTriangleVertices;
    normalIndices = &subNormalIndices;
    
    scanner.open(filename);
    seekOffset = 0;
    seekEnd = 0;
}


bool AsciiMeshLoader::seekToTriangleBorderPosition(pos_type position)
{
    return scanner.seekToString(position, "facet normal", 1024, seekOffset);
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


bool AsciiMeshLoader::load()
{
    try {
        loadTriangles();
        isSuccessfullyLoaded = true;
    }
    catch(const std::exception& ex){
        errorMessage = ex.what();
        isSuccessfullyLoaded = false;
    }
    return isSuccessfullyLoaded;
}


void AsciiMeshLoader::loadConcurrently()
{
    loaderThread = thread([this](){ load(); });
}


void AsciiMeshLoader::loadTriangles()
{
    if(!scanner.getLine()){
        scanner.throwEx("No data");
    }
    
    if(seekOffset == pos_type(0)){
        scanner.checkStringEx("solid");
        string name;
        if(scanner.readString(name)){
            sharedMesh->setName(name);
        }
        scanner.checkLFEx();
        scanner.getLine();
    }

    while(true) {
        scanner.checkStringEx("facet normal");
        Vector3f v;
        scanner.readFloatEx(v.x());
        scanner.readFloatEx(v.y());
        scanner.readFloatEx(v.z());
        addNormal(v);
        scanner.checkLFEx();
        scanner.getLine();

        scanner.checkStringEx("outer loop");
        scanner.checkLFEx();
        scanner.getLine();
        for(int i=0; i < 3; ++i){
            scanner.checkStringEx("vertex");
            scanner.readFloatEx(v.x());
            scanner.readFloatEx(v.y());
            scanner.readFloatEx(v.z());
            addVertex(v);
            scanner.checkLFEx();
            scanner.getLine();
        }
        scanner.checkStringEx("endloop");
        scanner.checkLFEx();
        scanner.getLine();

        scanner.checkStringEx("endfacet");
        scanner.checkLFEx();

        if(seekEnd > 0 && scanner.ifs.tellg() >= seekEnd){
            break;
        }
        scanner.getLine();
        
        if(scanner.checkString("endsolid")){
            break;
        }
        if(scanner.checkEOF()){
            scanner.throwEx("\"endsolid\" is not found");
        }
    }
}


void AsciiMeshLoader::initializeIntegration(MeshLoader* prevLoader)
{
    MeshLoader::initializeIntegration(prevLoader);

    numTriangles = triangleVertices->size() / 3;
    
    if(prevLoader){
        triangleOffset = prevLoader->triangleOffset + prevLoader->numTriangles;
    }
}


SgMeshPtr STLSceneLoaderImpl::integrateSubLoaderMeshes
(MeshLoader& mainLoader, vector<MeshLoader*> loaders)
{
    size_t totalNumVertices = 0;
    size_t totalNumNormals = 0;
    size_t totalNumTriangles = 0;
    MeshLoader* prevLoader = nullptr;
    for(auto loader : loaders){
        loader->initializeIntegration(prevLoader);
        totalNumVertices += loader->numActualVertices;
        totalNumNormals += loader->numActualNormals;
        totalNumTriangles += loader->numTriangles;
        mainLoader.bbox.expandBy(loader->bbox);
        prevLoader = loader;
    }

    auto mesh = mainLoader.sharedMesh;
    mesh->vertices()->resize(totalNumVertices);
    mesh->normals()->resize(totalNumNormals);

    // In the case of the ASCII format
    if(mesh->numTriangles() == 0){
        mesh->setNumTriangles(totalNumTriangles);
    }
    if(!mesh->hasNormalIndices()){
        mesh->normalIndices().resize(totalNumTriangles * 3);
    }

    for(int i=0; i < loaders.size() - 1; ++i){
        loaders[i]->integrateConcurrently();
    }
    loaders.back()->integrate();
            
    for(auto loader : loaders){
        loader->join();
    }

    return mainLoader.completeMesh(false);
}
