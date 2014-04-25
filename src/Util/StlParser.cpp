/*!
 * @brief  Defines the minimum processing for performing pasing file for STL. 
 * @author Hisashi Ikari 
 * @file
 */
#include "SceneGraph.h"
#include "SceneShape.h"
#include "Exception.h"
#include "StlParser.h"
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <iostream>
#include <fstream>
#include "gettext.h"

using namespace boost::algorithm;

namespace cnoid {

/*!
 * @brief Perform the processing for each node as sax.
 */
class StlParserImpl
{
public:
    StlParserImpl() {}
    SgGroup* createScene(const std::string& fileName);

protected:        
    int  symbol      (std::string& value);
    void readVertices(std::string& value, SgMesh* mesh);
    void readNormals (std::string& value, SgMesh* mesh);

    SgMesh*         createMesh     ();
    SgMaterial*     createMaterial ();
    SgPosTransform* createTransform();
    void            createIndexes  (SgMesh* mesh);

    void setVector3(std::string& value, SgVertexArray* array);
};


enum {
    EXT_VERTEX,
    EXT_NORMAL,
};


inline int StlParserImpl::symbol(std::string& value) 
{
    // vertex is large frequency.
    static const int SYMBOL_NUM = 2;
    static const std::string SYMBOLS[SYMBOL_NUM] = {"vertex", "facet normal"};
    for (int i = 0; i < SYMBOL_NUM; i++) {
        if (boost::istarts_with(value, SYMBOLS[i])) {
            return i;
        }
    }
    return -1;
}


StlParser::StlParser()
{
    stlParserImpl = new StlParserImpl;
}


StlParser::~StlParser()
{
    delete stlParserImpl;
}


SgGroup* StlParser::createScene(const std::string& fileName)
{
    return stlParserImpl->createScene(fileName);
}


SgGroup* StlParserImpl::createScene(const std::string& fileName)
{
    SgGroup* group = new SgGroup();
    std::ifstream* ifs = new std::ifstream(fileName.c_str(), std::ios::in);

    SgShape*     shape     = new SgShape();
    SgMesh*      mesh      = createMesh();
    SgTransform* transform = createTransform();
    SgMaterial*  material  = createMaterial();

    std::string line;
    while (!ifs->eof() && getline(*ifs, line)) {
        trim(line);
        switch (symbol(line)) {
        case EXT_NORMAL: readNormals (line, mesh); break;
        case EXT_VERTEX: readVertices(line, mesh); break;
        }
    }
    createIndexes(mesh);

    shape->setMesh(mesh);
    shape->setMaterial(material);
    transform->addChild(shape);
    group->addChild(transform);

    delete ifs;
    return group; 
}


inline void StlParserImpl::readNormals(std::string& value, SgMesh* mesh)
{
    std::string normals = value.substr(12);
    setVector3(normals, mesh->normals());
}


inline void StlParserImpl::readVertices(std::string& value, SgMesh* mesh)
{
    std::string vertices = value.substr(6);
    setVector3(vertices, mesh->vertices());
}


void StlParserImpl::setVector3(std::string& value, SgVertexArray* array)
{
    trim(value);

    Vector3f point;
    int i = 0;
    for ( split_iterator<std::string::iterator> iter = make_split_iterator( value, token_finder(is_space(), token_compress_on) );
          iter != split_iterator<std::string::iterator>(); 
          iter++)
        {
            point[i++] = boost::lexical_cast<float>(*iter);
            if (3 == i) {
                array->push_back(point);
                return;
            }
        }
}


void StlParserImpl::createIndexes(SgMesh* mesh)
{
    int sv = mesh->vertices()->size();
    int sn = mesh->normals()->size();

    for (int i = 0; i < sv; i+=3) {
        mesh->addTriangle(i + 0, i + 1, i + 2);
    }
    for (int i = 0; i < sn; i++) {
        mesh->normalIndices().push_back(i);
        mesh->normalIndices().push_back(i);
        mesh->normalIndices().push_back(i);
    }
}


SgMesh* StlParserImpl::createMesh()
{
    SgMesh* mesh = new SgMesh;

    // It must be generated normals and vertices.
    SgVertexArrayPtr vertices = new SgVertexArray;
    SgNormalArrayPtr normals  = new SgNormalArray;

    mesh->setVertices(vertices);
    mesh->setNormals (normals);
        
    return mesh;
}


SgMaterial* StlParserImpl::createMaterial()
{
    SgMaterial* material = new SgMaterial;

    // Use the value of the same default of dae.
    material->setEmissiveColor   (Vector3f(0.0, 0.0, 0.0));
    material->setAmbientIntensity(0.0);
    material->setDiffuseColor    (Vector3f(1.0, 1.0, 1.0));
    material->setSpecularColor   (Vector3f(0.0, 0.0, 0.0));
    material->setShininess       (0.0);
    material->setTransparency    (0.0);

    return material;
}


SgPosTransform* StlParserImpl::createTransform()
{
    SgPosTransform* transform = new SgPosTransform;

    // Use the value of the same default of dae.
    Affine3 S;
    S.linear() = (Vector3(1.0, 1.0, 1.0)).asDiagonal();
    S.translation().setZero();

    AngleAxis SR = AngleAxis(0.0, Vector3(0, 0, 0)), R = SR;
    Translation3 T(Vector3(0.0, 0.0, 0.0)), C = T;

    transform->setTransform(T * C * R * SR * S * SR.inverse() * C.inverse());

    return transform;
}


};
