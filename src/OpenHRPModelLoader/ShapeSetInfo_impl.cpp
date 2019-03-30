/*!
  @file
  @author Shizuko Hattori
*/

#include "ShapeSetInfo_impl.h"
#include "ModelLoaderUtil.h"

using namespace std;
using namespace cnoid;
using namespace cnoid::openHRPModelloader;
using namespace OpenHRP;

namespace {
typedef map< int, ShapePrimitiveType>PrimitiveTypeMap;
PrimitiveTypeMap primitiveTypeMap;
}

ShapeSetInfo_impl::ShapeSetInfo_impl(PortableServer::POA_ptr poa) :
    poa(PortableServer::POA::_duplicate(poa))
{
    if(primitiveTypeMap.empty()){
        primitiveTypeMap[SgMesh::MESH]     = SP_MESH;
        primitiveTypeMap[SgMesh::BOX]      = SP_BOX;
        primitiveTypeMap[SgMesh::SPHERE]   = SP_SPHERE;
        primitiveTypeMap[SgMesh::CYLINDER] = SP_CYLINDER;
        primitiveTypeMap[SgMesh::CONE]     = SP_CONE;
        primitiveTypeMap[SgMesh::CAPSULE]  = SP_MESH;
        //                                 = SP_PLANE;
    }
}


ShapeSetInfo_impl::~ShapeSetInfo_impl()
{

}


PortableServer::POA_ptr ShapeSetInfo_impl::_default_POA()
{
    return PortableServer::POA::_duplicate(poa);
}


ShapeInfoSequence* ShapeSetInfo_impl::shapes()
{
    return new ShapeInfoSequence(shapes_);
}


AppearanceInfoSequence* ShapeSetInfo_impl::appearances()
{
    return new AppearanceInfoSequence(appearances_);
}


MaterialInfoSequence* ShapeSetInfo_impl::materials()
{
    return new MaterialInfoSequence(materials_);
}


TextureInfoSequence* ShapeSetInfo_impl::textures()
{
    return new TextureInfoSequence(textures_);
}


void ShapeSetInfo_impl::setShapeIndices(SgNode* node, TransformedShapeIndexSequence& shapeIndices)
{
    MeshExtractor* extractor = new MeshExtractor;
    extractor->extract(node, [&](){ createTransformedShapeIndex( extractor, shapeIndices ); });
}


void ShapeSetInfo_impl::createTransformedShapeIndex(MeshExtractor* extractor, TransformedShapeIndexSequence& shapeIndices)
{
    SgShape* shape = extractor->currentShape();
    const Affine3& T = extractor->currentTransform();

    int shapeInfoIndex;
    SgShapeToShapeInfoIndexMap::iterator p = shapeInfoIndexMap.find(shape);
    if(p != shapeInfoIndexMap.end()){
        shapeInfoIndex = p->second;
    }else{
        shapeInfoIndex = shapes_.length();
        shapes_.length(shapeInfoIndex + 1);
        ShapeInfo& shapeInfo = shapes_[shapeInfoIndex];

        shapeInfo.url = "";
        setMesh(shape->mesh(), shapeInfo);
        setPrimitive(shape->mesh(), shapeInfo);
        shapeInfo.appearanceIndex = setAppearance(shape);
        shapeInfoIndexMap[shape] = shapeInfoIndex;
    }

    int n = shapeIndices.length();
    shapeIndices.length(n + 1);
    TransformedShapeIndex& transformedShapeIndex = shapeIndices[n];

    transformedShapeIndex.shapeIndex = shapeInfoIndex;
    transformedShapeIndex.inlinedShapeTransformMatrixIndex = -1;
    setTransformMatrix(T, transformedShapeIndex.transformMatrix);

}


void ShapeSetInfo_impl::setMesh(SgMesh* mesh, ShapeInfo& shapeInfo)
{
    SgVertexArray& vertices = *mesh->vertices();
    const int numVertices = vertices.size();
    shapeInfo.vertices.length(numVertices * 3);

    for(size_t pos=0, i=0; i < numVertices; i++){
        const Vector3f& v = vertices[i];
        setVector3(v, &shapeInfo.vertices[pos]);
        pos +=3;
    }

    SgIndexArray& indices = mesh->triangleVertices();
    setSequence( indices, shapeInfo.triangles );
}


void ShapeSetInfo_impl::setPrimitive(SgMesh* mesh, ShapeInfo& shapeInfo)
{
    shapeInfo.primitiveType = primitiveTypeMap[mesh->primitiveType()];
    FloatSequence& param = shapeInfo.primitiveParameters;

    switch(shapeInfo.primitiveType){
    case SP_BOX : {
        const Vector3& s = mesh->primitive<SgMesh::Box>().size;
        setSequence( s, param );
        break; }
    case SP_SPHERE : {
        SgMesh::Sphere sphere = mesh->primitive<SgMesh::Sphere>();
        param.length( CORBA::ULong(1) );
        param[0] = sphere.radius;
        break; }
    case SP_CYLINDER : {
        SgMesh::Cylinder cylinder = mesh->primitive<SgMesh::Cylinder>();
        param.length( CORBA::ULong(5) );
        param[0] = cylinder.radius;
        param[1] = cylinder.height;
        param[2] = cylinder.top ? 1 : 0;
        param[3] = cylinder.bottom ? 1 : 0;
        param[4] = cylinder.side ? 1 : 0;
        break; }
    case SP_CONE : {
        SgMesh::Cone cone = mesh->primitive<SgMesh::Cone>();
        param.length( CORBA::ULong(4) );
        param[0] = cone.radius;
        param[1] = cone.height;
        param[2] = cone.bottom ? 1 : 0;
        param[3] = cone.side ? 1 : 0;
        break; }
    default :
        break;
    }
}


int ShapeSetInfo_impl::setAppearance(SgShape* shape)
{
    int appearanceIndex = appearances_.length();
    appearances_.length(appearanceIndex + 1);
    AppearanceInfo& appInfo = appearances_[appearanceIndex];

    // Choreonoid Shape node has no setting corresponding to solid and creaseAngle fields
    appInfo.solid = true;
    appInfo.creaseAngle = 0;

    SgMesh* mesh = shape->mesh();
    if(mesh->hasNormals()){
        setNormals(mesh, appInfo);
    }
    if(mesh->hasColors()){
        setColors(mesh, appInfo);
    }

    SgMaterial* material = shape->material();
    if(material){
        appInfo.materialIndex = setMaterial(material);
    }else{
        appInfo.materialIndex = -1;
    }

    SgTexture* texture = shape->texture();
    if(texture){
        appInfo.textureIndex  = setTexture(texture);
        setTexCoords(mesh, texture->textureTransform(), appInfo);
    }else{
        appInfo.textureIndex  = -1;
    }

    return appearanceIndex;
}


void ShapeSetInfo_impl::setColors(SgMesh* mesh, AppearanceInfo& appInfo)
{
    SgColorArray& colors = *mesh->colors();
    int numColors = colors.size();
    appInfo.colors.length(numColors * 3);
    for(int pos=0, i=0; i<numColors; i++){
        setVector3(colors[i], &appInfo.colors[pos]);
        pos +=3;
    }

    const SgIndexArray& colorIndices = mesh->colorIndices();
    if(colorIndices.empty()){
        if( numColors==mesh->numTriangles() ){
            appInfo.colorPerVertex = false;
        }else{
            appInfo.colorPerVertex = true;
        }
    }else{
        if( colorIndices.size() == mesh->triangleVertices().size() ){
            appInfo.colorPerVertex = true;
        }else{
            appInfo.colorPerVertex = false;
        }
        setSequence( colorIndices, appInfo.colorIndices );
    }
}


void ShapeSetInfo_impl::setNormals(SgMesh* mesh, AppearanceInfo& appInfo)
{
    SgNormalArray& normals = *mesh->normals();
    int numNormals = normals.size();
    appInfo.normals.length(numNormals * 3);
    for(int pos=0, i=0; i<numNormals; i++){
        setVector3(normals[i], &appInfo.normals[pos]);
        pos +=3;
    }

    const SgIndexArray& normalIndices = mesh->normalIndices();
    if(normalIndices.empty()){
        if( numNormals==mesh->numTriangles() ){
            appInfo.normalPerVertex = false;
        }else{
            appInfo.normalPerVertex = true;
        }
    }else{
        if( normalIndices.size() == mesh->triangleVertices().size() ){
            appInfo.normalPerVertex = true;
        }else{
            appInfo.normalPerVertex = false;
        }
        setSequence( normalIndices, appInfo.normalIndices );
    }
}


int ShapeSetInfo_impl::setMaterial(SgMaterial* material)
{
    SgMaterialToMaterialInfoIndexMap::iterator it = materialInfoIndexMap.find(material);
    if(it!=materialInfoIndexMap.end()){
        return it->second;
    }

    int materialInfoIndex = materials_.length();
    materials_.length(materialInfoIndex + 1 );
    MaterialInfo& materialInfo = materials_[materialInfoIndex];
    materialInfoIndexMap[material] = materialInfoIndex;

    materialInfo.ambientIntensity = material->ambientIntensity();
    materialInfo.shininess = material->shininess();
    materialInfo.transparency = material->transparency();

    setVector3( material->diffuseColor(), materialInfo.diffuseColor);
    setVector3( material->emissiveColor(), materialInfo.emissiveColor);
    setVector3( material->specularColor(), materialInfo.specularColor);

    return materialInfoIndex;
}


int ShapeSetInfo_impl::setTexture(SgTexture* texture)
{
    SgTextureToTextureInfoIndexMap::iterator it = textureInfoIndexMap.find(texture);
    if(it!=textureInfoIndexMap.end()){
        return it->second;
    }

    int textureInfoIndex = textures_.length();
    textures_.length(textureInfoIndex + 1 );
    TextureInfo& textureInfo = textures_[textureInfoIndex];
    textureInfoIndexMap[texture] = textureInfoIndex;

    SgImage* image = texture->image();
    unsigned char* pixels = image->pixels();
    unsigned long pixelsLength = image->width() * image->height() * image->numComponents();
    textureInfo.image.length( pixelsLength );
    for(unsigned long i = 0 ; i < pixelsLength ; i++ ){
        textureInfo.image[i] = pixels[i];
    }

    textureInfo.numComponents = image->numComponents();
    textureInfo.width = image->width();
    textureInfo.height = image->height();
    textureInfo.repeatS = texture->repeatS();
    textureInfo.repeatT = texture->repeatT();
    textureInfo.url = "";

    return textureInfoIndex;
}


void ShapeSetInfo_impl::setTexCoords(SgMesh* mesh, SgTextureTransform* textureTransform, AppearanceInfo& appInfo)
{
    Eigen::Affine2d m;
    if(textureTransform){
        Eigen::Translation<double, 2> translation =
                Eigen::Translation<double, 2>(textureTransform->translation());
        Eigen::Translation<double, 2> center =
                Eigen::Translation<double, 2>(textureTransform->center());
        Eigen::Translation<double, 2> centerInv =
                Eigen::Translation<double, 2>(-textureTransform->center());
        Eigen::DiagonalMatrix<double, 2> scale = Eigen::Scaling(textureTransform->scale());
        Eigen::Rotation2Dd rotation(textureTransform->rotation());
        m = centerInv * scale * rotation * center * translation;
    }else{
        m = Eigen::Affine2d::Identity();
    }
    for(int i=0,k=0; i<3; i++){
        for(int j=0; j<3; j++){
            appInfo.textransformMatrix[k++] = m(i,j);
        }
    }

    SgTexCoordArray& texCoords = *mesh->texCoords();
    int numCoords = texCoords.size();
    appInfo.textureCoordinate.length(numCoords * 2);
    for(int i=0, pos=0; i < numCoords; i++ ){
        Vector2 texCoord = m * texCoords[i].cast<double>();
        appInfo.textureCoordinate[pos++] = texCoord(0);
        appInfo.textureCoordinate[pos++] = texCoord(1);
    }

    setSequence( mesh->texCoordIndices(), appInfo.textureCoordIndices );

}
