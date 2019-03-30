/**
   \file
   \author Shizuko Hattori
*/

#ifndef CNOID_OPENHRP_MODELLOADER_SHAPESETINFO_IMPL_H
#define CNOID_OPENHRP_MODELLOADER_SHAPESETINFO_IMPL_H

#include <cnoid/corba/OpenHRPModelLoader/ModelLoader.hh>
#include <cnoid/SceneDrawables>
#include <cnoid/MeshExtractor>
#include <map>

namespace cnoid {

class ShapeSetInfo_impl : public virtual POA_OpenHRP::ShapeSetInfo
{
public:

    ShapeSetInfo_impl(PortableServer::POA_ptr poa);
    virtual ~ShapeSetInfo_impl();
    virtual PortableServer::POA_ptr _default_POA();

    virtual OpenHRP::ShapeInfoSequence* shapes() override;
    virtual OpenHRP::AppearanceInfoSequence* appearances() override;
    virtual OpenHRP::MaterialInfoSequence* materials() override;
    virtual OpenHRP::TextureInfoSequence* textures() override;

protected :
    void setShapeIndices(SgNode* node, OpenHRP::TransformedShapeIndexSequence& shapeIndices);

private:

    PortableServer::POA_var poa;

    OpenHRP::ShapeInfoSequence  shapes_;
    OpenHRP::AppearanceInfoSequence appearances_;
    OpenHRP::MaterialInfoSequence materials_;
    OpenHRP::TextureInfoSequence textures_;

    typedef std::map<SgShape*, int> SgShapeToShapeInfoIndexMap;
    SgShapeToShapeInfoIndexMap shapeInfoIndexMap;
    typedef std::map<SgMaterial*, int> SgMaterialToMaterialInfoIndexMap;
    SgMaterialToMaterialInfoIndexMap materialInfoIndexMap;
    typedef std::map<SgTexture*, int> SgTextureToTextureInfoIndexMap;
    SgTextureToTextureInfoIndexMap textureInfoIndexMap;

    void createTransformedShapeIndex(MeshExtractor* extractor, OpenHRP::TransformedShapeIndexSequence& shapeIndices);
    void setMesh(SgMesh* mesh, OpenHRP::ShapeInfo& shapeInfo);
    void setPrimitive(SgMesh* mesh, OpenHRP::ShapeInfo& shapeInfo);
    int setAppearance(SgShape* shape);
    void setColors(SgMesh* mesh, OpenHRP::AppearanceInfo& appInfo);
    void setNormals(SgMesh* mesh, OpenHRP::AppearanceInfo& appInfo);
    int setMaterial(SgMaterial* material);
    int setTexture(SgTexture* texture);
    void setTexCoords(SgMesh* mesh, SgTextureTransform* textureTransform, OpenHRP::AppearanceInfo& appInfo);
};

}

#endif
