/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_MATERIAL_TABLE_H
#define CNOID_BODY_MATERIAL_TABLE_H

#include "ContactMaterial.h"
#include <cnoid/NullOut>
#include <cnoid/ClonableReferenced>
#include <functional>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MaterialTable : public ClonableReferenced
{
public:
    typedef std::function<ContactMaterial*(const ContactMaterial* org)> ContactMaterialCopyFactory;

    MaterialTable();
    virtual ~MaterialTable();

    /**
       \param cloneMap If this is specified, deep copy is performed. Shallow copy is performed for nullptr.
       \param factory A factory function for a custom ContactMaterial type.
    */
    MaterialTable* clone() const { return new MaterialTable(*this); }
    MaterialTable* clone(CloneMap& cloneMap, ContactMaterialCopyFactory factory = nullptr) const {
        return new MaterialTable(*this, &cloneMap, factory); }

    void clear();

    int maxMaterialId() const;
    int numMaterials() const;
    Material* material(int id) const;
    int numContactMaterials() const;
    ContactMaterial* contactMaterial(int id1, int id2) const;
    ContactMaterial* contactMaterial(const std::string& name1, const std::string& name2) const;

    void forEachMaterial(std::function<void(int id, Material* material)> func);
    void forEachMaterialPair(std::function<void(int id1, int id2, ContactMaterial* cm)> func);

    int addMaterial(Material* material);
    void setContactMaterial(int id1, int id2, ContactMaterial* cm);

    void merge(MaterialTable* table);
    
    bool load(const std::string& filename, std::ostream& os = nullout());

protected:
    MaterialTable(const MaterialTable& org, CloneMap* cloneMap, ContactMaterialCopyFactory factory);
    Referenced* doClone(CloneMap* cloneMap) const override;

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<MaterialTable> MaterialTablePtr;

}

#endif
