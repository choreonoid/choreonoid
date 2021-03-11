/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_MATERIAL_TABLE_H
#define CNOID_BODY_MATERIAL_TABLE_H

#include "ContactMaterial.h"
#include <cnoid/NullOut>
#include <functional>
#include "exportdecl.h"

namespace cnoid {

class CloneMap;

class CNOID_EXPORT MaterialTable : public Referenced
{
public:
    typedef std::function<ContactMaterial*(const ContactMaterial* org)> ContactMaterialCopyFactory;

    MaterialTable();
    virtual ~MaterialTable();
    //! The constructor to do shallow copy
    MaterialTable(const MaterialTable& org);
    //! The constructor to do deep copy with a custom ContactMaterial type
    MaterialTable(const MaterialTable& org, CloneMap& cloneMap, ContactMaterialCopyFactory factory = nullptr);

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

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<MaterialTable> MaterialTablePtr;

}

#endif
