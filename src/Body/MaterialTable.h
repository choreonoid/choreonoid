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

class MaterialTableImpl;

class CNOID_EXPORT MaterialTable : public Referenced
{
  public:
    typedef std::function<ContactMaterial*(const ContactMaterial* org)> ContactMaterialCopyFactory;

    MaterialTable();
    MaterialTable(const MaterialTable& org);
    MaterialTable(const MaterialTable& org, ContactMaterialCopyFactory factory);
    virtual ~MaterialTable();

    bool load(const std::string& filename, std::ostream& os = nullout());

    int maxMaterialId() const;
    Material* material(int id) const;
    ContactMaterial* contactMaterial(int id1, int id2) const;

    void forEachMaterial(std::function<void(int id, Material* material)> func);
    void forEachMaterialPair(std::function<void(int id1, int id2, ContactMaterial* cm)> func);

    int addMaterial(Material* material);
    void setContactMaterial(int id1, int id2, ContactMaterial* cm);

  private:
    MaterialTableImpl* impl;
};

typedef ref_ptr<MaterialTable> MaterialTablePtr;

}

#endif
