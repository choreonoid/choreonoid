/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_MATERIAL_TABLE_H
#define CNOID_BODY_MATERIAL_TABLE_H

#include "ContactMaterial.h"
#include <cnoid/NullOut>
#include <functional>

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
    int addMaterial(Material* material);

    Material* material(int id) const;
    ContactMaterial* contactMaterial(int id1, int id2) const;
    void setContactMaterial(int id1, int id2, ContactMaterial* cm);
    void forEachMaterialPair(std::function<void(int id1, int id2, ContactMaterial* cm)> callback);

  private:
    MaterialTableImpl* impl;
};

typedef ref_ptr<MaterialTable> MaterialTablePtr;

}

#endif
