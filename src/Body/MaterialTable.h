/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_MATERIAL_TABLE_H
#define CNOID_BODY_MATERIAL_TABLE_H

#include "ContactMaterial.h"
#include <cnoid/NullOut>

namespace cnoid {

class MaterialTableImpl;

class CNOID_EXPORT MaterialTable : public Referenced
{
  public:
    MaterialTable();
    MaterialTable(const MaterialTable& org);
    ~MaterialTable();

    int numMaterials() const;
    Material* material(int id) const;
    ContactMaterial* contactMaterial(int id1, int id2) const;

    bool load(const std::string& filename, std::ostream& os = nullout());
    void addMaterial(Material* material);

  private:
    MaterialTableImpl* impl;
};

typedef ref_ptr<MaterialTable> MaterialTablePtr;

}

#endif
