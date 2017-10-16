/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_MATERIAL_TABLE_H
#define CNOID_BODY_MATERIAL_TABLE_H

#include "ContactMaterial.h"

namespace cnoid {

class MaterialTableImpl;

class CNOID_EXPORT MaterialTable : public Referenced
{
  public:
    MaterialTable();
    ~MaterialTable();

    bool load(const std::string& filename);

    Material* findMaterial(int id) const;
    ContactMaterial* findContactMaterial(const Material* m1, const Material* m2) const;

  private:
    MaterialTableImpl* impl;
};

typedef ref_ptr<MaterialTable> MaterialTablePtr;

}

#endif
