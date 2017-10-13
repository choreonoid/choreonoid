/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_CONTACT_MATERIAL_TABLE_H
#define CNOID_BODY_CONTACT_MATERIAL_TABLE_H

#include <cnoid/Referenced>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ContactMaterialTable : public Referenced
{
public:
    ContactMaterialTable();
    ~ContactMaterialTable();

    bool load(const std::string& filename);
};

typedef ref_ptr<ContactMaterialTable> ContactMaterialTablePtr;

}

#endif

