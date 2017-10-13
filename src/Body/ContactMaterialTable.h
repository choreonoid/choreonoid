/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_CONTACT_MATERIAL_TABLE_H
#define CNOID_BODY_CONTACT_MATERIAL_TABLE_H

#include <cnoid/Referenced>

namespace cnoid {

class ContactMaterialTable : public Referenced
{
public:
    ContactMaterialTable();
    ~ContactMaterialTable();

    bool load(const std::string& filename);
};

typedef ref_ptr<ContactMaterialTable> ContactMaterialTablePtr;

}

#endif

