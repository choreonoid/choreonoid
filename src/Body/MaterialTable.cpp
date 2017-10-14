/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "MaterialTable.h"

using namespace cnoid;

namespace cnoid {

class MaterialTableImpl
{
public:
    MaterialTableImpl();
};

}


MaterialTable::MaterialTable()
{
    impl = new MaterialTableImpl;
}


MaterialTableImpl::MaterialTableImpl()
{

}


MaterialTable::~MaterialTable()
{
    delete impl;
}


bool MaterialTable::load(const std::string& filename)
{
    return false;
}


Material* MaterialTable::findMaterial(int id) const
{
    return 0;
}

        
ContactMaterial* MaterialTable::findContactMaterial(const Material* m1, const Material* m2) const
{
    return 0;
}
