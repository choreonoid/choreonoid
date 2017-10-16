/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "MaterialTable.h"
#include <cnoid/YAMLReader>
#include <cnoid/IdPair>
#include <boost/format.hpp>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using boost::format;

namespace cnoid {

class MaterialTableImpl
{
public:
    int numMaterials;
    vector<MaterialPtr> materials;

    typedef unordered_map<IdPair<>, ContactMaterialPtr> ContactMaterialMap;
    ContactMaterialMap contactMaterialMap;
    
    MaterialTableImpl();
    void addMaterial(Material* material);
    void loadMaterials(Mapping* topNode, std::ostream& os);
    void loadContactMaterials(Mapping* topNode, std::ostream& os);
    void setContactMaterialPairs(ContactMaterial* contactMaterial, const vector<int>& materialIndices, int index1);
};

}


MaterialTable::MaterialTable()
{
    impl = new MaterialTableImpl;
}


MaterialTableImpl::MaterialTableImpl()
{
    numMaterials = 0;
}


MaterialTable::MaterialTable(const MaterialTable& org)
{
    
}


MaterialTable::~MaterialTable()
{
    delete impl;
}


int MaterialTable::numMaterials() const
{
    return impl->numMaterials;
}


Material* MaterialTable::material(int id) const
{
    if(id < impl->materials.size()){
        id = 0; // default material
    }
    Material* material = impl->materials[id];
    if(!material){
        material = impl->materials[0]; // default material
    }
    return material;
}

        
ContactMaterial* MaterialTable::contactMaterial(int id1, int id2) const
{
    auto iter = impl->contactMaterialMap.find(IdPair<>(id1, id2));
    if(iter != impl->contactMaterialMap.end()){
        return iter->second;
    }
    return 0;
}


void MaterialTable::addMaterial(Material* material)
{
    impl->addMaterial(material);
}


void MaterialTableImpl::addMaterial(Material* material)
{
    if(material){
        const int id = Material::id(material->name());
        if(id >= materials.size()){
            materials.resize(id + 1);
            ++numMaterials;
        } else if(!materials[id]){
            ++numMaterials;
        }
        materials[id] = material;
    }
}


bool MaterialTable::load(const std::string& filename, std::ostream& os)
{
    bool result = false;
    
    try {
        YAMLReader reader;
        MappingPtr node = reader.loadDocument(filename)->toMapping();
        if(node){
            impl->loadMaterials(node, os);
            impl->loadContactMaterials(node, os);
            result = true;
        }
    } catch(const ValueNode::Exception& ex){
        os << ex.message();
    }

    os.flush();
    
    return result;
}


void MaterialTableImpl::loadMaterials(Mapping* topNode, std::ostream& os)
{
    std::unordered_set<string> names;

    auto& materialList = *topNode->findListing("materials");
    if(materialList.isValid()){
        for(int i=0; i < materialList.size(); ++i){
            Mapping* info = materialList[i].toMapping();
            MaterialPtr material = new Material(info);
            if(material->name().empty()){
                os << (format(_("Nameless material is defined at the %1%th material item.")) % i) << endl;
            } else {
                auto inserted = names.insert(material->name());
                if(!inserted.second){
                    os << (format(_("Material name \"%1%\" is duplicated. The defenition is updated with the last item."))
                           % material->name()) << endl;
                }
                addMaterial(material);
            }
        }
    }

    if(materials.empty()){
        materials.resize(1);
    }
    if(!materials[0]){
        Material* defaultMaterial = new Material;
        defaultMaterial->setName("default");
        defaultMaterial->setRoughness(0.5);
        defaultMaterial->setViscosity(0.0);
        materials[0] = defaultMaterial;
    }
}


void MaterialTableImpl::loadContactMaterials(Mapping* topNode, std::ostream& os)
{
    vector<int> materialIndices;
    
    auto& contactList = *topNode->findListing("contactMaterials");
    if(contactList.isValid()){
        for(int i=0; i < contactList.size(); ++i){
            Mapping* info = contactList[i].toMapping();
            
            auto materials = info->extract("materials");
            if(!materials){
                info->throwException(_("No material pair is specified"));
            } else {
                auto& materialList = *materials->toListing();
                if(materialList.size() < 2){
                    materialList.throwException(_("The counterpart of the material pair is lacking"));
                }
                materialIndices.clear();
                for(int i=0; i < materialList.size(); ++i){
                    int id = Material::id(materialList[i].toString());
                    materialIndices.push_back(id);
                }
                ContactMaterialPtr contactMaterial = new ContactMaterial(info);
                setContactMaterialPairs(contactMaterial, materialIndices, 0);
            }
        }
    }
}


void MaterialTableImpl::setContactMaterialPairs(ContactMaterial* contactMaterial, const vector<int>& materialIndices, int index1)
{
    const int id1 = materialIndices[index1];
    for(size_t index2 = index1 + 1; index2 < materialIndices.size(); ++index2){
        const int id2 = materialIndices[index2];
        IdPair<> idPair(id1, id2);
        contactMaterialMap[idPair] = contactMaterial;
    }
    if(materialIndices.size() - index1 > 2){
        setContactMaterialPairs(contactMaterial, materialIndices, index1 + 1);
    }
}
