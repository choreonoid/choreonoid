/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "MaterialTable.h"
#include <cnoid/YAMLReader>
#include <cnoid/IdPair>
#include <cnoid/CloneMap>
#include <fmt/format.h>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <ostream>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace cnoid {

class MaterialTable::Impl
{
public:
    /**
       It may be better to use a map type as a containder of materials to improve the efficiency when
       a large number of materials are defined in the system but only a few of them are stored in a table.
    */
    vector<MaterialPtr> materials;

    typedef unordered_map<IdPair<>, ContactMaterialPtr> ContactMaterialMap;
    ContactMaterialMap contactMaterialMap;
    
    Impl();
    Impl(const Impl& org);
    Impl(const Impl& org, CloneMap* cloneMap, MaterialTable::ContactMaterialCopyFactory factory);
    void setDefaultMaterial();
    int addMaterial(Material* material);
    void merge(MaterialTable* table);
    void loadMaterials(Mapping* topNode, std::ostream& os);
    void loadContactMaterials(Mapping* topNode, std::ostream& os);
};

}


MaterialTable::MaterialTable()
{
    impl = new Impl;
}


MaterialTable::Impl::Impl()
{

}


MaterialTable::MaterialTable(const MaterialTable& org)
{
    impl = new Impl(*org.impl);
}


MaterialTable::Impl::Impl(const Impl& org)
    : Impl(org, nullptr, nullptr)
{

}

    
MaterialTable::MaterialTable(const MaterialTable& org, CloneMap& cloneMap, ContactMaterialCopyFactory factory)
{
    impl = new Impl(*org.impl, &cloneMap, factory);
}


MaterialTable::Impl::Impl(const Impl& org, CloneMap* cloneMap, MaterialTable::ContactMaterialCopyFactory factory)
{
    materials.reserve(org.materials.size());

    for(auto material : org.materials){
        if(cloneMap){
            material = cloneMap->getClone<Material>(
                material, [](const Material* org){ return new Material(*org); });
        }
        materials.push_back(material);
    }
    
    if(!org.contactMaterialMap.empty()){
        for(auto& kv : org.contactMaterialMap){
            auto idPair = kv.first;
            ContactMaterial* src = kv.second;
            ContactMaterial* copy = nullptr;
            if(!cloneMap){
                copy = src;
            } else {
                if(!factory){
                    copy = cloneMap->getClone<ContactMaterial>(
                        src, [](const ContactMaterial* org){ return new ContactMaterial(*org); });
                } else {
                    copy = cloneMap->getClone(src, factory);
                }
            }
            contactMaterialMap.insert(ContactMaterialMap::value_type(idPair, copy));
        }
    }
}


MaterialTable::~MaterialTable()
{
    delete impl;
}


void MaterialTable::clear()
{
    impl->materials.clear();
    impl->contactMaterialMap.clear();
}


int MaterialTable::maxMaterialId() const
{
    return impl->materials.size() - 1;
}


int MaterialTable::numMaterials() const
{
    int n = 0;
    for(auto& material : impl->materials){
        if(material){
            ++n;
        }
    }
    return n;
}


Material* MaterialTable::material(int id) const
{
    if(id >= static_cast<int>(impl->materials.size())){
        id = 0; // default material
    }
    Material* material = impl->materials[id];
    if(!material){
        // Return the default material for an empty material
        material = impl->materials[0];
        if(!material){
            impl->setDefaultMaterial();
            material = impl->materials[0];
        }
    }
    return material;
}


void MaterialTable::Impl::setDefaultMaterial()
{
    auto defaultMaterial = new Material;
    defaultMaterial->setName("default");
    if(materials.empty()){
        materials.push_back(defaultMaterial);
    } else {
        materials[0] = defaultMaterial;
    }
}


int MaterialTable::numContactMaterials() const
{
    return impl->contactMaterialMap.size();
}

        
ContactMaterial* MaterialTable::contactMaterial(int id1, int id2) const
{
    auto iter = impl->contactMaterialMap.find(IdPair<>(id1, id2));
    if(iter != impl->contactMaterialMap.end()){
        return iter->second;
    }
    return nullptr;
}


ContactMaterial* MaterialTable::contactMaterial(const std::string& name1, const std::string& name2) const
{
    return contactMaterial(Material::idOfName(name1), Material::idOfName(name2));
}


void MaterialTable::forEachMaterial(std::function<void(int id, Material* material)> func)
{
    auto& materials = impl->materials;
    for(size_t i=0; i < materials.size(); ++i){
        Material* material = materials[i];
        if(material){
            func(i, material);
        }
    }
}


void MaterialTable::forEachMaterialPair(std::function<void(int id1, int id2, ContactMaterial* cm)> func)
{
    auto iter = impl->contactMaterialMap.begin();
    while(iter != impl->contactMaterialMap.end()){
        const IdPair<>& idPair = iter->first;
        ContactMaterialPtr& cm = iter->second;
        func(idPair(0), idPair(1), cm);
        ++iter;
    }
}


/**
   \return Material ID
*/
int MaterialTable::addMaterial(Material* material)
{
    return impl->addMaterial(material);
}


int MaterialTable::Impl::addMaterial(Material* material)
{
    int id = -1;
    
    if(material){
        id = Material::idOfName(material->name());
        if(static_cast<int>(materials.size()) <= id){
            materials.resize(id + 1);
        }
        materials[id] = material;
    }

    return id;
}


void MaterialTable::setContactMaterial(int id1, int id2, ContactMaterial* cm)
{
    impl->contactMaterialMap[IdPair<>(id1, id2)] = cm;
}


void MaterialTable::merge(MaterialTable* table)
{
    impl->merge(table);
}


void MaterialTable::Impl::merge(MaterialTable* table)
{
    for(auto& material : table->impl->materials){
        if(material){
            addMaterial(material);
        }
    }
    for(auto& kv : table->impl->contactMaterialMap){
        contactMaterialMap[kv.first] = kv.second;
    }
}


bool MaterialTable::load(const std::string& filename, std::ostream& os)
{
    bool result = false;
    
    try {
        YAMLReader reader;
        MappingPtr node = reader.loadDocument(filename)->toMapping();
        if(node){
            clear();
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


void MaterialTable::Impl::loadMaterials(Mapping* topNode, std::ostream& os)
{
    std::unordered_set<string> names;

    auto& materialList = *topNode->findListing("materials");
    if(materialList.isValid()){
        materials.reserve(materialList.size());
        for(size_t i=0; i < materialList.size(); ++i){
            Mapping* info = materialList[i].toMapping();
            MaterialPtr material = new Material(info);
            if(material->name().empty()){
                os << format(_("Nameless material is defined at the {}th material item."), i) << endl;
            } else {
                auto inserted = names.insert(material->name());
                if(!inserted.second){
                    os << format(_("Material name \"{}\" is duplicated. The defenition is updated with the last item."),
                            material->name()) << endl;
                }
                addMaterial(material);
            }
        }
    }
}


void MaterialTable::Impl::loadContactMaterials(Mapping* topNode, std::ostream& os)
{
    vector<int> materialIndices;
    
    auto contactList = topNode->findListing("contact_materials");
    if(!contactList->isValid()){
        contactList = topNode->findListing("contactMaterials");
    }
        
    if(contactList->isValid()){
        for(int i=0; i < contactList->size(); ++i){
            Mapping* info = (*contactList)[i].toMapping();
            
            auto materials = info->extract("materials");
            if(!materials){
                info->throwException(_("No material pair is specified"));
            } else {
                auto& materialList = *materials->toListing();
                if(materialList.size() < 2){
                    materialList.throwException(_("The counterpart of the material pair is lacking"));
                }
                materialIndices.clear();
                for(auto& material : materialList){
                    int id = Material::idOfName(material->toString());
                    materialIndices.push_back(id);
                }
                ContactMaterialPtr contactMaterial = new ContactMaterial(info);

                for(size_t j = 0; j < materialIndices.size() - 1; ++j){
                    for(size_t k = j + 1; k < materialIndices.size(); ++k){
                        int id1 = materialIndices[j];
                        int id2 = materialIndices[k];
                        contactMaterialMap[IdPair<>(id1, id2)] = contactMaterial;
                    }
                }
            }
        }
    }
}
