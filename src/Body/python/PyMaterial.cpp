#include "../Material.h"
#include "../ContactMaterial.h"
#include "../MaterialTable.h"
#include <cnoid/PyUtil>

using namespace std;
using namespace cnoid;
namespace nb = nanobind;

namespace cnoid {

void exportPyMaterial(nb::module_& m)
{
    nb::class_<Material, Referenced>(m, "Material")
        .def(nb::init<>())
        .def(nb::init<const Mapping*>())
        .def(nb::init<const Material&>())
        .def_static("idOfName", [](const std::string& name){ return Material::idOfName(name); })
        .def_static("nameOfId", [](int id){ return Material::nameOfId(id); })
        .def_prop_rw("name", (const std::string&(Material::*)()const) &Material::name, &Material::setName)
        .def("setName", &Material::setName)
        .def_prop_rw("roughness", &Material::roughness, &Material::setRoughness)
        .def("setRoughness", &Material::setRoughness)
        .def_prop_rw("viscosity", &Material::viscosity, &Material::setViscosity)
        .def("setViscosity", &Material::setViscosity)
        .def_prop_rw("stiffness", &Material::stiffness, &Material::setStiffness)
        .def("setStiffness", &Material::setStiffness)
        .def_prop_rw("damping", &Material::damping, &Material::setDamping)
        .def("setDamping", &Material::setDamping)
        .def_prop_ro("info", (Mapping*(Material::*)()) &Material::info)
        .def("getInfo",
             [](Material& self, const std::string& key, bool defaultValue){
                 return self.info(key, defaultValue); })
        .def("getInfo",
             [](Material& self, const std::string& key, int defaultValue){
                 return self.info(key, defaultValue); })
        .def("getInfo",
             [](Material& self, const std::string& key, double defaultValue){
                 return self.info(key, defaultValue); })
        .def("resetInfo", &Material::resetInfo)
        ;

    nb::class_<ContactMaterial, Referenced>(m, "ContactMaterial")
        .def(nb::init<>())
        .def(nb::init<const Mapping*>())
        .def(nb::init<const ContactMaterial&>())
        .def_prop_rw("friction", &ContactMaterial::friction, &ContactMaterial::setFriction)
        .def("setFriction", &ContactMaterial::setFriction)
        .def_prop_rw("staticFriction", &ContactMaterial::staticFriction, &ContactMaterial::setStaticFriction)
        .def("setStaticFriction", &ContactMaterial::setStaticFriction)
        .def_prop_rw("dynamicFriction", &ContactMaterial::dynamicFriction, &ContactMaterial::setDynamicFriction)
        .def("setDynamicFriction", &ContactMaterial::setDynamicFriction)
        .def_prop_rw("restitution", &ContactMaterial::restitution, &ContactMaterial::setRestitution)
        .def("setRestitution", &ContactMaterial::setRestitution)
        .def_prop_ro("info", (Mapping*(ContactMaterial::*)()) &ContactMaterial::info)
        .def("getInfo",
             [](ContactMaterial& self, const std::string& key, bool defaultValue){
                 return self.info(key, defaultValue); })
        .def("getInfo",
             [](ContactMaterial& self, const std::string& key, int defaultValue){
                 return self.info(key, defaultValue); })
        .def("getInfo",
             [](ContactMaterial& self, const std::string& key, double defaultValue){
                 return self.info(key, defaultValue); })
        ;

    nb::class_<MaterialTable, Referenced>(m, "MaterialTable")
        .def(nb::init<>())
        .def(nb::init<const MaterialTable&>())
        .def("load", [](MaterialTable& self, const std::string& filename){ return self.load(filename); })
        .def_prop_ro("maxMaterialId", &MaterialTable::maxMaterialId)
        .def("material", &MaterialTable::material)
        .def("contactMaterial",
             (ContactMaterial*(MaterialTable::*)(int,int)const) &MaterialTable::contactMaterial)
        .def("contactMaterial",
             (ContactMaterial*(MaterialTable::*)(const std::string&,const std::string&)const)
             &MaterialTable::contactMaterial)
        .def("forEachMaterial", &MaterialTable::forEachMaterial)
        .def("forEachMaterialPair", &MaterialTable::forEachMaterialPair)
        .def("addMaterial", &MaterialTable::addMaterial)
        .def("setContactMaterial", &MaterialTable::setContactMaterial)
        ;
}

}
