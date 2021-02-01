#include "../Material.h"
#include "../ContactMaterial.h"
#include "../MaterialTable.h"
#include <cnoid/PyUtil>

using namespace std;
using namespace cnoid;
namespace py = pybind11;

namespace cnoid {

void exportPyMaterial(py::module& m)
{
    py::class_<Material, MaterialPtr, Referenced>(m, "Material")
        .def(py::init<>())
        .def(py::init<const Mapping*>())
        .def(py::init<const Material&>())
        .def_static(
            //"idOfName", [](py::object, const std::string& name){ return Material::idOfName(name); })
            "idOfName", [](const std::string& name){ return Material::idOfName(name); })
        .def_static(
            "nameOfId", [](py::object, int id){ return Material::nameOfId(id); })
        .def_property("name", (const std::string&(Material::*)()const) &Material::name, &Material::setName)
        .def("setName", &Material::setName)
        .def_property("roughness", &Material::roughness, &Material::setRoughness)
        .def("setRoughness", &Material::setRoughness)
        .def_property("viscosity", &Material::viscosity, &Material::setViscosity)
        .def("setViscosity", &Material::setViscosity)
        .def_property_readonly("info", (Mapping*(Material::*)()) &Material::info)
        .def("getInfo",
             [](Material& self, const std::string& key, bool defaultValue){
                 return self.info(key, defaultValue);
             })
        .def("getInfo",
             [](Material& self, const std::string& key, int defaultValue){
                 return self.info(key, defaultValue);
             })
        .def("getInfo",
             [](Material& self, const std::string& key, double defaultValue){
                 return self.info(key, defaultValue);
             })
        .def("resetInfo", &Material::resetInfo)
        ;

    py::class_<ContactMaterial, ContactMaterialPtr, Referenced>(m, "ContactMaterial")
        .def(py::init<>())
        .def(py::init<const Mapping*>())
        .def(py::init<const ContactMaterial&>())
        .def_property("friction", &ContactMaterial::friction, &ContactMaterial::setFriction)
        .def("setFriction", &ContactMaterial::setFriction)
        .def_property("staticFriction", &ContactMaterial::staticFriction, &ContactMaterial::setStaticFriction)
        .def("setStaticFriction", &ContactMaterial::setStaticFriction)
        .def_property("dynamicFriction", &ContactMaterial::dynamicFriction, &ContactMaterial::setDynamicFriction)
        .def("setDynamicFriction", &ContactMaterial::setDynamicFriction)
        .def_property("restitution", &ContactMaterial::restitution, &ContactMaterial::setRestitution)
        .def("setRestitution", &ContactMaterial::setRestitution)
        .def_property_readonly("info", (Mapping*(ContactMaterial::*)()) &ContactMaterial::info)
        .def("getInfo",
             [](ContactMaterial& self, const std::string& key, bool defaultValue){
                 return self.info(key, defaultValue);
             })
        .def("getInfo",
             [](ContactMaterial& self, const std::string& key, int defaultValue){
                 return self.info(key, defaultValue);
             })
        .def("getInfo",
             [](ContactMaterial& self, const std::string& key, double defaultValue){
                 return self.info(key, defaultValue);
             })
        ;
        
    py::class_<MaterialTable, MaterialTablePtr, Referenced>(m, "MaterialTable")
        .def(py::init<>())
        .def(py::init<const MaterialTable&>())
        .def("load", [](MaterialTable& self, const std::string& filename){ return self.load(filename); })
        .def_property_readonly("maxMaterialId", &MaterialTable::maxMaterialId)
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
