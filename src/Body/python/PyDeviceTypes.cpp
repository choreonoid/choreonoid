#include "PyDeviceList.h"
#include "../Device.h"
#include "../Link.h"
#include "../ForceSensor.h"
#include "../RateGyroSensor.h"
#include "../AccelerationSensor.h"
#include "../Imu.h"
#include "../VisionSensor.h"
#include "../Camera.h"
#include "../RangeCamera.h"
#include "../RangeSensor.h"
#include "../Light.h"
#include "../PointLight.h"
#include "../SpotLight.h"
#include <cnoid/PyUtil>
#include <cnoid/PyEigenTypes>
#include <cnoid/PyNumpyHelper>

using namespace std;
using namespace cnoid;
namespace nb = nanobind;

namespace cnoid {

void exportPyDeviceTypes(nb::module_& m)
{
    nb::class_<Device, Referenced>(m, "Device")
        .def("__repr__",
             [](const Device& self){
                 return string("<cnoid.Body.") + self.typeName() + " named '" + self.name() + "'>"; })
        .def_prop_ro("typeName", &Device::typeName)
        .def_prop_rw("index", &Device::index, &Device::setIndex)
        .def("setIndex", &Device::setIndex)
        .def_prop_rw("id", &Device::id, &Device::setId)
        .def("setId", &Device::setId)
        .def_prop_rw("name", &Device::name, &Device::setName)
        .def("setName", &Device::setName)
        .def("link", (Link*(Device::*)()) &Device::link)
        .def("setLink", &Device::setLink)
        .def("clone", (Device*(Device::*)()const) &Device::clone)
        .def("clearState", &Device::clearState)
        .def("hasStateOnly", &Device::hasStateOnly)
        .def_prop_rw(
            "T_local",
            [](Device& self) -> Isometry3::MatrixType& { return self.T_local().matrix(); },
            [](Device& self, const python::Matrix4RMArg& T){ self.T_local() = T.value; })
        ;

    nb::class_<ForceSensor, Device>(m, "ForceSensor")
        .def(nb::init<>())
        .def_prop_rw(
            "wrench",
            [](ForceSensor& self){ return python::eigenVectorView(self.wrench(), nb::find(&self)); },
            [](ForceSensor& self, const python::Vector6Arg& F){ self.wrench() = F.value; })
        .def_prop_rw(
            "maxWrench",
            [](ForceSensor& self){ return python::eigenVectorView(self.maxWrench(), nb::find(&self)); },
            [](ForceSensor& self, const python::Vector6Arg& F){ self.maxWrench() = F.value; })
        .def_prop_rw(
            "F",
            [](ForceSensor& self){ return python::eigenVectorView(self.F(), nb::find(&self)); },
            [](ForceSensor& self, const python::Vector6Arg& F){ self.F() = F.value; })
        .def_prop_rw(
            "F_max",
            [](ForceSensor& self){ return python::eigenVectorView(self.F_max(), nb::find(&self)); },
            [](ForceSensor& self, const python::Vector6Arg& F){ self.F_max() = F.value; })
        .def_prop_rw(
            "force",
            [](ForceSensor& self){ return python::eigenVectorView(self.force(), nb::find(&self)); },
            [](ForceSensor& self, const python::Vector3Arg& f){ self.force() = f.value; })
        .def_prop_rw(
            "f",
            [](ForceSensor& self){ return python::eigenVectorView(self.f(), nb::find(&self)); },
            [](ForceSensor& self, const python::Vector3Arg& f){ self.f() = f.value; })
        .def_prop_rw(
            "torque",
            [](ForceSensor& self){ return python::eigenVectorView(self.torque(), nb::find(&self)); },
            [](ForceSensor& self, const python::Vector3Arg& tau){ self.torque() = tau.value; })
        .def_prop_rw(
            "tau",
            [](ForceSensor& self){ return python::eigenVectorView(self.tau(), nb::find(&self)); },
            [](ForceSensor& self, const python::Vector3Arg& tau){ self.tau() = tau.value; })
        ;

    nb::class_<RateGyroSensor, Device>(m, "RateGyroSensor")
        .def(nb::init<>())
        .def_prop_rw(
            "w",
            [](RateGyroSensor& self){ return python::eigenVectorView(self.w(), nb::find(&self)); },
            [](RateGyroSensor& self, const python::Vector3Arg& w){ self.w() = w.value; })
        .def_prop_rw(
            "w_max",
            [](RateGyroSensor& self){ return python::eigenVectorView(self.w_max(), nb::find(&self)); },
            [](RateGyroSensor& self, const python::Vector3Arg& w){ self.w_max() = w.value; })
        ;

    nb::class_<AccelerationSensor, Device>(m, "AccelerationSensor")
        .def(nb::init<>())
        .def_prop_rw(
            "dv",
            [](AccelerationSensor& self){ return python::eigenVectorView(self.dv(), nb::find(&self)); },
            [](AccelerationSensor& self, const python::Vector3Arg& dv){ self.dv() = dv.value; })
        .def_prop_rw(
            "dv_max",
            [](AccelerationSensor& self){ return python::eigenVectorView(self.dv_max(), nb::find(&self)); },
            [](AccelerationSensor& self, const python::Vector3Arg& dv){ self.dv_max() = dv.value; })
        ;

    nb::class_<Imu, Device>(m, "Imu")
        .def(nb::init<>())
        .def_prop_rw(
            "w",
            [](Imu& self){ return python::eigenVectorView(self.w(), nb::find(&self)); },
            [](Imu& self, const python::Vector3Arg& w){ self.w() = w.value; })
        .def_prop_rw(
            "w_max",
            [](Imu& self){ return python::eigenVectorView(self.w_max(), nb::find(&self)); },
            [](Imu& self, const python::Vector3Arg& w){ self.w_max() = w.value; })
        .def_prop_rw(
            "dv",
            [](Imu& self){ return python::eigenVectorView(self.dv(), nb::find(&self)); },
            [](Imu& self, const python::Vector3Arg& dv){ self.dv() = dv.value; })
        .def_prop_rw(
            "dv_max",
            [](Imu& self){ return python::eigenVectorView(self.dv_max(), nb::find(&self)); },
            [](Imu& self, const python::Vector3Arg& dv){ self.dv_max() = dv.value; })
        ;

    nb::class_<VisionSensor, Device> visionSensor(m, "VisionSensor");
    visionSensor
        .def_prop_rw("on", (bool(VisionSensor::*)()const) &VisionSensor::on, (void(VisionSensor::*)(bool)) &VisionSensor::on)
        .def_prop_rw(
            "opticalFrameRotation",
            [](VisionSensor& self){ return python::eigenMatrixView(self.opticalFrameRotation(), nb::find(&self)); },
            [](VisionSensor& self, const python::Matrix3RMArg& R){ self.setOpticalFrameRotation(R.value); })
        .def("setOpticalFrameRotation", [](VisionSensor& self, const python::Matrix3RMArg& R){ self.setOpticalFrameRotation(R.value); })
        .def("setOpticalFrame", &VisionSensor::setOpticalFrame)
        .def_prop_rw("frameRate", &VisionSensor::frameRate, &VisionSensor::setFrameRate)
        .def("setFrameRate", &VisionSensor::setFrameRate)
        .def_prop_rw("delay", &VisionSensor::delay, &VisionSensor::setDelay)
        .def("setDelay", &VisionSensor::setDelay)
        ;

    nb::enum_<VisionSensor::OpticalFrameType>(visionSensor, "OpticalFrameType", nb::is_arithmetic())
        .value("GL", VisionSensor::GL)
        .value("CV", VisionSensor::CV)
        .value("Robotics", VisionSensor::Robotics)
        .export_values();

    nb::class_<Camera, VisionSensor> camera(m, "Camera");
    camera
        .def(nb::init<>())
        .def_prop_rw("imageType", &Camera::imageType, &Camera::setImageType)
        .def("setImageType", &Camera::setImageType)
        .def_prop_rw("lensType", &Camera::lensType, &Camera::setLensType)
        .def("setLensType", &Camera::setLensType)
        .def_prop_rw("nearClipDistance", &Camera::nearClipDistance, &Camera::setNearClipDistance)
        .def("setNearClipDistance", &Camera::setNearClipDistance)
        .def_prop_rw("farClipDistance", &Camera::farClipDistance, &Camera::setFarClipDistance)
        .def("setFarClipDistance", &Camera::setFarClipDistance)
        .def_prop_rw("fieldOfView", &Camera::fieldOfView, &Camera::setFieldOfView)
        .def("setFieldOfView", &Camera::setFieldOfView)
        .def_prop_rw("horizontalFieldOfView", &Camera::horizontalFieldOfView, &Camera::setHorizontalFieldOfView)
        .def("setHorizontalFieldOfView", &Camera::setHorizontalFieldOfView)
        .def("setResolution", &Camera::setResolution)
        .def_prop_rw("resolutionX", &Camera::resolutionX, &Camera::setResolutionX)
        .def("setResolutionX", &Camera::setResolutionX)
        .def_prop_rw("resolutionY", &Camera::resolutionY, &Camera::setResolutionY)
        .def("setResolutionY", &Camera::setResolutionY)
        .def_prop_rw("imageStateClonable", &Camera::isImageStateClonable, &Camera::setImageStateClonable)
        ;

    nb::enum_<Camera::ImageType>(camera, "ImageType", nb::is_arithmetic())
        .value("NO_IMAGE", Camera::NO_IMAGE)
        .value("COLOR_IMAGE", Camera::COLOR_IMAGE)
        .value("GRAYSCALE_IMAGE", Camera::GRAYSCALE_IMAGE)
        .export_values();

    nb::enum_<Camera::LensType>(camera, "LensType", nb::is_arithmetic())
        .value("NORMAL_LENS", Camera::NORMAL_LENS)
        .value("FISHEYE_LENS", Camera::FISHEYE_LENS)
        .value("DUAL_FISHEYE_LENS", Camera::DUAL_FISHEYE_LENS)
        .export_values();

    nb::class_<RangeCamera, Camera>(m, "RangeCamera")
        .def(nb::init<>())
        .def_prop_ro("numPoints", &RangeCamera::numPoints)
        .def_prop_ro(
            "points",
            [](RangeCamera& self) -> nb::object {
                const auto& points = self.points();
                if(points.empty()){
                    return python::bufferView2d<float>(nullptr, 0, 3, nb::find(&self));
                }
                return python::bufferView2d<float>(
                    const_cast<float*>(points[0].data()), points.size(), 3, nb::find(&self));
            })
        .def_prop_rw("maxDistance", &RangeCamera::maxDistance, &RangeCamera::setMaxDistance)
        .def("setMaxDistance", &RangeCamera::setMaxDistance)
        .def_prop_rw("minDistance", &RangeCamera::minDistance, &RangeCamera::setMinDistance)
        .def("setMinDistance", &RangeCamera::setMinDistance)
        .def_prop_rw("organized", &RangeCamera::isOrganized, &RangeCamera::setOrganized)
        .def_prop_rw("dense", &RangeCamera::isDense, &RangeCamera::setDense)
        .def_prop_rw("detectionRate", &RangeCamera::detectionRate, &RangeCamera::setDetectionRate)
        .def_prop_rw("errorDeviation", &RangeCamera::errorDeviation, &RangeCamera::setErrorDeviation)
        ;

    nb::class_<RangeSensor, VisionSensor>(m, "RangeSensor")
        .def(nb::init<>())
        .def_prop_ro("minYawAngle", &RangeSensor::minYawAngle)
        .def_prop_ro("maxYawAngle", &RangeSensor::maxYawAngle)
        .def_prop_ro("yawRange", &RangeSensor::yawRange)
        .def("setYawRange", (void(RangeSensor::*)(double, double)) &RangeSensor::setYawRange)
        .def_prop_rw("numYawSamples", &RangeSensor::numYawSamples, &RangeSensor::setNumYawSamples)
        .def_prop_rw("yawStep", &RangeSensor::yawStep, &RangeSensor::setYawStep)
        .def_prop_ro("minPitchAngle", &RangeSensor::minPitchAngle)
        .def_prop_ro("maxPitchAngle", &RangeSensor::maxPitchAngle)
        .def_prop_ro("pitchRange", &RangeSensor::pitchRange)
        .def("setPitchRange", (void(RangeSensor::*)(double, double)) &RangeSensor::setPitchRange)
        .def_prop_rw("numPitchSamples", &RangeSensor::numPitchSamples, &RangeSensor::setNumPitchSamples)
        .def_prop_rw("pitchStep", &RangeSensor::pitchStep, &RangeSensor::setPitchStep)
        .def_prop_rw("maxDistance", &RangeSensor::maxDistance, &RangeSensor::setMaxDistance)
        .def_prop_rw("minDistance", &RangeSensor::minDistance, &RangeSensor::setMinDistance)
        .def_prop_rw("scanRate", &RangeSensor::scanRate, &RangeSensor::setScanRate)
        .def_prop_rw("detectionRate", &RangeSensor::detectionRate, &RangeSensor::setDetectionRate)
        .def_prop_rw("errorDeviation", &RangeSensor::errorDeviation, &RangeSensor::setErrorDeviation)
        .def_prop_ro(
            "rangeData",
            [](RangeSensor& self) -> nb::object {
                auto& rangeData = self.rangeData();
                return python::bufferView1d<double>(
                    rangeData.empty() ? nullptr : rangeData.data(), rangeData.size(), nb::find(&self));
            })
        .def("clearRangeData", &RangeSensor::clearRangeData)
        ;

    nb::class_<Light, Device>(m, "Light")
        .def_prop_rw("on", (bool(Light::*)()const) &Light::on, (void(Light::*)(bool)) &Light::on)
        .def_prop_rw(
            "color",
            [](Light& self){ return self.color().cast<double>().eval(); },
            [](Light& self, const python::Vector3Arg& c){ self.setColor(c.value); })
        .def("setColor", [](Light& self, const python::Vector3Arg& c){ self.setColor(c.value); })
        .def_prop_rw("intensity", &Light::intensity, &Light::setIntensity)
        .def("setIntensity", &Light::setIntensity)
        ;

    nb::class_<PointLight, Light>(m, "PointLight")
        .def(nb::init<>())
        .def_prop_rw("constantAttenuation", &PointLight::constantAttenuation, &PointLight::setConstantAttenuation)
        .def("setConstantAttenuation", &PointLight::setConstantAttenuation)
        .def_prop_rw("linearAttenuation", &PointLight::linearAttenuation, &PointLight::setLinearAttenuation)
        .def("setLinearAttenuation", &PointLight::setLinearAttenuation)
        .def_prop_rw("quadraticAttenuation", &PointLight::quadraticAttenuation, &PointLight::setQuadraticAttenuation)
        .def("setQuadraticAttenuation", &PointLight::setQuadraticAttenuation)
        ;

    nb::class_<SpotLight, PointLight>(m, "SpotLight")
        .def(nb::init<>())
        .def_prop_rw("direction", &SpotLight::direction,
                     [](SpotLight& self, const python::Vector3Arg& d){ self.setDirection(d.value); })
        .def("setDirection", [](SpotLight& self, const python::Vector3Arg& d){ self.setDirection(d.value); })
        .def_prop_rw("beamWidth", &SpotLight::beamWidth, &SpotLight::setBeamWidth)
        .def("setBeamWidth", &SpotLight::setBeamWidth)
        .def_prop_rw("cutOffAngle", &SpotLight::cutOffAngle, &SpotLight::setCutOffAngle)
        .def("setCutOffAngle", &SpotLight::setCutOffAngle)
        .def_prop_rw("cutOffExponent", &SpotLight::cutOffExponent, &SpotLight::setCutOffExponent)
        .def("setCutOffExponent", &SpotLight::setCutOffExponent)
        ;

    python::exportDeviceListNarrowFunction<Device>(m, "DeviceList");
    python::exportDeviceListNarrowFunction<ForceSensor>(m, "ForceSensorList");
}

}
