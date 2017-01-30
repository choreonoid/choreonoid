/*!
   @file
   @author Hiroyuki Matsushita
*/

#include "SDFSensorConverter.h"

using namespace cnoid;

SDFSensorInfo::SDFSensorInfo()
{
    devices_.clear();
}

SDFSensorInfo::~SDFSensorInfo()
{
}

void SDFSensorInfo::addDevice(DevicePtr device)
{
    if (device) {
        device->setIndex(devices_.size());
        devices_.push_back(device);
    }

    return;
}

SDFSensorConverter::SDFSensorConverter()
{
    deviceMap_.clear();

    // TODO: in the case of libsdformat3: sdformatVersion_ = sdf::SDF::Version();
    sdformatVersion_ = sdf::SDF::version;
}

SDFSensorConverter::~SDFSensorConverter()
{
    clearAllDevices();
}

void SDFSensorConverter::convert(const std::string name, sdf::ElementPtr ep)
{
    sdf::ElementPtr p;

    if (name.empty() == true || (! ep) || (ep->GetName() != "link" && ep->GetName() != "joint") || 
        ep->HasElement("sensor") == false) {
        return;
    }

    for (p = ep->GetElement("sensor"); p; p = p->GetNextElement("sensor")) {
        createDevice(name, p, ep);
    }

    return;
}

void SDFSensorConverter::attachDevices(Body* body)
{
    Link*                           link;
    Affine3                         T(Affine3::Identity());
    std::map<std::string, uint32_t> ids;
    SDFSensorInfo*                  si;
    DevicePtr                       p;

    if ((! body) || deviceMap_.empty() == true) {
        return;
    } else if (body->numLinks() < 1) {
        return;
    }

    ids.clear();

    for (size_t i = 0; i < body->numLinks(); i++) {
        link = body->link(i);

        if (deviceMap_.find(link->name()) == deviceMap_.end()) {
            continue;
        }

        si = deviceMap_[ link->name() ];
        const DeviceList<>& dl = si->getDevices();

        for (size_t j = 0; j < dl.size(); j++) {
            p = dl[ j ];

            if (ids.find(p->typeName()) != ids.end()) {
                p->setId(ids[ p->typeName() ] + 1);
                ids[ p->typeName() ] = p->id();
            } else {
                ids[ p->typeName() ] = 0;
                p->setId(0);
            }

            p->setIndex(p->id());
            p->setLink(link);
            const Matrix3 RsT = link->Rs();
            p->setLocalTranslation(RsT * (T * p->localTranslation()));
            p->setLocalRotation(RsT * (T.linear() * p->localRotation()));
            body->addDevice(p);

#if (DEBUG_SDF_SENSOR_CONVERTER > 0) /* (DEBUG_SDF_SENSOR_CONVERTER > 0) */
            std::cout << __FUNCTION__ << ": attached (id=" <<  p->id() << " type=" << p->typeName()
                      << " name=" << p->name() << ")" << std::endl
                      << "-- translation --" << std::endl << p->localTranslation() << std::endl
                      << "-- rotation    --" << std::endl << p->localRotation() << std::endl << std::endl; 
#endif                               /* (DEBUG_SDF_SENSOR_CONVERTER > 0) */
        }
    }

    clearAllDevices();

    return;
}

void SDFSensorConverter::mergeInfo(const std::string src, const std::string dst)
{
    /*
      @if jp
      ルートリンクについては、SDFBodyLoaderImpl::load メソッドにおいて、SDF のリンク名を指定した
      cnoid::Link オブジェクトを生成する為、以下の回避条件を設けている。
      @endif
     */
    if (rootLinkName_.empty() == false && rootLinkName_ == src) {
        return;
    }

    if (deviceMap_.find(src) != deviceMap_.end()) {
        if (deviceMap_.find(dst) != deviceMap_.end()) {
            SDFSensorInfo*      sp = deviceMap_[ src ];
            SDFSensorInfo*      dp = deviceMap_[ dst ];
            const DeviceList<>& dl = sp->getDevices();

            for (size_t i = 0; i < dl.size(); i++) {
                dp->addDevice(dl[ i ]);
            }
        } else {
            deviceMap_[ dst ] = deviceMap_[ src ];
        }

        deviceMap_.erase(src);
    }

    return;
}

void SDFSensorConverter::clearAllDevices()
{
    std::map<std::string, SDFSensorInfo*>::iterator it;

    if (deviceMap_.empty() == false) {
        for (it = deviceMap_.begin(); it != deviceMap_.end(); it++) {
            it->second->clearDevices();
        }
    }

    deviceMap_.clear();

    return;
}

#if (DEBUG_SDF_SENSOR_CONVERTER > 0) /* (DEBUG_SDF_SENSOR_CONVERTER > 0) */
void SDFSensorConverter::dumpDeviceMap()
{
    std::map<std::string, SDFSensorInfo*>::iterator it;

    std::cout << std::endl << "dumping of sensor devices:" << std::endl;

    if (deviceMap_.empty() == true) {
        std::cout << "  <empty>" << std::endl << std::endl;
        return;
    }

    for (it = deviceMap_.begin(); it != deviceMap_.end(); it++) {
        std::cout << "  " << it->first << std::endl;

        const DeviceList<>& dl = it->second->getDevices();
        DevicePtr p;

        for (size_t i = 0; i < dl.size(); i++) {
            p = dl[ i ];

            std::cout << "    index=" << p->index() << " name=" << p->name() << " type=" << p->typeName()
                      << std::endl;
        }

        std::cout << std::endl;
    }

    std::cout << std::endl;

    return;
}
#endif                               /* (DEBUG_SDF_SENSOR_CONVERTER > 0) */

/*
  Private methods.
 */

SDFSensorInfo* SDFSensorConverter::getSDFSensorInfo(const std::string key)
{
    SDFSensorInfo* ret;

    if (deviceMap_.find(key) == deviceMap_.end()) {
        ret = new SDFSensorInfo();
        deviceMap_[ key ] = ret;
    } else {
        ret = deviceMap_[ key ];
    }

    return ret;
}

void SDFSensorConverter::removeSDFSensorInfo(const std::string key)
{
    if (deviceMap_.find(key) != deviceMap_.end()) {
        deviceMap_[ key ]->clearDevices();
        deviceMap_.erase(key);
    }

    return;
}

void SDFSensorConverter::setDevicePose(DevicePtr device, const sdf::Pose& pose)
{
    Vector3    trans;
    Quaternion R;

    if (device) {
        trans << pose.pos.x, pose.pos.y, pose.pos.z; 
        R.x() = pose.rot.x;
        R.y() = pose.rot.y;
        R.z() = pose.rot.z;
        R.w() = pose.rot.w;

        device->setLocalTranslation(trans);
        device->setLocalRotation(R.matrix());
    }

    return;
}

void SDFSensorConverter::createDevice(const std::string key, sdf::ElementPtr ep, sdf::ElementPtr parent)
{
    std::string name;
    std::string type;

    if (key.empty() == true || (! ep) || (! parent)) {
        return;
    } else if (ep->HasAttribute("name") == false || ep->HasAttribute("type") == false) {
        return;
    }

    name = ep->Get<std::string>("name");
    type = ep->Get<std::string>("type");

    if (type == "camera") {
        createOrdinaryCameraDevice(key, name, ep);
    } else if (type == "depth") {
        createDepthCameraDevice(key, name, ep);
    } else if (type == "force_torque") {
        if (parent->GetName() == "link") {
            MessageView::instance()->putln(
                MessageView::ERROR,
                boost::format("%1% cannot set a [ %2% ] sensor (%3% in %4%)")
                    % parent->GetName() % type % name % key
                );
            return;
        }

        createForceTorqueDevice(key, name, ep);
    } else if (type == "gpu_ray") {
        createGPURayDevice(key, name, ep);
    } else if (type == "imu") {
        createIMUDevice(key, name, ep);
    } else if (type == "multicamera") {
        createMultiCameraDevice(key, name, ep);
    } else if (type == "ray") {
        createRayDevice(key, name, ep);
#if 0    /* These SDF default support types, but not yet supported */
    } else if (type == "altimeter") {
    } else if (type == "contact") {
        if (parent->GetName() == "joint") {
            MessageView::instance()->putln(
                MessageView::ERROR,
                boost::format("%1% cannot set a [ %2% ] sensor (%3% in %4%)")
                    % parent->GetName() % type % name % key
                );
            return;
        }
    } else if (type == "gps") {
    } else if (type == "logical_camera") {
    } else if (type == "magnetometer") {
    } else if (type == "rfid") {
    } else if (type == "rfidtag") {
    } else if (type == "sonar") {
    } else if (type == "wireless_receiver") {
    } else if (type == "wireless_transmitter") {
#endif   /* These SDF default support types, but not yet supported */
    } else {
        MessageView::instance()->putln(
            MessageView::WARNING,
            boost::format("unable to create SDF sensor of type [ %1% ] (%2% in %3%)") % type % name % key
            );
    }

    return;
}

bool SDFSensorConverter::validateCameraElement(sdf::ElementPtr cp, const std::string key, const std::string name)
{
    sdf::ElementPtr image;
    sdf::ElementPtr clip;

    if (! cp) {
        return false;
    }

    if (cp->HasElement("horizontal_fov") == false || cp->HasElement("image") == false ||
        cp->HasElement("clip") == false) {
        MessageView::instance()->putln(
            MessageView::ERROR,
            boost::format("must be set horizontal_fov, image, clip (%1% in %2%)") % name % key
            );
        return false;
    }

    image = cp->GetElement("image");
    clip = cp->GetElement("clip");
                
    if (image->HasElement("width") == false || image->HasElement("height") == false ||
        clip->HasElement("near") == false || clip->HasElement("far") == false) {
        MessageView::instance()->putln(
            MessageView::ERROR,
            boost::format("must be set width, height, near, far (%1% in %2%)") % name % key
            );
        return false;
    }

    if (image->HasElement("format") == true && image->Get<std::string>("format") != "R8G8B8" &&
        image->Get<std::string>("format") != "L8") {
        MessageView::instance()->putln(
            MessageView::ERROR,
            boost::format("unsupported image format [ format=%1% ] (%2% in %3%)")
                % image->Get<std::string>("format") % name % key
            );
        return false;
    }

    return true;
}

void SDFSensorConverter::createCameraDevice(const std::string key, const std::string name, sdf::ElementPtr ep,
                                            const SDFSensorConverter::CameraType type)
{
    CameraPtr       camera;
    RangeCamera*    rcamera;
    sdf::ElementPtr cp;     // element <camera>
    SDFSensorInfo*  p;

    if (type == SDFSensorConverter::ORDINARY) {
        camera = new Camera();
        camera->setImageType(Camera::COLOR_IMAGE);
        rcamera = 0;
    } else if (type == SDFSensorConverter::RANGE) {
        rcamera = new RangeCamera();
        rcamera->setOrganized(true);
        rcamera->setImageType(Camera::COLOR_IMAGE);
    } else if (type == SDFSensorConverter::POINT_CLOUD) {
        rcamera = new RangeCamera();
        rcamera->setOrganized(false);
        rcamera->setImageType(Camera::COLOR_IMAGE);
    } else {
        // programer's error ?
#if (DEBUG_SDF_SENSOR_CONVERTER > 0) /* (DEBUG_SDF_SENSOR_CONVERTER > 0) */
        std::cout << __FUNCTION__ << ": occured error (unknown type [ " << type << "])" << std::endl; 
#endif                               /* (DEBUG_SDF_SENSOR_CONVERTER > 0) */
        return;
    }

    if (rcamera) {
        camera = rcamera;
    }

    if (ep->HasElement("camera") == true) {
        cp = ep->GetElement("camera");

        if (validateCameraElement(cp, key, name) == false) {
            return;
        }

        camera->setFieldOfView(cp->Get<double>("horizontal_fov"));
        camera->setNearClipDistance(cp->GetElement("clip")->Get<double>("near"));
        camera->setFarClipDistance(cp->GetElement("clip")->Get<double>("far"));
        camera->setResolution(cp->GetElement("image")->Get<int>("width"),
                              cp->GetElement("image")->Get<int>("height"));
    }

    // setting frame rate
    if (ep->HasElement("update_rate")) {
        camera->setFrameRate(ep->Get<double>("update_rate"));
    }

    // setting pose
    if (ep->HasElement("pose")) {
        setDevicePose(camera, ep->Get<sdf::Pose>("pose"));
    }

    camera->setName(name);

    p = getSDFSensorInfo(key);
    p->addDevice(camera);

    return;
}

void SDFSensorConverter::createOrdinaryCameraDevice(const std::string key, const std::string name, sdf::ElementPtr ep)
{
    // XXX: these condition, temporary implementation
    if (ep->HasElement("plugin") == true && ep->GetElement("plugin")->HasElement("pointCloudTopicName") == true) {
        createCameraDevice(key, name, ep, SDFSensorConverter::POINT_CLOUD);
    } else {
        createCameraDevice(key, name, ep, SDFSensorConverter::ORDINARY);
    }

    return;
}

void SDFSensorConverter::createDepthCameraDevice(const std::string key, const std::string name, sdf::ElementPtr ep)
{
    // XXX: these condition, temporary implementation
    if (ep->HasElement("plugin") == true && ep->GetElement("plugin")->HasElement("pointCloudTopicName") == true) {
        createCameraDevice(key, name, ep, SDFSensorConverter::POINT_CLOUD);
    } else {
        createCameraDevice(key, name, ep, SDFSensorConverter::RANGE);
    }

    return;
}

void SDFSensorConverter::createForceTorqueDevice(const std::string key, const std::string name, sdf::ElementPtr ep)
{
    ForceSensorPtr fsensor;
    SDFSensorInfo* p;

    fsensor = new ForceSensor();

    // setting pose
    if (ep->HasElement("pose")) {
        setDevicePose(fsensor, ep->Get<sdf::Pose>("pose"));
    }

    fsensor->setName(name);

    p = getSDFSensorInfo(key);
    p->addDevice(fsensor);

    return;
}

void SDFSensorConverter::createGPURayDevice(const std::string key, const std::string name, sdf::ElementPtr ep)
{
    createRayDevice(key, name, ep);

    return;
}

bool SDFSensorConverter::setIMUParameter14(AccelerationSensorPtr asensor, RateGyroSensorPtr gsensor,
                                           sdf::ElementPtr ep)
{
    sdf::ElementPtr p;

    if (ep->HasElement("noise")) {
        p = ep->GetElement("noise");

        if (p->HasElement("type") == false || p->HasElement("rate") == false || p->HasElement("accel") == false) {
            return false;
        }
    }

    return true;
}

bool SDFSensorConverter::validateIMUNoiseElement(sdf::ElementPtr ep)
{
    return (ep->HasElement("noise") == true &&
            ep->GetElement("noise")->HasAttribute("type") == true) ? true : false;
}

bool SDFSensorConverter::setIMUParameter(AccelerationSensorPtr asensor, RateGyroSensorPtr gsensor, sdf::ElementPtr ep)
{
    sdf::ElementPtr p;

    if (ep->HasElement("angular_velocity")) {
        p = ep->GetElement("angular_velocity");

        if (p->HasElement("x") && validateIMUNoiseElement(p->GetElement("x")) == false) {
            return false;
        }
        if (p->HasElement("y") && validateIMUNoiseElement(p->GetElement("y")) == false) {
            return false;
        }
        if (p->HasElement("z") && validateIMUNoiseElement(p->GetElement("z")) == false) {
            return false;
        }
    }

    if (ep->HasElement("linear_acceleration")) {
        p = ep->GetElement("linear_acceleration");

        if (p->HasElement("x") && validateIMUNoiseElement(p->GetElement("x")) == false) {
            return false;
        }
        if (p->HasElement("y") && validateIMUNoiseElement(p->GetElement("y")) == false) {
            return false;
        }
        if (p->HasElement("z") && validateIMUNoiseElement(p->GetElement("z")) == false) {
            return false;
        }
    }

    return true;
}

void SDFSensorConverter::createIMUDevice(const std::string key, const std::string name, sdf::ElementPtr ep)
{
    AccelerationSensorPtr asensor;
    RateGyroSensorPtr     gsensor;
    bool                  chk;
    SDFSensorInfo*        p;

    asensor = new AccelerationSensor();
    gsensor = new RateGyroSensor();

    if (sdformatVersion() <= "1.4") {
        chk = setIMUParameter14(asensor, gsensor, ep);
    } else {
        chk = setIMUParameter(asensor, gsensor, ep);
    }

    if (chk == false) {
        MessageView::instance()->putln(
            MessageView::ERROR,
            boost::format("found invalid imu sensor element (%1% in %2%)") % name % key
            );
        return;
    }

    // setting pose
    if (ep->HasElement("pose")) {
        setDevicePose(asensor, ep->Get<sdf::Pose>("pose"));
        setDevicePose(gsensor, ep->Get<sdf::Pose>("pose"));
    }

    asensor->setName(name + "_accel");
    gsensor->setName(name + "_rate");

    p = getSDFSensorInfo(key);
    p->addDevice(asensor);
    p->addDevice(gsensor);

    return;
}

/*
  @if jp
  ステレオカメラデバイス。
  http://gazebosim.org/tutorials?tut=ros_gzplugins より抜粋。(2016/02/08 時点)
  -----
  Multicamera

  Description: synchronizes multiple camera's shutters such that they publish their images together.
  Typically used for stereo cameras, uses a very similar interface as the plain Camera plugin

  Note: currently only supports stereo cameras. See Github issue.
  -----
  Github issue は closed となっており、コメントは「Closing because of lack of interest.」との事。
  @endif
 */
void SDFSensorConverter::createMultiCameraDevice(const std::string key, const std::string name, sdf::ElementPtr ep)
{
    CameraPtr       camera;
    sdf::ElementPtr cp;     // element <camera>
    std::string     cn;     // camera name
    double          fr;     // camera frame rate
    SDFSensorInfo*  p;
    uint32_t        i;

    if (ep->HasElement("camera") == true) {
        p = getSDFSensorInfo(key);
        fr = (ep->HasElement("update_rate")) ? ep->Get<double>("update_rate") : 30.0;
        i = 0;

        for (cp = ep->GetElement("camera"); cp; cp = cp->GetNextElement("camera")) {
            camera = new Camera();
            camera->setImageType(Camera::COLOR_IMAGE);

            if (validateCameraElement(cp, key, name) == false) {
                continue;
            }

            camera->setFieldOfView(cp->Get<double>("horizontal_fov"));
            camera->setNearClipDistance(cp->GetElement("clip")->Get<double>("near"));
            camera->setFarClipDistance(cp->GetElement("clip")->Get<double>("far"));
            camera->setResolution(cp->GetElement("image")->Get<int>("width"),
                                  cp->GetElement("image")->Get<int>("height"));

            cn = name + "_";

            if (cp->HasAttribute("name") == true && cp->Get<std::string>("name") != "__default__") {
                cn += cp->Get<std::string>("name");
            } else {
                cn += boost::lexical_cast<std::string>(i++);
            }

            // setting frame rate
            camera->setFrameRate(fr);

            // setting pose
            if (ep->HasElement("pose")) {
                setDevicePose(camera, ep->Get<sdf::Pose>("pose"));
            }

            camera->setName(cn);
            p->addDevice(camera);
        }
    } else {
        createOrdinaryCameraDevice(key, name, ep);
    }

    return;
}

bool SDFSensorConverter::convertAngle(double* angle, double min, double max, const double defaultValue,
                                      const std::string key, const std::string name)
{
    double result;
    double dmin;
    double dmax;

    result = defaultValue;

    if (! angle) {
        return false;
    } else if (min > max) {
        MessageView::instance()->putln(
            MessageView::ERROR,
            boost::format("max_angle must be greater of equal to min_angle [ min=%1% max=%2% ] (%3% in %4%)")
                % min % max % name % key
            );
        return false;
    } else if (min != max) {
        dmin = (min < 0.0) ? (min * -1.0d) : min;
        dmax = (max < 0.0) ? (max * -1.0d) : max;

        if (dmin == dmax) {
            /*
              range sensor possible range threshold. 
              see VisionRenderer::initializeCamera(). (in src/BodyPlugin/GLVisionSimulatorItem.cpp)
             */
            const double thresh = (170.0 * M_PI / 180.0); // 170[deg]

            result = dmin + dmax;

            if (result >= thresh) {
                MessageView::instance()->putln(
                    MessageView::ERROR,
                    boost::format("sensor range too big [ setting=%1% threshold=%2% ] (%3% in %4%)")
                        % result % thresh % name % key
                    );
                return false;
            }
        } else {
           MessageView::instance()->putln(
               MessageView::WARNING,
               boost::format(
                   "unable to convert angle, use default value [ min=%1% max=%2% default=%3% ] (%4% in %5%)")
                   % dmin % dmax % defaultValue % name % key
               );
        }
    } else {
        result = 0.017;

        if (min != 0.0) {
           MessageView::instance()->putln(
               MessageView::WARNING,
               boost::format(
                   "unable to convert angle, use alternate value [ min=%1% max=%2% alternate=%3% ] (%4% in %5%)")
                   % min % max % result % name % key
               );
        }
    }

    *angle = result;

    return true;
}

void SDFSensorConverter::createRayDevice(const std::string key, const std::string name, sdf::ElementPtr ep)
{
    RangeSensorPtr  rsensor;
    sdf::ElementPtr scan;
    sdf::ElementPtr range;
    sdf::ElementPtr ray;     // element <ray>
    sdf::ElementPtr hrz;     // element <horizontal>
    sdf::ElementPtr vrt;     // element <vertical>
    double          angle;
    SDFSensorInfo*  p;

    rsensor = new RangeSensor();

    if (ep->HasElement("ray") == true) {
        ray = ep->GetElement("ray");

        if (ray->HasElement("scan") == false || ray->HasElement("range") == false) {
            return;
        }

        scan = ray->GetElement("scan");
        range = ray->GetElement("range");

        if (scan->HasElement("horizontal") == false || range->HasElement("min") == false ||
            range->HasElement("max") == false) {
            return;
        }

        hrz = scan->GetElement("horizontal");

        if (hrz->HasElement("resolution") == false || hrz->HasElement("min_angle") == false ||
            hrz->HasElement("max_angle") == false) {
            return;
        }

        if (ray->HasElement("vertical") == true) {
            vrt = ray->GetElement("vertical");

            if (vrt->HasElement("min_angle") == false || vrt->HasElement("max_angle") == false) {
                return;
            }
        } else {
            vrt = sdf::ElementPtr();
        }

        // setting yaw
        if (convertAngle(&angle, hrz->Get<double>("min_angle"), hrz->Get<double>("max_angle"),
                         rsensor->yawRange(), key, name) == true) {
            rsensor->setYawRange(angle);
        }

        rsensor->setYawResolution(hrz->Get<int>("resolution"));

        // setting pitch
        if (vrt) {
            if (convertAngle(&angle, vrt->Get<double>("min_angle"), vrt->Get<double>("max_angle"),
                             rsensor->pitchRange(), key, name) == true) {
                rsensor->setPitchRange(angle);
            }

            if (vrt->HasElement("resolution") == true) {
                rsensor->setPitchResolution(vrt->Get<int>("resolution"));
            }
        }

        // setting distance
        rsensor->setMaxDistance(range->Get<double>("max"));
        rsensor->setMinDistance(range->Get<double>("min"));
    }

    // setting pose
    if (ep->HasElement("pose")) {
        setDevicePose(rsensor, ep->Get<sdf::Pose>("pose"));
    }

    rsensor->setName(name);

    p = getSDFSensorInfo(key);
    p->addDevice(rsensor);

    return;
}

