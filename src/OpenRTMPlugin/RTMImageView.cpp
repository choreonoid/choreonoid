/**
   @author Shin'ichiro Nakaoka
*/

#include "RTMImageView.h"
#include "OpenRTMUtil.h"
#include <cnoid/ImageWidget>
#include <cnoid/ViewManager>
#include <cnoid/LazyCaller>
#include <cnoid/CorbaUtil>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataInPort.h>

#ifdef USE_BUILTIN_CAMERA_IMAGE_IDL
# include "deprecated/corba/CameraImage.hh"
#else
# ifdef _WIN32
#  include <rtm/idl/CameraCommonInterface.hh>
# else
#  include <rtm/ext/CameraCommonInterface.hh>
# endif
#endif

#include <QBoxLayout>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using namespace RTC;
using fmt::format;

namespace {

class ImageViewRTC : public RTC::DataFlowComponentBase
{
public:
    Img::TimedCameraImage timedCameraImage;
    InPort<Img::TimedCameraImage> imageInPort;
    ImageWidget* imageWidget;

    ImageViewRTC(Manager* manager);
    void setImageWidget(ImageWidget* widget);
    ReturnCode_t onInitialize() override;
    ReturnCode_t onExecute(UniqueId ec_id);
};

}

namespace cnoid {

class RTMImageViewImpl
{
public:
    RTMImageView* self;
    ImageWidget* imageWidget;
    ScopedConnection connection;
    ImageViewRTC* rtc;
    RTC::ExecutionContext_var execContext;

    RTMImageViewImpl(RTMImageView* self);
    ~RTMImageViewImpl();
    void createRTC(const std::string& name);
    void deleteRTC();
};

}


void RTMImageView::initializeClass(ExtensionManager* ext)
{
    static const char* spec[] = {
        "implementation_id", "ImageView",
        "type_name",         "ImageView",
        "description",       "Choreonoid View for Showing an Image.",
        "version",           "1.0.0",
        "vendor",            "Choreonoid",
        "category",          "Choreonoid",
        "activity_type",     "DataFlowComponent",
        "max_instance",      "100",
        "language",          "C++",
        "lang_type",         "compile",
        ""
    };

    Properties profile(spec);
    RTC::Manager::instance().registerFactory(
        profile, Create<ImageViewRTC>, Delete<ImageViewRTC>);

    ext->viewManager().registerClass<RTMImageView>(
        "RTMImageView", N_("RTM-Image"), ViewManager::MULTI_OPTIONAL);
}


RTMImageView::RTMImageView()
{
    impl = new RTMImageViewImpl(this);
}


RTMImageViewImpl::RTMImageViewImpl(RTMImageView* self)
    : self(self)
{
    self->setDefaultLayoutArea(View::CENTER);
    QVBoxLayout* vbox = new QVBoxLayout;
    vbox->setSpacing(0);
    imageWidget = new ImageWidget;
    imageWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    imageWidget->setScalingEnabled(true);
    vbox->addWidget(imageWidget, 1);
    self->setLayout(vbox);
    self->setFocusPolicy(Qt::StrongFocus);

    rtc = 0;
    execContext = RTC::ExecutionContext::_nil();
    connection.reset(sigAboutToFinalizeRTM().connect([&]() { deleteRTC(); }));
}


RTMImageView::~RTMImageView()
{
    delete impl;
}


RTMImageViewImpl::~RTMImageViewImpl()
{
    deleteRTC();
}


void RTMImageView::setName(const std::string& name_)
{
    if (name_ != name()) {
        View::setName(name_);
        impl->createRTC(name_);
    }
}


void RTMImageViewImpl::createRTC(const std::string& name)
{
    deleteRTC();

    auto args =
        format("ImageView?instance_name={}&"
               "exec_cxt.periodic.type=PeriodicExecutionContext&exec_cxt.periodic.rate=30",
               self->name());

    rtc = dynamic_cast<ImageViewRTC*>(cnoid::createManagedRTC(args));

    if (rtc) {
        rtc->setImageWidget(imageWidget);
        RTC::ExecutionContextList_var eclist = rtc->get_owned_contexts();
        for (CORBA::ULong i = 0; i < eclist->length(); ++i) {
            if (!CORBA::is_nil(eclist[i])) {
                execContext = RTC::ExecutionContext::_narrow(eclist[i]);
                break;
            }
        }
    }
}


void RTMImageViewImpl::deleteRTC()
{
    if (rtc) {
        cnoid::deleteRTC(rtc);
        rtc = 0;
    }
    execContext = RTC::ExecutionContext::_nil();
}


void RTMImageView::onActivated()
{
    if (!CORBA::is_nil(impl->execContext)) {
        impl->execContext->activate_component(impl->rtc->getObjRef());
    }
}


void RTMImageView::onDeactivated()
{
    if (!CORBA::is_nil(impl->execContext)) {
        impl->execContext->deactivate_component(impl->rtc->getObjRef());
    }
}


ImageViewRTC::ImageViewRTC(Manager* manager)
    : DataFlowComponentBase(manager),
    imageInPort("cameraImage", timedCameraImage)
{
    imageWidget = 0;
}


void ImageViewRTC::setImageWidget(ImageWidget* widget)
{
    imageWidget = widget;
}


ReturnCode_t ImageViewRTC::onInitialize()
{
    addInPort("cameraImage", imageInPort);
    return RTC_OK;
}


ReturnCode_t ImageViewRTC::onExecute(UniqueId ec_id)
{
    if (imageInPort.isNew()) {
        do {
            imageInPort.read();
        } while (imageInPort.isNew());

        int numComponents;
        bool jpegCompression = false;

        switch (timedCameraImage.data.image.format) {
            case Img::CF_GRAY:
                numComponents = 1;
                break;
            case Img::CF_RGB:
                numComponents = 3;
                break;
#ifdef USE_BUILTIN_CAMERA_IMAGE_IDL
            case Img::CF_RGB_JPEG:
#else
            case Img::CF_JPEG:
#endif
                numComponents = 3;
                jpegCompression = true;
                break;
            case Img::CF_UNKNOWN:
            default:
                numComponents = 0;
                break;
        }

        if(!timedCameraImage.data.image.raw_data.length()){
            callLater([&]() { imageWidget->clear(); });
        }

        if (jpegCompression) {
            int size = timedCameraImage.data.image.raw_data.length();
            const unsigned char* src = timedCameraImage.data.image.raw_data.get_buffer();
            const char *format = "JPG";
            QImage qImage = QImage::fromData(src, size, format);

            callLater([&, qImage]() { imageWidget->setImage(qImage); });
        } else {
            Image image;
            image.setSize(timedCameraImage.data.image.width, timedCameraImage.data.image.height, numComponents);
            size_t length = timedCameraImage.data.image.raw_data.length();
            memcpy(image.pixels(), timedCameraImage.data.image.raw_data.get_buffer(), length);

            callLater([&, image]() { imageWidget->setImage(image); });
        }

    }

    return RTC::RTC_OK;
}
