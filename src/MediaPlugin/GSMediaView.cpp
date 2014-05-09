/**
   @author Shin'ichiro Nakaoka
*/

#include "GSMediaView.h"
#include "MediaItem.h"
#include <cnoid/TimeBar>
#include <cnoid/ConnectionSet>
#include <cnoid/MenuManager>
#include <cnoid/ViewManager>
#include <cnoid/Archive>
#include <cnoid/ItemTreeView>
#include <cnoid/MessageView>
#include <cnoid/LazyCaller>
#include <cnoid/Sleep>
#include <QEvent>
#include <QResizeEvent>
#include <QApplication>
#include <QPainter>
#include <gst/gst.h>
#include <gst/interfaces/xoverlay.h>
#include <boost/bind.hpp>
#include "gettext.h"

#include <iostream>

using namespace std;
using namespace boost;
using namespace cnoid;

//#define CNOID_GST_1_0

namespace {
const bool TRACE_FUNCTIONS = false;
const bool TRACE_FUNCTIONS2 = false;

Action* aspectRatioCheck = 0;
Action* orgSizeCheck = 0;
}

namespace cnoid {
    
class GSMediaViewImpl
{
public:

    GSMediaViewImpl(GSMediaView* self);
    ~GSMediaViewImpl();

    GSMediaView* self;
    MessageView* mv;
    TimeBar* timeBar;
    ConnectionSet timeBarConnections;
    MediaItemPtr currentMediaItem;
    GstElement* playbin;
    GstElement* videoSink;        
    bool prepareXwindowIdProcessed;
    WId windowId;
    gint videoWidth;
    gint videoHeight;
    QRegion bgRegion;
    gint64 duration;
    gulong padProbeId;
    bool isPlaying;
    bool isEOS;
    bool isSeeking;
    bool isWaitingForPositiveSeekPos;
    gint64 currentSeekPos;
    LazyCaller seekLater;

    void updateRenderRectangle();
    void onWindowIdChanged();
    GstBusSyncReply onBusMessageSync(GstMessage* msg);
    void onBusMessageAsync(GstMessage* msg);

#ifdef CNOID_GST_1_0
    GstPadProbeReturn onVideoPadGotBuffer(GstPad* pad, GstPadProbeInfo* info);
#else
    gulong onVideoPadGotBuffer(GstPad* pad, GstMiniObject* mini_obj);
#endif

    void onSeekLater();
    void seek();
    void seek(double time);
            
    void onItemCheckToggled(Item* item, bool isChecked);
    void activateCurrentMediaItem();
    bool onPlaybackInitialized(double time);
    void onPlaybackStarted(double time);
    bool onTimeChanged(double time);
    void onPlaybackStopped(double time);
    void stopPlayback();

    void onZoomPropertyChanged();

    bool storeState(Archive& archive);
    bool restoreState(const Archive& archive);
};
}


namespace {

GstBusSyncReply busSyncHandler(GstBus* bus, GstMessage* message, gpointer instance)
{
    return ((GSMediaViewImpl*)instance)->onBusMessageSync(message);
}

#ifdef CNOID_GST_1_0
GstPadProbeReturn videoPadBufferProbeCallback(GstPad* pad, GstPadProbeInfo* info, gpointer instance)
#else
    gulong videoPadBufferProbeCallback(GstPad* pad, GstMiniObject* mini_obj, gpointer instance)
#endif
{
    return ((GSMediaViewImpl*)instance)->onVideoPadGotBuffer(pad, mini_obj);
}
}
        


bool GSMediaView::initializeClass(ExtensionManager* ext)
{
    static bool initialized = false;

    if(!initialized){

#ifndef GLIB_VERSION_2_32
        g_thread_init(NULL);
#endif

        GError* error = NULL;
        if(!gst_init_check(NULL, NULL, &error)){
            MessageView::mainInstance()->put(_("GStreamer cannot be initialized:"));
            if(error){
                MessageView::mainInstance()->putln(error->message);
                g_error_free(error);
            }
            return false;
        }

        ext->viewManager().registerClass<GSMediaView>(
            "MediaView", N_("Media"), ViewManager::SINGLE_OPTIONAL);
        
        MenuManager& mm = ext->menuManager();
        
        mm.setPath("/Options").setPath(N_("Media View"));
        
        aspectRatioCheck = mm.addCheckItem(_("Keep Aspect Ratio"));
        aspectRatioCheck->setChecked(true);
        
        orgSizeCheck = mm.addCheckItem(_("Keep Original Size"));
        orgSizeCheck->setChecked(true);

        initialized = true;
    }

    return initialized;
}


GSMediaView::GSMediaView()
{
    if(TRACE_FUNCTIONS){
        cout << "GSMediaView::GSMediaView()" << endl;
    }

    setName(N_("Media"));
    setDefaultLayoutArea(View::CENTER);

    setAttribute(Qt::WA_NativeWindow);
    setAttribute(Qt::WA_PaintOnScreen);
    setAttribute(Qt::WA_OpaquePaintEvent);

    //
    //setAutoFillBackground(true);

    impl = new GSMediaViewImpl(this);
}


GSMediaViewImpl::GSMediaViewImpl(GSMediaView* self)
    : self(self),
      mv(MessageView::mainInstance()),
      timeBar(TimeBar::instance()),
      playbin(0),
      videoSink(0),
      windowId(0),
      seekLater(bind(&GSMediaViewImpl::onSeekLater, this))
{
    if(TRACE_FUNCTIONS){
        cout << "GSMediaViewImpl::GSMediaViewImpl()" << endl;
    }

    playbin = gst_element_factory_make("playbin2", NULL);

    if(!playbin){
        mv->putln(_("Initialization of the GSMediaView failed. The playbin2 plugin is missing."));
        return;
    }

    videoSink = gst_element_factory_make("xvimagesink", NULL);
    if(videoSink){
        GstStateChangeReturn sret = gst_element_set_state(GST_ELEMENT(videoSink), GST_STATE_READY);
        if(sret != GST_STATE_CHANGE_SUCCESS){
            gst_element_set_state(GST_ELEMENT(videoSink), GST_STATE_NULL);
            gst_object_unref(videoSink);
            videoSink = 0;
        }
    }
    if(!videoSink){
        videoSink = gst_element_factory_make("ximagesink", NULL);
        if(videoSink){
            GstStateChangeReturn sret = gst_element_set_state(GST_ELEMENT(videoSink), GST_STATE_READY);
            if(sret != GST_STATE_CHANGE_SUCCESS){
                gst_element_set_state(GST_ELEMENT(videoSink), GST_STATE_NULL);
                gst_object_unref(videoSink);
                videoSink = 0;
            }
        }
    }
    if(!videoSink){
        mv->putln(_("Initialization of the GSMediaView failed. The ximagesink could not be created."));
        return;
    }

    g_object_set(G_OBJECT(videoSink), "force-aspect-ratio", (gboolean)TRUE, NULL);
    g_object_set(G_OBJECT(playbin), "video-sink", videoSink, NULL);
    
    GstBus* bus = gst_pipeline_get_bus(GST_PIPELINE(playbin));
    gst_bus_set_sync_handler(GST_BUS(bus), busSyncHandler, (gpointer)this);
    gst_object_unref(GST_OBJECT(bus));

    videoWidth = -1;
    videoHeight = -1;
    
    padProbeId = 0;
    isPlaying = false;
    isSeeking = false;
    currentSeekPos = 0;
    prepareXwindowIdProcessed = false;

    aspectRatioCheck->sigToggled().connect(bind(&GSMediaViewImpl::onZoomPropertyChanged, this));
    orgSizeCheck->sigToggled().connect(bind(&GSMediaViewImpl::onZoomPropertyChanged, this));
    
    ItemTreeView::mainInstance()->sigCheckToggled().connect(
        bind(&GSMediaViewImpl::onItemCheckToggled, this, _1, _2));
}


GSMediaView::~GSMediaView()
{
    if(TRACE_FUNCTIONS){
        cout << "GSMediaView::~GSMediaView()" << endl;
    }

    delete impl;
}


GSMediaViewImpl::~GSMediaViewImpl()
{
    if(TRACE_FUNCTIONS){
        cout << "GSMediaViewImpl::~GSMediaViewImpl()" << endl;
    }
    
    if(playbin){
        stopPlayback();

        // unset busSyncHandler
        GstBus* bus = gst_pipeline_get_bus(GST_PIPELINE(playbin));
        gst_bus_set_sync_handler(GST_BUS(bus), NULL, NULL);
        gst_object_unref(GST_OBJECT(bus));
    }
    timeBarConnections.disconnect();

    if(playbin){
        gst_object_unref(GST_OBJECT(playbin));
    }
}


bool GSMediaView::event(QEvent* event)
{
    if(TRACE_FUNCTIONS){
        cout << "GSMediaView::event()" << endl;
    }
    
    if(event->type() == QEvent::WinIdChange){
        impl->onWindowIdChanged();
    }
    return QWidget::event(event);
}


void GSMediaViewImpl::onWindowIdChanged()
{
    windowId = self->winId();
    QApplication::syncX();

    if(TRACE_FUNCTIONS){
        cout << "GSMediaView::onWindowIdChanged(" << windowId << ")" << endl;
    }
    
    
    if(currentMediaItem){
        if(prepareXwindowIdProcessed){
#ifdef CNOID_GST_1_0
            gst_video_overlay_set_window_handle(GST_VIDEO_OVERLAY(videoSink), windowId);
#else
            //gst_x_overlay_prepare_xwindow_id(GST_X_OVERLAY(videoSink));
            gst_x_overlay_set_xwindow_id(GST_X_OVERLAY(videoSink), windowId);
            gst_x_overlay_expose(GST_X_OVERLAY(videoSink));
#endif
        } else {
            activateCurrentMediaItem();
        }
    }
}


void GSMediaView::resizeEvent(QResizeEvent* event)
{
    if(TRACE_FUNCTIONS){
        cout << "GSMediaView::resizeEvent()" << endl;
    }

    impl->updateRenderRectangle();

    if(!impl->isPlaying && impl->currentMediaItem){
        impl->seekLater();
    }
}


void GSMediaViewImpl::updateRenderRectangle()
{
    int width = self->width();
    int height = self->height();
    
    bgRegion = QRect(0, 0, width, height);

    int x, y;
    
    if(orgSizeCheck->isChecked()){
        if(videoWidth > 0 && videoHeight > 0){
            x = (width - videoWidth) / 2;
            y = (height - videoHeight) / 2;
            width = videoWidth;
            height = videoHeight;
        }
    } else {
        if(width >= 3 && height >= 3){
            x = 1;
            y = 1;
            width -= 2;
            height -= 2;
        } else {
            x = 0;
            y = 0;
        }
    }        
    gst_x_overlay_set_render_rectangle(GST_X_OVERLAY(videoSink), x, y, width, height);
    bgRegion = bgRegion.subtracted(QRegion(x, y, width, height));
}


void GSMediaView::paintEvent(QPaintEvent* event)
{
    if(TRACE_FUNCTIONS){
        cout << "GSMediaView::paintEvent()" << endl;
    }

    QPainter painter(this);
    if(impl->currentMediaItem){
        painter.setClipRegion(impl->bgRegion);
        painter.setClipping(true);
    }
    painter.fillRect(0, 0, width(), height(), Qt::black);
    
    if(!impl->isPlaying && impl->currentMediaItem){
        impl->seekLater();
    }
}    


QPaintEngine* GSMediaView::paintEngine () const
{
    if(TRACE_FUNCTIONS){
        cout << "GSMediaView::paintEngine()" << endl;
    }
    
    //return 0;
    return QWidget::paintEngine();
}


void GSMediaView::onActivated()
{
    if(TRACE_FUNCTIONS){
        cout << "GSMediaView::onActivated()" << endl;
    }
    if(!impl->isPlaying && impl->currentMediaItem){
        impl->seekLater();
    }
}


void GSMediaView::onDeactivated()
{
    if(TRACE_FUNCTIONS){
        cout << "GSMediaView::onDeactivated()" << endl;
    }
    //gst_video_overlay_set_window_handle(GST_VIDEO_OVERLAY(videoSink), 0);
}


GstBusSyncReply GSMediaViewImpl::onBusMessageSync(GstMessage* message)
{
    if(TRACE_FUNCTIONS2){
        cout << "GSMediaViewImpl::onBusMessageSync(): ";
        cout << "message=" << GST_MESSAGE_TYPE_NAME(message) << endl;
    }

    switch(GST_MESSAGE_TYPE(message)){

    case GST_MESSAGE_ASYNC_DONE:
        isSeeking = false;
        break;

    case GST_MESSAGE_ELEMENT:
        //gst_is_video_overlay_prepare_window_handle_message(message))
        if(gst_structure_has_name(gst_message_get_structure(message), "prepare-xwindow-id")){
            if(windowId){
                if(TRACE_FUNCTIONS2){
                    cout << "X-WINDOW ID is set" << endl;
                }
                prepareXwindowIdProcessed = true;

#ifdef CNOID_GST_1_0                
                gst_video_overlay_set_window_handle(GST_VIDEO_OVERLAY(GST_MESSAGE_SRC(message)), windowId);
#else
                gst_x_overlay_set_xwindow_id(GST_X_OVERLAY(videoSink), windowId);
#endif
                gst_message_unref(message);
                return GST_BUS_DROP;
            }
        }
        break;

    default:
        break;
    }

    callLater(bind(&GSMediaViewImpl::onBusMessageAsync, this, gst_message_copy(message)));

    return GST_BUS_PASS;
}


void GSMediaViewImpl::onBusMessageAsync(GstMessage* msg)
{
    if(TRACE_FUNCTIONS2){
        cout << "GSMediaView::onBusMessageAsync()" << endl;
    }
    
    switch(GST_MESSAGE_TYPE(msg)){

    case GST_MESSAGE_EOS:
        if(TRACE_FUNCTIONS2){
            cout << "GST_MESSAGE_EOS" << endl;
        }
        isEOS = true;
        break;

    case GST_MESSAGE_ERROR:
    {
        if(TRACE_FUNCTIONS2){
            cout << "GST_MESSAGE_ERROR" << endl;
        }
        gchar* debug;
        GError* err;
        gst_message_parse_error(msg, &err, &debug);
        g_free (debug);
        cout << "Media playback error: " << err->message << endl;
        g_error_free (err);
        stopPlayback();
        break;
    }

    case GST_MESSAGE_WARNING:
        //gst_message_parse_warning();
        break;

    case GST_MESSAGE_INFO:
        //gst_message_parse_info();
        break;

    default:
        if(TRACE_FUNCTIONS2){
            cout << "debug: on_bus_message: unhandled message=" << GST_MESSAGE_TYPE_NAME(msg) << endl;
        }
        break;
    }

    gst_message_unref(msg);
}


#ifdef CNOID_GST_1_0
GstPadProbeReturn GSMediaViewImpl::onVideoPadGotBuffer(GstPad* pad, GstPadProbeInfo* info)
#else
    gulong GSMediaViewImpl::onVideoPadGotBuffer(GstPad* pad, GstMiniObject* mini_obj)
#endif
{
    if(TRACE_FUNCTIONS2){
        cout << "GSMediaView::onVideoPadGotBuffer()" << endl;
    }

#ifdef CNOID_GST_1_0
    GstBuffer* buffer = GST_PAD_PROBE_INFO_BUFFER(info);
#else
    GstBuffer* buffer = GST_BUFFER_CAST(mini_obj);
#endif
    
    if(buffer){
#ifdef CNOID_GST_1_0
        GstCaps* caps = gst_pad_get_current_caps(pad);
#else
        GstCaps* caps = gst_buffer_get_caps(buffer);
#endif
        const GstStructure* structure = gst_caps_get_structure(caps, 0);

        if(gst_structure_get_int(structure, "width", &videoWidth) &&
           gst_structure_get_int(structure, "height", &videoHeight)){
            updateRenderRectangle();
        }
    }

#ifndef CNOID_GST_1_0
    gst_pad_remove_buffer_probe(pad, padProbeId);
    padProbeId = 0; // Clear probe id to indicate that it has been removed
    return TRUE; // Keep buffer in pipeline (do not throw away)
#else
    return GST_PAD_PROBE_REMOVE;
#endif
}


void GSMediaViewImpl::onSeekLater()
{
    if(TRACE_FUNCTIONS){
        cout << "GSMediaViewImpl::onSeekLater()" << endl;
    }
    if(!isSeeking){
        isSeeking = true;
        gst_element_seek_simple(playbin, GST_FORMAT_TIME,
                                (GstSeekFlags)(GST_SEEK_FLAG_ACCURATE|GST_SEEK_FLAG_FLUSH),
                                std::max((gint64)0, currentSeekPos));
    }
}


void GSMediaViewImpl::seek()
{
    if(TRACE_FUNCTIONS){
        cout << "GSMediaViewImpl::seek()" << endl;
    }
    if(isSeeking){
        seekLater();
    } else {
        isSeeking = true;
        gst_element_seek_simple(playbin, GST_FORMAT_TIME,
                                (GstSeekFlags)(GST_SEEK_FLAG_ACCURATE|GST_SEEK_FLAG_FLUSH),
                                std::max((gint64)0, currentSeekPos));
    }
}


void GSMediaViewImpl::seek(double time)
{
    if(TRACE_FUNCTIONS){
        cout << "GSMediaView::seek(" << time << ")" << endl;
    }

    currentSeekPos = (time + currentMediaItem->offsetTime()) * GST_SECOND;
    seek();
}



void GSMediaViewImpl::onItemCheckToggled(Item* item, bool isChecked)
{
    if(TRACE_FUNCTIONS){
        cout << "GSMediaViewImpl::onItemCheckToggled()" << endl;
    }
    
    MediaItem* mediaItem = dynamic_cast<MediaItem*>(item);
    if(mediaItem){
        ItemList<MediaItem> checkedItems = ItemTreeView::mainInstance()->checkedItems<MediaItem>();
        MediaItemPtr targetItem;
        if(!checkedItems.empty()){
            targetItem = checkedItems.front();
        }
        if(targetItem != currentMediaItem){

            if(currentMediaItem){
                stopPlayback();
            }
            currentMediaItem = targetItem;

            if(!currentMediaItem || currentMediaItem->mediaURI().empty()){
                g_object_set(G_OBJECT(playbin), "uri", "", NULL);
                timeBarConnections.disconnect();

            } else {
                GstPad* pad = gst_element_get_static_pad(GST_ELEMENT(videoSink), "sink");
                //padProbeId = gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_BUFFER,
                //                               (GstPadProbeCallback)videoPadBufferProbeCallback, this, NULL);

                padProbeId = gst_pad_add_buffer_probe(pad, (GCallback)videoPadBufferProbeCallback, this);
                
                gst_object_unref(pad);

                if(self->winId()){
                    activateCurrentMediaItem();
                }
            }
        }
    }
}


void GSMediaViewImpl::activateCurrentMediaItem()
{
    if(TRACE_FUNCTIONS){
        cout << "GSMediaViewImpl::activateCurrentMediaItem()" << endl;
    }
    
    if(currentMediaItem){

        g_object_set(G_OBJECT(videoSink), "show-preroll-frame", (gboolean)FALSE, NULL);
        g_object_set(G_OBJECT(playbin), "uri", currentMediaItem->mediaURI().c_str(), NULL);
        
        gst_element_set_state(GST_ELEMENT(playbin), GST_STATE_PAUSED);
        
        GstState state, pending;
        gst_element_get_state(GST_ELEMENT(playbin), &state, &pending, GST_CLOCK_TIME_NONE); // wait
        //seek(timeBar->time());
        seekLater();

        g_object_set(G_OBJECT(videoSink), "show-preroll-frame", (gboolean)TRUE, NULL);

        if(timeBarConnections.empty()){
            timeBarConnections.add(
                timeBar->sigPlaybackInitialized().connect(
                    bind(&GSMediaViewImpl::onPlaybackInitialized, this, _1)));
            timeBarConnections.add(
                timeBar->sigPlaybackStarted().connect(
                    bind(&GSMediaViewImpl::onPlaybackStarted, this, _1)));
            timeBarConnections.add(
                timeBar->sigPlaybackStopped().connect(
                    bind(&GSMediaViewImpl::onPlaybackStopped, this, _1)));
            timeBarConnections.add(
                timeBar->sigTimeChanged().connect(
                    bind(&GSMediaViewImpl::onTimeChanged, this, _1)));
        }

        self->update();
    }
}


bool GSMediaViewImpl::onPlaybackInitialized(double time)
{
    if(TRACE_FUNCTIONS){
        cout << "GSMediaViewImpl::onPlaybackInitialized(%1):" << endl;
    }

    GstState state, pending;

    //playbin->set_state(Gst::STATE_PAUSED);
    gst_element_set_state(GST_ELEMENT(playbin), GST_STATE_PAUSED);
    
    gst_element_get_state(GST_ELEMENT(playbin), &state, &pending, GST_CLOCK_TIME_NONE); // wait
    
    seek(time);
    gst_element_get_state(GST_ELEMENT(playbin), &state, &pending, GST_CLOCK_TIME_NONE); // wait for seek
    
    return true;
}


void GSMediaViewImpl::onPlaybackStarted(double time)
{
    if(TRACE_FUNCTIONS){
        cout << "GSMediaViewImpl::onPlaybackStarted(%1)" << endl;
    }

    isPlaying = true;
    isEOS = false;

    if(currentSeekPos >= 0){
        isWaitingForPositiveSeekPos = false;
        GstStateChangeReturn ret = gst_element_set_state(GST_ELEMENT(playbin), GST_STATE_PLAYING);
    } else {
        isWaitingForPositiveSeekPos = true;
    }
}


bool GSMediaViewImpl::onTimeChanged(double time)
{
    /*
      if(TRACE_FUNCTIONS){
      cout << "GSMediaViewImpl::onTimeChanged(" << time << ")" << endl;
      }
    */

    if(!self->isActive()){
        if(TRACE_FUNCTIONS){
            cout << "GSMediaViewImpl::onTimeChanged(): view is not active." << endl;
        }
        return false;
    }
    
    bool doContinue = false;

    if(isPlaying){
        currentSeekPos = (time + currentMediaItem->offsetTime()) * GST_SECOND;
        if(isWaitingForPositiveSeekPos){
            if(currentSeekPos >= 0){
                gst_element_seek_simple(playbin, GST_FORMAT_TIME,
                                        (GstSeekFlags)(GST_SEEK_FLAG_ACCURATE|GST_SEEK_FLAG_FLUSH),
                                        currentSeekPos);
                gst_element_set_state(GST_ELEMENT(playbin), GST_STATE_PLAYING);
                isWaitingForPositiveSeekPos = false;
            }
        } 
        doContinue = !isEOS;
    } else {
        seek(time);
    }

    return doContinue;
}


void GSMediaViewImpl::onPlaybackStopped(double time)
{
    if(TRACE_FUNCTIONS){
        cout << "GSMediaViewImpl::onPlaybackStopped()" << endl;
    }
    
    if(isPlaying){
        GstStateChangeReturn ret = gst_element_set_state(GST_ELEMENT(playbin), GST_STATE_PAUSED);
        if(TRACE_FUNCTIONS){
            cout << "ret = " << ret << endl;
        }
        isPlaying = false;
    }
}


void GSMediaViewImpl::stopPlayback()
{
    if(TRACE_FUNCTIONS){
        cout << "GSMediaViewImpl::stopPlayback()" << endl;
    }
    
    GstStateChangeReturn ret = gst_element_set_state(GST_ELEMENT(playbin), GST_STATE_NULL);

    if(TRACE_FUNCTIONS){
        cout << "ret = " << ret << endl;
    }
    
    if(padProbeId != 0){
        if(TRACE_FUNCTIONS){
            cout << "padProbeId = " << padProbeId << endl;
        }
        
        //GstPad* pad = gst_element_get_static_pad(GST_ELEMENT(videoSink), "sink");
        //gst_pad_remove_probe(pad, padProbeId);
        //padProbeId  = 0;
    }

    isPlaying = false;
}


void GSMediaViewImpl::onZoomPropertyChanged()
{
    if(orgSizeCheck->isChecked()){
        g_object_set(G_OBJECT(videoSink), "force-aspect-ratio", (gboolean)FALSE, NULL);
        updateRenderRectangle();
        //gst_x_overlay_expose(GST_X_OVERLAY(videoSink));
        self->update();
    } else {
        g_object_set(G_OBJECT(videoSink), "force-aspect-ratio", (gboolean)aspectRatioCheck->isChecked(), NULL);
        updateRenderRectangle();
        gst_x_overlay_expose(GST_X_OVERLAY(videoSink));
    }
}


bool GSMediaView::storeState(Archive& archive)
{
    return impl->storeState(archive);
}


bool GSMediaViewImpl::storeState(Archive& archive)
{
    archive.write("keepAspectRatio", aspectRatioCheck->isChecked());
    archive.write("keepOriginalSize", orgSizeCheck->isChecked());
    return true;
}


bool GSMediaView::restoreState(const Archive& archive)
{
    return impl->restoreState(archive);
}


bool GSMediaViewImpl::restoreState(const Archive& archive)
{
    aspectRatioCheck->setChecked(archive.get("keepAspectRatio", aspectRatioCheck->isChecked()));
    orgSizeCheck->setChecked(archive.get("keepOriginalSize", orgSizeCheck->isChecked()));
    return true;
}
