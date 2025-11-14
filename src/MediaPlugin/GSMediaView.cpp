#include "GSMediaView.h"
#include "MediaItem.h"
#include <cnoid/TimeBar>
#include <cnoid/ConnectionSet>
#include <cnoid/MenuManager>
#include <cnoid/MainMenu>
#include <cnoid/ViewManager>
#include <cnoid/Archive>
#include <cnoid/RootItem>
#include <cnoid/ItemList>
#include <cnoid/MessageView>
#include <cnoid/Timer>
#include <cnoid/LazyCaller>
#include <cnoid/Format>
#include <QEvent>
#include <QResizeEvent>
#include <QPainter>
#include <QWindow>

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
#include <QGuiApplication>
#else
#include <QX11Info>
#endif

#include <X11/Xlib.h>
#include <gst/gst.h>
#include <gst/video/videooverlay.h>
#include <iostream>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

const bool TRACE_FUNCTIONS = false;
const bool TRACE_FUNCTIONS2 = false;

Action* aspectRatioCheck = nullptr;
Action* orgSizeCheck = nullptr;

}

namespace cnoid {
    
class GSMediaViewImpl
{
public:
    GSMediaView* self;
    MessageView* mv;
    ScopedConnectionSet connections;
    TimeBar* timeBar;
    ConnectionSet timeBarConnections;
    MediaItemPtr currentMediaItem;
    GstElement* playbin;
    GstElement* videoSink;        
    WId windowId;
    Display* display;
    GC gc;
    gint videoWidth;
    gint videoHeight;
    vector<XRectangle> rects;
    gint64 duration;
    bool isPlaying;
    bool isEOS;
    bool isSeeking;
    bool hasPendingSeek;
    bool isWaitingForPositiveSeekPos;
    gint64 currentSeekPos;
    Timer pendingSeekTimer;

    GSMediaViewImpl(GSMediaView* self);
    ~GSMediaViewImpl();
    void updateRenderRectangle();
    void onWindowIdChanged();
    GstBusSyncReply onBusMessageSync(GstMessage* msg);
    void onBusMessageAsync(GstMessage* msg);
    void putGstMessage(
        GstMessage* message,
        std::function<void(GstMessage* message, GError** gerror, gchar** debug)> parse,
        const char* prefix);
    GstPadProbeReturn onVideoPadGotBuffer(GstPad* pad, GstPadProbeInfo* info);
    void seek(double time);
    void seek();
    void checkPendingSeek();
    void onItemCheckToggled(Item* item, bool isChecked);
    void activateCurrentMediaItem();
    bool onPlaybackInitialized(double time);
    void onPlaybackStarted(double time);
    bool onTimeChanged(double time);
    double onPlaybackStopped(double time, bool isStoppedManually);
    void stopPlayback();
    void onZoomPropertyChanged();
    bool storeState(Archive& archive);
    bool restoreState(const Archive& archive);
};

}

static GstBusSyncReply busSyncHandler(GstBus* bus, GstMessage* message, gpointer instance)
{
    return ((GSMediaViewImpl*)instance)->onBusMessageSync(message);
}

static GstPadProbeReturn videoPadBufferProbeCallback(GstPad* pad, GstPadProbeInfo* info, gpointer instance)
{
    return ((GSMediaViewImpl*)instance)->onVideoPadGotBuffer(pad, info);
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
            MessageView::mainInstance()->put(_("GStreamer cannot be initialized: "));
            if(error){
                MessageView::mainInstance()->putln(error->message);
                g_error_free(error);
            } else {
                MessageView::mainInstance()->putln("");
            }
            return false;
        }

        ext->viewManager().registerClass<GSMediaView>(N_("MediaView"), N_("Media"));

        if(auto optionsMenu = MainMenu::instance()->get_Options_Menu()){
            MenuManager& mm = ext->menuManager();
            mm.setCurrent(optionsMenu).setPath(N_("Media View"));
            aspectRatioCheck = mm.addCheckItem(_("Keep Aspect Ratio"));
            orgSizeCheck = mm.addCheckItem(_("Keep Original Size"));
        } else {
            aspectRatioCheck = new Action;
            orgSizeCheck = new Action;
        }
        aspectRatioCheck->setChecked(true);
        orgSizeCheck->setChecked(true);

        initialized = true;
    }

    return initialized;
}


GSMediaView::GSMediaView()
{
    setName(N_("Media"));
    setDefaultLayoutArea(CenterArea);

    setAttribute(Qt::WA_NativeWindow);
    setAttribute(Qt::WA_DontCreateNativeAncestors);
    setAttribute(Qt::WA_PaintOnScreen);
    setAttribute(Qt::WA_OpaquePaintEvent);

    impl = new GSMediaViewImpl(this);
}


GSMediaViewImpl::GSMediaViewImpl(GSMediaView* self)
    : self(self),
      mv(MessageView::mainInstance()),
      timeBar(TimeBar::instance()),
      playbin(0),
      videoSink(0),
      windowId(0),
      display(0),
      gc(0)
{
    playbin = gst_element_factory_make("playbin", NULL);

    if(!playbin){
        mv->putln(_("Initialization of the GSMediaView failed. The playbin plugin is missing."));
        return;
    }

    videoSink = gst_element_factory_make("xvimagesink", NULL);
    if(videoSink){
        GstStateChangeReturn sret = gst_element_set_state(GST_ELEMENT(videoSink), GST_STATE_READY);
        if(sret != GST_STATE_CHANGE_SUCCESS){
            gst_element_set_state(GST_ELEMENT(videoSink), GST_STATE_NULL);
            gst_object_unref(videoSink);
            videoSink = nullptr;
        }
    }
    if(!videoSink){
        videoSink = gst_element_factory_make("ximagesink", NULL);
        if(videoSink){
            GstStateChangeReturn sret = gst_element_set_state(GST_ELEMENT(videoSink), GST_STATE_READY);
            if(sret != GST_STATE_CHANGE_SUCCESS){
                gst_element_set_state(GST_ELEMENT(videoSink), GST_STATE_NULL);
                gst_object_unref(videoSink);
                videoSink = nullptr;
            }
        }
    }
    if(!videoSink){
        mv->putln(_("Initialization of the GSMediaView failed. The ximagesink could not be created."));
        return;
    }

    g_object_set(G_OBJECT(videoSink), "force-aspect-ratio", (gboolean)TRUE, NULL);
    g_object_set(G_OBJECT(videoSink), "pixel-aspect-ratio", "1/1", NULL);
    g_object_set(G_OBJECT(playbin), "video-sink", videoSink, NULL);

    GstBus* bus = gst_pipeline_get_bus(GST_PIPELINE(playbin));
    gst_bus_set_sync_handler(GST_BUS(bus), busSyncHandler, (gpointer)this, NULL);
    gst_object_unref(GST_OBJECT(bus));

    videoWidth = -1;
    videoHeight = -1;
    
    isPlaying = false;
    isSeeking = false;
    hasPendingSeek = false;
    currentSeekPos = 0;
    
    pendingSeekTimer.setSingleShot(true);
    pendingSeekTimer.setInterval(200);
    pendingSeekTimer.sigTimeout().connect(
        [&](){ checkPendingSeek(); });

    connections.add(
        aspectRatioCheck->sigToggled().connect(
            [&](bool){ onZoomPropertyChanged(); }));

    connections.add(
        orgSizeCheck->sigToggled().connect(
            [&](bool){ onZoomPropertyChanged(); }));

    connections.add(
        RootItem::instance()->sigCheckToggled().connect(
            [&](Item* item, bool isChecked){
                onItemCheckToggled(item, isChecked); }));
}


GSMediaView::~GSMediaView()
{
    delete impl;
}


GSMediaViewImpl::~GSMediaViewImpl()
{
    if(playbin){
        stopPlayback();

        // unset busSyncHandler
        GstBus* bus = gst_pipeline_get_bus(GST_PIPELINE(playbin));
        gst_bus_set_sync_handler(GST_BUS(bus), NULL, NULL, NULL);
        gst_object_unref(GST_OBJECT(bus));
    }
    timeBarConnections.disconnect();

    if(playbin){
        gst_object_unref(GST_OBJECT(playbin));
    }

    if(display && gc){
        XFreeGC(display, gc);
    }
}


bool GSMediaView::event(QEvent* event)
{
    if(event->type() == QEvent::WinIdChange){
        impl->onWindowIdChanged();
    }
    return QWidget::event(event);
}


void GSMediaViewImpl::onWindowIdChanged()
{
    windowId = self->winId();
    if(display && gc){
        XFreeGC(display, gc);
    }

    display = nullptr;
    int appScreen = 0;

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    if(auto x11App = qGuiApp->nativeInterface<QNativeInterface::QX11Application>()){
        display = x11App->display();
        appScreen = DefaultScreen(display);
    }
#else
    display = QX11Info::display();
    appScreen = QX11Info::appScreen();
#endif

    gc = XCreateGC(display, windowId, 0, 0);
    unsigned long black = BlackPixel(display, appScreen);
    XSetForeground(display, gc, black);
    gst_video_overlay_set_window_handle(GST_VIDEO_OVERLAY(playbin), windowId);
}


void GSMediaView::resizeEvent(QResizeEvent* event)
{
    impl->updateRenderRectangle();

    if(!impl->isPlaying && impl->currentMediaItem){
        impl->seek();
    }
}


void GSMediaViewImpl::updateRenderRectangle()
{
    // Get devicePixelRatio for this widget to convert Qt logical coordinates
    // to physical X11/GStreamer coordinates
    qreal dpr = 1.0;
    if(auto window = self->windowHandle()){
        dpr = window->devicePixelRatio();
    }

    // Get logical dimensions from Qt widget
    int logicalWidth = self->width();
    int logicalHeight = self->height();

    // Convert to physical pixels for X11/GStreamer
    int physicalWidth = qRound(logicalWidth * dpr);
    int physicalHeight = qRound(logicalHeight * dpr);

    // Don't update render rectangle if widget doesn't have valid size yet
    if(physicalWidth <= 0 || physicalHeight <= 0){
        return;
    }

    int x = 0;
    int y = 0;
    int width = physicalWidth;
    int height = physicalHeight;

    if(orgSizeCheck->isChecked()){
        if(videoWidth > 0 && videoHeight > 0){
            // videoWidth/videoHeight are already in physical pixels
            x = (physicalWidth - videoWidth) / 2;
            y = (physicalHeight - videoHeight) / 2;
            width = videoWidth;
            height = videoHeight;
        }
    } else {
        // Scale border size by DPI ratio
        int border = qRound(1 * dpr);
        int minSize = border * 2 + 1;

        if(physicalWidth >= minSize && physicalHeight >= minSize){
            x = border;
            y = border;
            width = physicalWidth - 2 * border;
            height = physicalHeight - 2 * border;
        }
    }

    gst_video_overlay_set_render_rectangle(GST_VIDEO_OVERLAY(videoSink), x, y, width, height);

    // Background region calculation - use physical pixels for X11 painting
    QRegion background = QRect(0, 0, physicalWidth, physicalHeight);
    background = background.subtracted(QRegion(x, y, width, height));
    rects.clear();
    rects.reserve(background.rectCount());
    for(auto& qrect : background){
        XRectangle rect;
        rect.x = qrect.x();
        rect.y = qrect.y();
        rect.width = qrect.width();
        rect.height = qrect.height();
        rects.push_back(rect);
    }
}


QPaintEngine* GSMediaView::paintEngine () const
{
    return 0;
}


void GSMediaView::paintEvent(QPaintEvent* event)
{
    if(!impl->currentMediaItem){
        XFillRectangle(impl->display, impl->windowId, impl->gc, 0, 0, width(), height());
    } else if(impl->rects.size() > 0){
        XFillRectangles(impl->display, impl->windowId, impl->gc, &impl->rects[0], impl->rects.size());
    }
    if(!impl->isPlaying && impl->currentMediaItem){
        impl->seek();
    }
}


void GSMediaView::onActivated()
{
    if(!impl->isPlaying && impl->currentMediaItem){
        impl->seek();
    }
}


void GSMediaView::onDeactivated()
{

}


GstBusSyncReply GSMediaViewImpl::onBusMessageSync(GstMessage* message)
{
    if(TRACE_FUNCTIONS2){
        cout << "GSMediaViewImpl::onBusMessageSync(" << GST_MESSAGE_TYPE_NAME(message) << ")" << endl;
    }

    callLater([this, message](){ onBusMessageAsync(gst_message_copy(message)); });

    return GST_BUS_DROP;
}


void GSMediaViewImpl::onBusMessageAsync(GstMessage* msg)
{
    if(TRACE_FUNCTIONS2){
        cout << "GSMediaView::onBusMessageAsync(" << GST_MESSAGE_TYPE_NAME(msg) << ")" << endl;
    }
    
    switch(GST_MESSAGE_TYPE(msg)){

    case GST_MESSAGE_ASYNC_DONE:
        checkPendingSeek();
        break;
        
    case GST_MESSAGE_EOS:
        isEOS = true;
        break;

    case GST_MESSAGE_ERROR:
        putGstMessage(msg, gst_message_parse_error, "GStreamer error");
        stopPlayback();
        break;

    case GST_MESSAGE_WARNING:
        putGstMessage(msg, gst_message_parse_warning, "GStreamer warning");
        break;

    case GST_MESSAGE_INFO:
        putGstMessage(msg, gst_message_parse_info, "GStreamer info");
        break;

    default:
        break;
    }

    gst_message_unref(msg);
}


void GSMediaViewImpl::putGstMessage
(GstMessage* message, std::function<void(GstMessage* message, GError** gerror, gchar** debug)> parse, const char* prefix)
{
    gchar* debug;
    GError* err;
    parse(message, &err, &debug);
    g_free(debug);
    mv->putln(formatC("{0}: {1}", prefix, err->message), MessageView::Highlight);
    g_error_free(err);
}
    

GstPadProbeReturn GSMediaViewImpl::onVideoPadGotBuffer(GstPad* pad, GstPadProbeInfo* info)
{
    if(TRACE_FUNCTIONS2){
        cout << "GSMediaView::onVideoPadGotBuffer()" << endl;
    }

    GstBuffer* buffer = GST_PAD_PROBE_INFO_BUFFER(info);
    
    if(buffer){
        GstCaps* caps = gst_pad_get_current_caps(pad);
        const GstStructure* structure = gst_caps_get_structure(caps, 0);
        if(gst_structure_get_int(structure, "width", &videoWidth) &&
           gst_structure_get_int(structure, "height", &videoHeight)){
            callLater([&](){ updateRenderRectangle(); });
        }
    }

    return GST_PAD_PROBE_REMOVE;
}


void GSMediaViewImpl::seek(double time)
{
    if(TRACE_FUNCTIONS){
        cout << "GSMediaView::seek(" << time << ")" << endl;
    }

    currentSeekPos = (time + currentMediaItem->offsetTime()) * GST_SECOND;
    seek();
}


void GSMediaViewImpl::seek()
{
    if(TRACE_FUNCTIONS){
        cout << "GSMediaViewImpl::seek()" << endl;
    }

    if(isSeeking){
        hasPendingSeek = true;
        if(!pendingSeekTimer.isActive()){
            /**
               This is needed to avoid the dead lock because sometimes the bus does not
               returns the ASYNC_DONE message after calling the seek function.
               This bug is reported in the following page:
               https://bugzilla.gnome.org/show_bug.cgi?id=740121
            */
            pendingSeekTimer.start();
        }
    } else {
        isSeeking = true;
        hasPendingSeek = false;
        gst_element_seek_simple(
            playbin, GST_FORMAT_TIME,
            (GstSeekFlags)(GST_SEEK_FLAG_ACCURATE|GST_SEEK_FLAG_FLUSH),
            std::max((gint64)0, currentSeekPos));
    }
}


void GSMediaViewImpl::checkPendingSeek()
{
    isSeeking = false;
    
    if(hasPendingSeek){
        hasPendingSeek = false;
        pendingSeekTimer.stop();
        seek();
    }
}
        

void GSMediaViewImpl::onItemCheckToggled(Item* item, bool isChecked)
{
    if(TRACE_FUNCTIONS){
        cout << "GSMediaViewImpl::onItemCheckToggled()" << endl;
    }
    
    MediaItem* mediaItem = dynamic_cast<MediaItem*>(item);
    if(mediaItem){
        auto checkedItems = RootItem::instance()->checkedItems<MediaItem>();
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
                self->update();  // Trigger repaint to clear the view

            } else {
                GstPad* pad = gst_element_get_static_pad(GST_ELEMENT(videoSink), "sink");
                gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_BUFFER,
                                  (GstPadProbeCallback)videoPadBufferProbeCallback, this, NULL);
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
        seek(timeBar->time());

        g_object_set(G_OBJECT(videoSink), "show-preroll-frame", (gboolean)TRUE, NULL);

        if(timeBarConnections.empty()){
            timeBarConnections.add(
                timeBar->sigPlaybackInitialized().connect(
                    [&](double time){ return onPlaybackInitialized(time); }));
            timeBarConnections.add(
                timeBar->sigPlaybackStarted().connect(
                    [&](double time){ onPlaybackStarted(time); }));
            timeBarConnections.add(
                timeBar->sigPlaybackStoppedEx().connect(
                    [&](double time, bool isStoppedManually){ return onPlaybackStopped(time, isStoppedManually);}));
            timeBarConnections.add(
                timeBar->sigTimeChanged().connect(
                    [&](double time){ return onTimeChanged(time); }));
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
    if(TRACE_FUNCTIONS){
        cout << "GSMediaViewImpl::onTimeChanged(" << time << ")" << endl;
    }
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


double GSMediaViewImpl::onPlaybackStopped(double time, bool isStoppedManually)
{
    if(TRACE_FUNCTIONS){
        cout << "GSMediaViewImpl::onPlaybackStopped()" << endl;
    }

    double lastValidTime = time;

    if(isPlaying){
        // Query GStreamer for the actual video duration
        gint64 duration_ns = 0;
        if(gst_element_query_duration(playbin, GST_FORMAT_TIME, &duration_ns)){
            // Convert nanoseconds to seconds and subtract offset
            double videoDuration = (double)duration_ns / GST_SECOND;
            if(currentMediaItem){
                double videoEndTime = videoDuration - currentMediaItem->offsetTime();
                lastValidTime = videoEndTime;
            }
        }

        GstStateChangeReturn ret = gst_element_set_state(GST_ELEMENT(playbin), GST_STATE_PAUSED);
        if(TRACE_FUNCTIONS){
            cout << "ret = " << ret << endl;
        }
        isPlaying = false;
    }

    return lastValidTime;
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

    // Clear the video overlay window to remove the last frame
    if(display && windowId && gc){
        qreal dpr = 1.0;
        if(auto window = self->windowHandle()){
            dpr = window->devicePixelRatio();
        }
        int physicalWidth = qRound(self->width() * dpr);
        int physicalHeight = qRound(self->height() * dpr);
        XFillRectangle(display, windowId, gc, 0, 0, physicalWidth, physicalHeight);
        XFlush(display);  // Ensure X11 commands are executed immediately
    }

    isPlaying = false;
}


void GSMediaViewImpl::onZoomPropertyChanged()
{
    if(orgSizeCheck->isChecked()){
        g_object_set(G_OBJECT(videoSink), "force-aspect-ratio", (gboolean)FALSE, NULL);
        updateRenderRectangle();
        self->update();
    } else {
        g_object_set(G_OBJECT(videoSink), "force-aspect-ratio", (gboolean)aspectRatioCheck->isChecked(), NULL);
        updateRenderRectangle();
        gst_video_overlay_expose(GST_VIDEO_OVERLAY(videoSink));
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
