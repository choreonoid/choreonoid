/**
   @author Saeki, Keisuke
   @author Shin'ichiro Nakaoka
*/

#include "DSMediaView.h"
#include "MediaItem.h"
#include <cnoid/TimeBar>
#include <cnoid/Archive>
#include <cnoid/ConnectionSet>
#include <cnoid/MessageView>
#include <cnoid/ViewManager>
#include <cnoid/ItemTreeView>
#include <cnoid/MenuManager>
#include <cnoid/Sleep>
#include <QPainter>
#include <Dshow.h>
#include <iostream>
#include <algorithm>
#include <windows.h>
#include <mbctype.h>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace std::placeholders;
using namespace cnoid;
using fmt::format;

namespace {
const bool TRACE_FUNCTIONS = false;

bool initialized = false;

Action* aspectRatioCheck = 0;
Action* orgSizeCheck = 0;

const UINT WM_DSHOW_NOTIFY = WM_USER + 21;

}

namespace cnoid {

class DSMediaViewImpl
{
public:
    DSMediaViewImpl(DSMediaView* self);
    ~DSMediaViewImpl();

    DSMediaView* self;
    MessageView* mv;
    ConnectionSet timeBarConnections;
    MediaItemPtr currentMediaItem;

    enum PLAYSTATE { Empty, Stopped, Paused, Running } currentState;

    // DirectShow interfaces
    IGraphBuilder*   graphBuilder;
    IMediaControl*   mediaControl;
    IMediaEventEx*   mediaEvent;
    IMediaSeeking*   mediaSeeking;
    IVideoWindow*    videoWindow;
    IBasicVideo*     basicVideo;
    IVideoFrameStep* videoFrameStep;

    // windows handle
    HWND hwnd;

    bool bMapped;
    bool bLoaded;
    bool bCanSeek;

    // Original Window Procedure
    WNDPROC orgWinProc;

    void onWindowIdChanged();
    void onAspectRatioCheckToggled();
    void onOrgSizeCheckToggled();
    void onItemCheckToggled(Item* item, bool isChecked);
    void initializeScreenWindow();
    void clearScreenWindow();
    void load();
    void unload();
    HRESULT adjustVideoWindow();
    void connectTimeBarSignals();
    bool onPlaybackInitialized(double time);
    void onPlaybackStarted(double time);
    bool onTimeChanged(double time);
    void onPlaybackStopped(double time);
    HRESULT seekMedia(double time);
    HRESULT playMedia();
    HRESULT pauseMedia();
    HRESULT stopMedia();
    HRESULT HandleDShowEvent();
    bool storeState(Archive& archive);
    bool restoreState(const Archive& archive);
};
}


bool DSMediaView::initialize(ExtensionManager* ext)
{
    if(TRACE_FUNCTIONS){
        cout << "DSMediaView::initialize()" << endl;
    }
    
    if(!initialized){

        if(!SUCCEEDED(CoInitializeEx(NULL, COINIT_APARTMENTTHREADED))){
            MessageView::mainInstance()->putln(_("CoInitialize failed. MediaView is not available."));
            return false;
        }

        ext->viewManager().registerClass<DSMediaView>(
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


void DSMediaView::finalize()
{
    if(TRACE_FUNCTIONS){
        cout << "DSMediaView::finalize()" << endl;
    }
    
    if(initialized){
        CoUninitialize();
    }
}


DSMediaView::DSMediaView()
{
    if(TRACE_FUNCTIONS){
        cout << "DSMediaView::DSMediaView()" << endl;
    }
    
    setName(N_("Media"));
    setDefaultLayoutArea(View::CENTER);
    
    setAttribute(Qt::WA_NativeWindow);
    setAttribute(Qt::WA_PaintOnScreen);
    setAttribute(Qt::WA_OpaquePaintEvent);

    impl = new DSMediaViewImpl(this);
}


DSMediaViewImpl::DSMediaViewImpl(DSMediaView* self)
    : self(self),
      mv(MessageView::mainInstance())
{
    if(TRACE_FUNCTIONS){
        cout << "DSMediaViewImpl::DSMediaViewImpl()" << endl;
    }
    
    graphBuilder = 0;
    mediaControl = 0;
    mediaEvent = 0;
    videoWindow = 0;
    basicVideo = 0;
    mediaSeeking = 0;
    videoFrameStep = 0;
    hwnd = 0;
    
    bMapped = false;
    bLoaded = false;
    bCanSeek = false;
    
    currentState = Empty;

    orgWinProc = NULL;

    aspectRatioCheck->sigToggled().connect(
        std::bind(&DSMediaViewImpl::onAspectRatioCheckToggled, this));

    orgSizeCheck->sigToggled().connect(
        std::bind(&DSMediaViewImpl::onOrgSizeCheckToggled, this));
    
    ItemTreeView::mainInstance()->sigCheckToggled().connect(
        std::bind(&DSMediaViewImpl::onItemCheckToggled, this, _1, _2));
}


DSMediaView::~DSMediaView()
{
    if(TRACE_FUNCTIONS){
        cout << "DSMediaView::~DSMediaView()" << endl;
    }
    
    delete impl;
}


DSMediaViewImpl::~DSMediaViewImpl()
{
    timeBarConnections.disconnect();
}


/*
  bool DSMediaView::event(QEvent* event)
  {
  if(TRACE_FUNCTIONS){
  cout << "DSMediaView::event()" << endl;
  }
    
  if(event->type() == QEvent::WinIdChange){
  impl->onWindowIdChanged();
  return true;
  }
  //return QWidget::event(event);

  return false;
  }
*/


namespace {
    
LRESULT CALLBACK videoWindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    if(TRACE_FUNCTIONS){
        cout << "DSMediaView::videoWindowProc()" << endl;
    }
    DSMediaViewImpl* impl = (DSMediaViewImpl*)GetWindowLongPtr(hwnd, GWLP_USERDATA);
    switch(uMsg) {
    case WM_DSHOW_NOTIFY:
        impl->HandleDShowEvent();
        break;
    }
    return CallWindowProc(impl->orgWinProc, hwnd, uMsg, wParam, lParam);
}
}


void DSMediaViewImpl::onWindowIdChanged()
{
    hwnd = (HWND)self->winId();
    
    if(TRACE_FUNCTIONS){
        cout << "DSMediaView::onWindowIdChanged(): hwnd = " << hwnd << endl;
    }

    initializeScreenWindow();
}


void DSMediaView::resizeEvent(QResizeEvent* event)
{
    if(TRACE_FUNCTIONS){
        cout << "DSMediaView::resizeEvent()" << endl;
    }

    if(impl->bLoaded){
        impl->adjustVideoWindow();
    }
}


void DSMediaView::paintEvent(QPaintEvent* event)
{
    if(TRACE_FUNCTIONS){
        cout << "DSMediaView::paintEvent()" << endl;
    }

    QPainter painter(this);

    painter.fillRect(0, 0, width(), height(), QColor(Qt::black));

    
    
}


QPaintEngine* DSMediaView::paintEngine () const
{
    //return 0;
    return View::paintEngine();
}


void DSMediaView::onActivated()
{
    if(TRACE_FUNCTIONS){
        cout << "DSMediaView::onActivated()" << endl;
    }
    impl->initializeScreenWindow();
}


void DSMediaView::onDeactivated()
{
    if(impl->bLoaded){
        impl->unload();
    }

    impl->clearScreenWindow();
}


void DSMediaViewImpl::onAspectRatioCheckToggled()
{
    if(bLoaded){
        adjustVideoWindow();
    }
}


void DSMediaViewImpl::onOrgSizeCheckToggled()
{
    if(bLoaded){
        adjustVideoWindow();
    }
}


void DSMediaViewImpl::onItemCheckToggled(Item* item, bool isChecked)
{
    if(TRACE_FUNCTIONS){
        cout << "DSMediaViewImpl::onItemCheckToggled()" << endl;
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
                if(bLoaded){
                    unload();
                }
            }
            currentMediaItem = targetItem;

            if(bMapped && currentMediaItem){
                load();
            }
        }
    }
}


void DSMediaViewImpl::initializeScreenWindow()
{
    hwnd = (HWND)self->winId();
    
    if(TRACE_FUNCTIONS){
        cout << "DSMediaView::initializeScreenWindow(): hwnd = " << hwnd << endl;
    }

    if(hwnd != NULL && orgWinProc == NULL){

#ifdef _WIN64
        orgWinProc = (WNDPROC)GetWindowLongPtr(hwnd, GWLP_WNDPROC);
        
        //SetWindowLong(hwnd, GWL_WNDPROC, (LONG)(WNDPROC)videoWindowProc);
        SetWindowLongPtr(hwnd, GWLP_WNDPROC, (LONG_PTR)videoWindowProc);
        SetWindowLongPtr(hwnd, GWLP_USERDATA, (LONG_PTR)this);
#else
        orgWinProc = (WNDPROC)GetWindowLong(hwnd, GWL_WNDPROC);

        //SetWindowLong(hwnd, GWL_WNDPROC, (LONG)(WNDPROC)videoWindowProc);
        SetWindowLongPtr(hwnd, GWL_WNDPROC, (LONG_PTR)videoWindowProc);
        SetWindowLongPtr(hwnd, GWL_USERDATA, (LONG_PTR)this);
#endif


        bMapped = true;
        if(currentMediaItem){
            load();
        }
    }
}


void DSMediaViewImpl::clearScreenWindow()
{
    if(hwnd != NULL && orgWinProc != NULL){

#ifdef _WIN64
        SetWindowLongPtr(hwnd, GWLP_WNDPROC, (LONG_PTR)orgWinProc);
#else
        SetWindowLongPtr(hwnd, GWL_WNDPROC, (LONG_PTR)orgWinProc);
#endif

    }

    orgWinProc = NULL;
    hwnd = NULL;
    bMapped = false;
}    


void DSMediaViewImpl::load()
{
    if(TRACE_FUNCTIONS){
        cout << "DSMediaView::load()" << endl;
    }
    
    HRESULT result;

#define EIF(x)                                                          \
    result = (x);                                                       \
    if(FAILED(result)) {                                                \
        throw DSException(format("FAILED(hr=0x{:x}) in" TEXT(#x), result)); \
    }

    struct DSException {
        DSException(std::string m) : message(m) { }
        std::string message;
    };
    
    try{
        // Get the interface for DirectShow's GraphBuilder
        EIF(CoCreateInstance(CLSID_FilterGraph, NULL, CLSCTX_INPROC_SERVER,
                             IID_IGraphBuilder, (void**)&graphBuilder));

        WCHAR wFile[MAX_PATH];

        // Convert filename to wide character string
        MultiByteToWideChar(_getmbcp(), 0, currentMediaItem->mediaURI().c_str(), -1, wFile, sizeof(wFile));

        // Have the graph builder construct its the appropriate graph automatically
        EIF(graphBuilder->RenderFile(wFile, NULL));

        // QueryInterface for DirectShow interfaces
        EIF(graphBuilder->QueryInterface(IID_IMediaControl, (void**)&mediaControl));
        EIF(graphBuilder->QueryInterface(IID_IMediaEventEx, (void**)&mediaEvent));
        EIF(graphBuilder->QueryInterface(IID_IMediaSeeking, (void**)&mediaSeeking));
        
        // Check seekable
        DWORD caps = AM_SEEKING_CanSeekAbsolute | AM_SEEKING_CanGetDuration; 
        bCanSeek = (S_OK == mediaSeeking->CheckCapabilities(&caps));
        if(!bCanSeek){
            mv->putln(_("This media file is not able to seek!"));
        }

        // Query for video interfaces, which may not be relevant for audio files
        EIF(graphBuilder->QueryInterface(IID_IVideoWindow, (void**)&videoWindow));
        EIF(graphBuilder->QueryInterface(IID_IBasicVideo, (void**)&basicVideo));

        // Have the graph signal event via window callbacks for performance
        EIF(mediaEvent->SetNotifyWindow((OAHWND)hwnd, WM_DSHOW_NOTIFY, 0));
        
        // Setup the video window
        EIF(videoWindow->put_Owner((OAHWND)hwnd));
        EIF(videoWindow->put_WindowStyle(WS_CHILD | WS_CLIPSIBLINGS));

        EIF(adjustVideoWindow());

        EIF(videoWindow->SetWindowForeground(OATRUE));
        EIF(videoWindow->put_Visible(OATRUE));

        connectTimeBarSignals();

        seekMedia(TimeBar::instance()->time());
        mediaControl->StopWhenReady();

        currentState = Stopped;
        bLoaded = true;

    } catch (const DSException& ex){
        mv->putln(ex.message);
        bLoaded = false;
    }
}


void DSMediaViewImpl::unload()
{
    if(TRACE_FUNCTIONS){
        cout << "DSMediaView::unload()" << endl;
    }
    
#define SAFE_RELEASE(x) { if(x) x->Release(); x = 0; }
    
    timeBarConnections.disconnect();

    // Release and zero DirectShow interfaces
    SAFE_RELEASE(mediaEvent);
    SAFE_RELEASE(mediaSeeking);
    SAFE_RELEASE(mediaControl);
    SAFE_RELEASE(basicVideo);
    SAFE_RELEASE(videoWindow);
    SAFE_RELEASE(videoFrameStep);
    SAFE_RELEASE(graphBuilder);

    currentState = Empty;

    bLoaded = false;
}


HRESULT DSMediaViewImpl::adjustVideoWindow()
{
    if(TRACE_FUNCTIONS){
        cout << "DSMediaView::adjustVideoWindow()" << endl;
    }
    
    if(videoWindow && basicVideo){
        LONG screenWidth = self->width();
        LONG screenHeight = self->height();
        LONG width, height;
        if(basicVideo->GetVideoSize(&width, &height) != E_NOINTERFACE){
            if(orgSizeCheck->isChecked()){
                if(width <= screenWidth && height <= screenHeight){
                    return videoWindow->SetWindowPosition(
                        (screenWidth - width) / 2, (screenHeight - height) / 2, width, height);
                } else {
                    return videoWindow->SetWindowPosition(0, 0, width, height);
                }
            } else if(aspectRatioCheck->isChecked()){
                double r1 = (double)screenWidth / width;
                double r2 = (double)screenHeight / height;
                LONG w = (LONG)(width * (std::min)(r1, r2));
                LONG h = (LONG)(height * (std::min)(r1, r2));
                return videoWindow->SetWindowPosition(
                    (screenWidth - w) / 2, (screenHeight - h) / 2, w, h);
            }
        }
        return videoWindow->SetWindowPosition(0, 0, screenWidth, screenHeight);
    }

    return S_OK;
}


void DSMediaViewImpl::connectTimeBarSignals()
{
    if(timeBarConnections.empty()){
        TimeBar* timeBar = TimeBar::instance();
        timeBarConnections.add(
            timeBar->sigPlaybackInitialized().connect(
                std::bind(&DSMediaViewImpl::onPlaybackInitialized, this, _1)));
        timeBarConnections.add(
            timeBar->sigPlaybackStarted().connect(
                std::bind(&DSMediaViewImpl::onPlaybackStarted, this, _1)));
        timeBarConnections.add(
            timeBar->sigPlaybackStopped().connect(
                std::bind(&DSMediaViewImpl::onPlaybackStopped, this, _1)));
        timeBarConnections.add(
            timeBar->sigTimeChanged().connect(
                std::bind(&DSMediaViewImpl::onTimeChanged, this, _1)));
    }
}


bool DSMediaViewImpl::onPlaybackInitialized(double time)
{
    seekMedia(time);
    pauseMedia();
    OAFilterState state;
    mediaControl->GetState(1000, &state);
    return true;
}


void DSMediaViewImpl::onPlaybackStarted(double time)
{
    playMedia();
}


bool DSMediaViewImpl::onTimeChanged(double time)
{
    if(currentState == Running){
        return true;
    } else {
        seekMedia(time);
        mediaControl->StopWhenReady();
    }
    return false;
}


void DSMediaViewImpl::onPlaybackStopped(double time)
{
    pauseMedia();
}


HRESULT DSMediaViewImpl::seekMedia(double time)
{
    HRESULT result = S_OK;
    if(bCanSeek){
        LONGLONG llNow;
        const LONGLONG ONE_SECOND = 10000000;
        llNow = (LONGLONG)((time + currentMediaItem->offsetTime()) * ONE_SECOND);
        result = mediaSeeking->SetPositions(
            &llNow, AM_SEEKING_AbsolutePositioning, NULL, AM_SEEKING_NoPositioning);
    }
    return result;
}


HRESULT DSMediaViewImpl::playMedia()
{
    HRESULT result = S_OK;
    if(currentState == Stopped || currentState == Paused){
        result = mediaControl->Run();
        if(FAILED(result)){
            mv->putln("MediaView failed to play media");
        } else {
            currentState = Running;
        }
    }
    return result;
}


HRESULT DSMediaViewImpl::pauseMedia()
{
    HRESULT result = S_OK;
    if(currentState == Running){
        result = mediaControl->Pause();
        if(FAILED(result)){
            mv->putln("MediaView failed to pause media");
        } else {
            currentState = Paused;
        }
    }
    return result;
}


HRESULT DSMediaViewImpl::stopMedia()
{
    HRESULT result = S_OK;
    if(currentState == Running || currentState == Paused){
        result = mediaControl->Stop();
        currentState = Stopped;
        seekMedia(0.0);
        result = mediaControl->Pause();
        if(FAILED(result)){
            mv->putln("MediaView failed to pause media");
        } else {
            currentState = Paused;
        }
    }
    return result;
}


HRESULT DSMediaViewImpl::HandleDShowEvent()
{
    if(TRACE_FUNCTIONS){
        cout << "DSMediaView::HandleDShowEvent()" << endl;
    }
    
    // Make sure that we don't access the media event interface
    // after it has already been released.
    if(!mediaEvent){
        return S_OK;
    }

    LONG evCode;
    LONG_PTR evParam1, evParam2;
    HRESULT result = S_OK;

    // Process all queued events
    while(SUCCEEDED(mediaEvent->GetEvent(&evCode, &evParam1, &evParam2, 0))){
        // Free memory associated with callback, since we're not using it
        result = mediaEvent->FreeEventParams(evCode, evParam1, evParam2);
        // If this is the end of the clip, reset to beginning
        if(EC_COMPLETE == evCode){
            LONGLONG pos = 0;
            if(SUCCEEDED(result = mediaControl->Pause())){
                currentState = Paused;
            } else {
                if (SUCCEEDED(result = mediaControl->Stop())){
                    currentState = Stopped;
                } else {
                    mv->putln(format(_("Failed({}) to stop media clip!\n"), result));
                    unload();
                    break;
                }
            }
            /*
            //os << "repeat!!" << endl;
            // Reset to first frame of movie
            hr = mediaSeeking->SetPositions(&pos, AM_SEEKING_AbsolutePositioning ,
            NULL, AM_SEEKING_NoPositioning);
            if (FAILED(hr))
            {
            // Some custom filters (like the Windows CE MIDI filter)
            // may not implement seeking interfaces (IMediaSeeking)
            // to allow seeking to the start.  In that case, just stop
            // and restart for the same effect.  This should not be
            // necessary in most cases.
            if (FAILED(hr = mediaControl->Stop()))
            {
            os << fmt("Failed(0x%08lx) to stop media clip!\n") % hr << endl;
            break;
            }

            if (FAILED(hr = mediaControl->Run()))
            {
            os << fmt("Failed(0x%08lx) to reset media clip!\n") % hr << endl;
            break;
            }
            }
            */
        }
    }

    return result;
}


bool DSMediaView::storeState(Archive& archive)
{
    return impl->storeState(archive);
}


bool DSMediaViewImpl::storeState(Archive& archive)
{
    if(TRACE_FUNCTIONS){
        cout << "DSMediaViewImpl::storeState()" << endl;
    }
    archive.write("keepAspectRatio", aspectRatioCheck->isChecked());
    archive.write("keepOriginalSize", orgSizeCheck->isChecked());
    return true;
}


bool DSMediaView::restoreState(const Archive& archive)
{
    return impl->restoreState(archive);
}


bool DSMediaViewImpl::restoreState(const Archive& archive)
{
    if(TRACE_FUNCTIONS){
        cout << "DSMediaViewImpl::restoreState()" << endl;
    }
    aspectRatioCheck->setChecked(archive.get("keepAspectRatio", aspectRatioCheck->isChecked()));
    orgSizeCheck->setChecked(archive.get("keepOriginalSize", orgSizeCheck->isChecked()));
    return true;
}
