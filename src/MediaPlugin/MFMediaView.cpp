#include "MFMediaView.h"
#include "MediaItem.h"
#include <cnoid/ViewManager>
#include <cnoid/MenuManager>
#include <cnoid/MessageOut>
#include <cnoid/RootItem>
#include <cnoid/TimeBar>
#include <cnoid/Archive>
#include <cnoid/ConnectionSet>
#include <cnoid/Format>
#include <cnoid/UTF8>
#include <cnoid/stdx/filesystem>
#include <QBoxLayout>
#include <QLabel>
#include <QResizeEvent>
#include <QWindow>
#include <QPainter>
#include <windows.h>
#include <mfapi.h>
#include <mfidl.h>
#include <mfreadwrite.h>
#include <mferror.h>
#include <evr.h>
#include <Shlwapi.h>
#include "gettext.h"

/*
 * KNOWN ISSUES (as of 2025-11-14):
 *
 * 1. Video frame disappears when view is resized while not playing
 *    - When the media is paused or stopped, resizing the view or triggering repaint
 *      causes the last video frame to disappear and the view becomes completely black.
 *    - This issue does NOT occur during playback - only when not playing.
 *
 * 2. Background artifacts when changing video size options during playback
 *    - When changing options (Keep Aspect Ratio, Keep Original Size) during playback,
 *      garbage/artifacts from the previous video frames remain in the background area.
 *    - The artifacts can be cleared by triggering an external repaint event (e.g.,
 *      resizing the view or dragging the main window), but this is not a proper solution.
 *
 * Attempted solutions (all unsuccessful):
 *    - RedrawWindow() with various flags (RDW_INVALIDATE, RDW_UPDATENOW, RDW_ERASE)
 *      Result: Made it worse, video disappeared completely
 *    - QApplication::processEvents() to force event processing
 *      Result: No effect
 *    - InvalidateRect() to invalidate the window region
 *      Result: No effect
 *    - Conditional painting (fill background only when media is not loaded)
 *      Result: Partially helped with stopped state, but artifacts remained
 *    - ExcludeClipRect() to exclude video area before painting background
 *      Result: Everything turned black when resizing
 *    - Empty paintEvent() relying only on EVR's SetBorderColor()
 *      Result: Artifacts never cleared at all
 *    - Current approach: Use QPainter to fill background (same as DSMediaView)
 *      Result: Video displays correctly, but the two issues above remain
 *
 * The root cause appears to be a coordination issue between EVR's direct rendering
 * to the window and Qt's paint system. Further investigation into Media Foundation
 * and EVR internals may be needed to properly resolve these issues.
 */

using namespace std;
using namespace cnoid;
namespace filesystem = cnoid::stdx::filesystem;

namespace {

Action* aspectRatioCheck = nullptr;
Action* orgSizeCheck = nullptr;

}

namespace cnoid {

class MFMediaViewImpl
{
public:
    MFMediaView* self;
    MediaItemPtr currentMediaItem;

    // Media Foundation interfaces
    IMFMediaSession* mediaSession;
    IMFMediaSource* mediaSource;
    IMFActivate* videoRendererActivate;
    IMFVideoDisplayControl* videoDisplayControl;
    IMFClock* clock;
    IMFSimpleAudioVolume* audioVolume;

    // State flags
    bool bMapped;
    bool bLoaded;
    bool bCanSeek;

    // Window handle
    HWND hwnd;
    WNDPROC orgWinProc;

    // TimeBar integration
    ScopedConnectionSet timeBarConnections;

    // Playback state
    enum PlaybackState {
        Stopped,
        Paused,
        Running
    };
    PlaybackState currentState;

    MFMediaViewImpl(MFMediaView* self);
    ~MFMediaViewImpl();

    void onWindowIdChanged();
    void onItemCheckToggled(Item* item, bool isChecked);
    void onAspectRatioCheckToggled();
    void onOrgSizeCheckToggled();

    HRESULT load();
    HRESULT unload();
    HRESULT createTopology(IMFMediaSource* source, IMFTopology** ppTopology);
    HRESULT addBranchToTopology(IMFTopology* topology, IMFMediaSource* source, IMFPresentationDescriptor* presDesc, DWORD streamIndex);
    HRESULT createSourceStreamNode(IMFMediaSource* source, IMFPresentationDescriptor* presDesc, IMFStreamDescriptor* streamDesc, IMFTopologyNode** ppNode);
    HRESULT createOutputNode(IMFStreamDescriptor* streamDesc, IMFTopologyNode** ppNode, IMFActivate** ppActivate);

    HRESULT adjustVideoWindow();
    HRESULT ensureVideoDisplayControl();

    // Playback control
    HRESULT playMedia();
    HRESULT pauseMedia();
    HRESULT stopMedia();
    HRESULT seekMedia(double time);

    // TimeBar callbacks
    void connectTimeBarSignals();
    void disconnectTimeBarSignals();
    bool onPlaybackInitialized(double time);
    void onPlaybackStarted(double time);
    double onPlaybackStopped(double time, bool isStoppedManually);
    bool onTimeChanged(double time);

    // Event handling
    HRESULT handleSessionEvent();
};

}

// Static member for COM/MF initialization
static bool comMfInitialized = false;

bool MFMediaView::initialize(ExtensionManager* ext)
{
    static bool initialized = false;

    if(!initialized){
        // Initialize COM
        HRESULT hr = CoInitializeEx(NULL, COINIT_APARTMENTTHREADED);
        if(FAILED(hr)){
            MessageOut::master()->putErrorln(
                _("Failed to initialize COM for MFMediaView."));
            return false;
        }

        // Initialize Media Foundation
        hr = MFStartup(MF_VERSION, MFSTARTUP_FULL);
        if(FAILED(hr)){
            MessageOut::master()->putErrorln(
                _("Failed to initialize Media Foundation."));
            CoUninitialize();
            return false;
        }

        comMfInitialized = true;

        // Register view
        ext->viewManager().registerClass<MFMediaView>(
            N_("MediaView"), N_("Media"), ViewManager::SINGLE_OPTIONAL);

        // Create menu options
        MenuManager& mm = ext->menuManager();
        mm.setPath("/Options").setPath(N_("Media View"));

        aspectRatioCheck = mm.addCheckItem(_("Keep Aspect Ratio"));
        aspectRatioCheck->setChecked(true);

        orgSizeCheck = mm.addCheckItem(_("Keep Original Size"));
        orgSizeCheck->setChecked(true);

        initialized = true;
    }

    return true;
}

void MFMediaView::finalize()
{
    if(comMfInitialized){
        MFShutdown();
        CoUninitialize();
        comMfInitialized = false;
    }
}

MFMediaView::MFMediaView()
{
    setName(N_("Media"));
    setDefaultLayoutArea(View::CENTER);

    // Set widget attributes for video rendering
    setAttribute(Qt::WA_NativeWindow);
    setAttribute(Qt::WA_PaintOnScreen);
    setAttribute(Qt::WA_OpaquePaintEvent);

    impl = new MFMediaViewImpl(this);
}

MFMediaView::~MFMediaView()
{
    delete impl;
}

bool MFMediaView::storeState(Archive& archive)
{
    if(aspectRatioCheck){
        archive.write("keepAspectRatio", aspectRatioCheck->isChecked());
    }
    if(orgSizeCheck){
        archive.write("keepOriginalSize", orgSizeCheck->isChecked());
    }
    return true;
}

bool MFMediaView::restoreState(const Archive& archive)
{
    if(aspectRatioCheck){
        aspectRatioCheck->setChecked(archive.get("keepAspectRatio", true));
    }
    if(orgSizeCheck){
        orgSizeCheck->setChecked(archive.get("keepOriginalSize", true));
    }
    return true;
}

bool MFMediaView::event(QEvent* event)
{
    if(event->type() == QEvent::WinIdChange){
        impl->onWindowIdChanged();
        return true;
    }
    return QWidget::event(event);
}

void MFMediaView::resizeEvent(QResizeEvent* event)
{
    impl->adjustVideoWindow();
}

void MFMediaView::paintEvent(QPaintEvent* event)
{
    QPainter painter(this);
    painter.fillRect(0, 0, width(), height(), QColor(Qt::black));
}

QPaintEngine* MFMediaView::paintEngine() const
{
    return View::paintEngine();
}

void MFMediaView::onActivated()
{
    impl->bMapped = true;

    // Check for checked MediaItem
    auto items = RootItem::instance()->checkedItems<MediaItem>();
    MediaItemPtr targetItem = items.toSingle(true);

    if(targetItem && targetItem != impl->currentMediaItem){
        impl->currentMediaItem = targetItem;
        impl->load();
    }
}

void MFMediaView::onDeactivated()
{
    if(impl->bLoaded){
        impl->unload();
    }
    impl->bMapped = false;
}

MFMediaViewImpl::MFMediaViewImpl(MFMediaView* self)
    : self(self)
{
    mediaSession = nullptr;
    mediaSource = nullptr;
    videoRendererActivate = nullptr;
    videoDisplayControl = nullptr;
    clock = nullptr;
    audioVolume = nullptr;

    bMapped = false;
    bLoaded = false;
    bCanSeek = true;

    hwnd = nullptr;
    orgWinProc = nullptr;

    currentState = Stopped;

    // Connect to option signals
    if(aspectRatioCheck){
        aspectRatioCheck->sigToggled().connect(
            [this](bool){ onAspectRatioCheckToggled(); });
    }
    if(orgSizeCheck){
        orgSizeCheck->sigToggled().connect(
            [this](bool){ onOrgSizeCheckToggled(); });
    }

    // Connect to item check signal
    RootItem::instance()->sigCheckToggled().connect(
        [this](Item* item, bool isChecked){ onItemCheckToggled(item, isChecked); });
}

MFMediaViewImpl::~MFMediaViewImpl()
{
    if(bLoaded){
        unload();
    }
}

void MFMediaViewImpl::onWindowIdChanged()
{
    hwnd = (HWND)self->winId();

    // Set video window if already loaded
    if(videoDisplayControl && hwnd){
        videoDisplayControl->SetVideoWindow(hwnd);
        adjustVideoWindow();
    }
}

void MFMediaViewImpl::onItemCheckToggled(Item* item, bool isChecked)
{
    MediaItem* mediaItem = dynamic_cast<MediaItem*>(item);
    if(mediaItem){
        MediaItemPtr targetItem = RootItem::instance()->checkedItems<MediaItem>().toSingle(true);

        if(targetItem != currentMediaItem){
            if(currentMediaItem && bLoaded){
                unload();
            }
            currentMediaItem = targetItem;
            if(bMapped && currentMediaItem){
                load();
            }
        }
    }
}

void MFMediaViewImpl::onAspectRatioCheckToggled()
{
    if(bLoaded){
        adjustVideoWindow();
    }
}

void MFMediaViewImpl::onOrgSizeCheckToggled()
{
    if(bLoaded){
        adjustVideoWindow();
    }
}

// Phase 2: Media loading and topology construction
HRESULT MFMediaViewImpl::load()
{
    if(!currentMediaItem){
        return E_FAIL;
    }

    HRESULT hr = S_OK;

    try {
        // Get file path
        filesystem::path fpath(fromUTF8(currentMediaItem->mediaFilePath()));
        if(!filesystem::exists(fpath)){
            MessageOut::master()->putErrorln(
                formatR(_("Media file not found: {}"), fpath.string()));
            return E_FAIL;
        }

        std::wstring wFilePath = fpath.wstring();

        // Create media session
        hr = MFCreateMediaSession(NULL, &mediaSession);
        if(FAILED(hr)){
            MessageOut::master()->putErrorln(
                formatR(_("Failed to create Media Foundation session. HRESULT: 0x{:08X}"), (unsigned int)hr));
            return hr;
        }

        // Create source resolver
        IMFSourceResolver* sourceResolver = nullptr;
        hr = MFCreateSourceResolver(&sourceResolver);
        if(FAILED(hr)){
            MessageOut::master()->putErrorln(
                formatR(_("Failed to create source resolver. HRESULT: 0x{:08X}"), (unsigned int)hr));
            return hr;
        }

        // Create media source from file
        MF_OBJECT_TYPE objectType = MF_OBJECT_INVALID;
        IUnknown* source = nullptr;
        hr = sourceResolver->CreateObjectFromURL(
            wFilePath.c_str(),
            MF_RESOLUTION_MEDIASOURCE,
            NULL,
            &objectType,
            &source);
        sourceResolver->Release();

        if(FAILED(hr)){
            MessageOut::master()->putErrorln(
                formatR(_("Failed to create media source from file: {}. HRESULT: 0x{:08X}"),
                    fpath.string(), (unsigned int)hr));
            return hr;
        }

        hr = source->QueryInterface(IID_PPV_ARGS(&mediaSource));
        source->Release();
        if(FAILED(hr)){
            MessageOut::master()->putErrorln(
                formatR(_("Failed to query IMFMediaSource interface. HRESULT: 0x{:08X}"), (unsigned int)hr));
            return hr;
        }

        // Create topology
        IMFTopology* topology = nullptr;
        hr = createTopology(mediaSource, &topology);
        if(FAILED(hr)){
            MessageOut::master()->putErrorln(
                formatR(_("Failed to create topology. HRESULT: 0x{:08X}"), (unsigned int)hr));
            return hr;
        }

        // Set topology on session
        hr = mediaSession->SetTopology(0, topology);
        topology->Release();
        if(FAILED(hr)){
            MessageOut::master()->putErrorln(
                formatR(_("Failed to set topology on session. HRESULT: 0x{:08X}"), (unsigned int)hr));
            return hr;
        }

        // Note: IMFVideoDisplayControl will be obtained later when playback starts,
        // after the topology has been resolved by the session.

        // Get clock
        hr = mediaSession->GetClock(&clock);

        // Get audio volume control
        hr = MFGetService(mediaSession, MR_STREAM_VOLUME_SERVICE, IID_PPV_ARGS(&audioVolume));

        bLoaded = true;
        currentState = Stopped;

        // Connect to TimeBar
        connectTimeBarSignals();

        // Seek to current TimeBar time and pause
        TimeBar* timeBar = TimeBar::instance();
        seekMedia(timeBar->time());
        pauseMedia();

        MessageOut::master()->putln(
            formatR(_("Media loaded: {}"), fpath.filename().string()));

    } catch(const std::exception& ex){
        MessageOut::master()->putErrorln(
            formatR(_("Exception while loading media: {}"), ex.what()));
        unload();
        return E_FAIL;
    }

    return S_OK;
}

HRESULT MFMediaViewImpl::unload()
{
    disconnectTimeBarSignals();

    if(audioVolume){
        audioVolume->Release();
        audioVolume = nullptr;
    }

    if(clock){
        clock->Release();
        clock = nullptr;
    }

    if(videoDisplayControl){
        videoDisplayControl->Release();
        videoDisplayControl = nullptr;
    }

    if(videoRendererActivate){
        videoRendererActivate->ShutdownObject();
        videoRendererActivate->Release();
        videoRendererActivate = nullptr;
    }

    if(mediaSession){
        mediaSession->Shutdown();
        mediaSession->Release();
        mediaSession = nullptr;
    }

    if(mediaSource){
        mediaSource->Shutdown();
        mediaSource->Release();
        mediaSource = nullptr;
    }

    bLoaded = false;
    currentState = Stopped;

    return S_OK;
}

HRESULT MFMediaViewImpl::createTopology(IMFMediaSource* source, IMFTopology** ppTopology)
{
    HRESULT hr = S_OK;

    IMFTopology* topology = nullptr;
    IMFPresentationDescriptor* presDesc = nullptr;

    // Create empty topology
    hr = MFCreateTopology(&topology);
    if(FAILED(hr)){
        return hr;
    }

    // Get presentation descriptor from source
    hr = source->CreatePresentationDescriptor(&presDesc);
    if(FAILED(hr)){
        topology->Release();
        return hr;
    }

    // Get number of streams
    DWORD streamCount = 0;
    hr = presDesc->GetStreamDescriptorCount(&streamCount);
    if(FAILED(hr)){
        presDesc->Release();
        topology->Release();
        return hr;
    }

    // Add branch for each stream
    for(DWORD i = 0; i < streamCount; i++){
        hr = addBranchToTopology(topology, source, presDesc, i);
        if(FAILED(hr)){
            MessageOut::master()->putWarningln(
                formatR(_("Warning: Failed to add stream {} to topology"), i));
        }
    }

    presDesc->Release();

    *ppTopology = topology;
    return S_OK;
}

HRESULT MFMediaViewImpl::addBranchToTopology(IMFTopology* topology, IMFMediaSource* source,
                                             IMFPresentationDescriptor* presDesc, DWORD streamIndex)
{
    HRESULT hr = S_OK;

    IMFStreamDescriptor* streamDesc = nullptr;
    IMFTopologyNode* sourceNode = nullptr;
    IMFTopologyNode* outputNode = nullptr;
    IMFActivate* activate = nullptr;

    BOOL selected = FALSE;
    hr = presDesc->GetStreamDescriptorByIndex(streamIndex, &selected, &streamDesc);
    if(FAILED(hr)){
        return hr;
    }

    if(!selected){
        streamDesc->Release();
        return S_OK;  // Stream not selected, skip
    }

    // Create source node
    hr = createSourceStreamNode(source, presDesc, streamDesc, &sourceNode);
    if(FAILED(hr)){
        streamDesc->Release();
        return hr;
    }

    // Create output node (renderer)
    hr = createOutputNode(streamDesc, &outputNode, &activate);
    if(FAILED(hr)){
        sourceNode->Release();
        streamDesc->Release();
        return hr;
    }

    // Save video renderer activate for later use
    if(activate && !videoRendererActivate){
        videoRendererActivate = activate;
        videoRendererActivate->AddRef();
    }
    if(activate){
        activate->Release();
    }

    // Add nodes to topology
    hr = topology->AddNode(sourceNode);
    if(SUCCEEDED(hr)){
        hr = topology->AddNode(outputNode);
    }

    // Connect source node to output node
    if(SUCCEEDED(hr)){
        hr = sourceNode->ConnectOutput(0, outputNode, 0);
    }

    streamDesc->Release();
    sourceNode->Release();
    outputNode->Release();

    return hr;
}

HRESULT MFMediaViewImpl::createSourceStreamNode(IMFMediaSource* source, IMFPresentationDescriptor* presDesc,
                                                IMFStreamDescriptor* streamDesc, IMFTopologyNode** ppNode)
{
    HRESULT hr = S_OK;

    IMFTopologyNode* node = nullptr;

    // Create source node
    hr = MFCreateTopologyNode(MF_TOPOLOGY_SOURCESTREAM_NODE, &node);
    if(FAILED(hr)){
        return hr;
    }

    // Set attributes
    hr = node->SetUnknown(MF_TOPONODE_SOURCE, source);
    if(SUCCEEDED(hr)){
        hr = node->SetUnknown(MF_TOPONODE_PRESENTATION_DESCRIPTOR, presDesc);
    }
    if(SUCCEEDED(hr)){
        hr = node->SetUnknown(MF_TOPONODE_STREAM_DESCRIPTOR, streamDesc);
    }

    if(FAILED(hr)){
        node->Release();
        return hr;
    }

    *ppNode = node;
    return S_OK;
}

HRESULT MFMediaViewImpl::createOutputNode(IMFStreamDescriptor* streamDesc, IMFTopologyNode** ppNode, IMFActivate** ppActivate)
{
    HRESULT hr = S_OK;

    IMFTopologyNode* node = nullptr;
    IMFMediaTypeHandler* handler = nullptr;
    GUID majorType;

    *ppActivate = nullptr;

    // Get major type of stream
    hr = streamDesc->GetMediaTypeHandler(&handler);
    if(FAILED(hr)){
        return hr;
    }

    hr = handler->GetMajorType(&majorType);
    handler->Release();
    if(FAILED(hr)){
        return hr;
    }

    // Create output node
    hr = MFCreateTopologyNode(MF_TOPOLOGY_OUTPUT_NODE, &node);
    if(FAILED(hr)){
        return hr;
    }

    // Create appropriate renderer based on media type
    if(majorType == MFMediaType_Video){
        // Create EVR (Enhanced Video Renderer)
        IMFActivate* activate = nullptr;
        hr = MFCreateVideoRendererActivate(hwnd, &activate);
        if(SUCCEEDED(hr)){
            hr = node->SetObject(activate);
            if(SUCCEEDED(hr)){
                *ppActivate = activate;  // Return activate to caller
                (*ppActivate)->AddRef();

                // Immediately activate the EVR to get the video display control
                IUnknown* evr = nullptr;
                HRESULT activateHr = activate->ActivateObject(IID_IUnknown, (void**)&evr);

                if(SUCCEEDED(activateHr) && evr){
                    // Get the video display control from the EVR
                    IMFGetService* getService = nullptr;
                    HRESULT serviceHr = evr->QueryInterface(IID_PPV_ARGS(&getService));

                    if(SUCCEEDED(serviceHr)){
                        serviceHr = getService->GetService(MR_VIDEO_RENDER_SERVICE, IID_PPV_ARGS(&videoDisplayControl));
                        getService->Release();
                    }
                    evr->Release();
                }
            }
            activate->Release();
        }
    } else if(majorType == MFMediaType_Audio){
        // Create SAR (Streaming Audio Renderer)
        IMFActivate* activate = nullptr;
        hr = MFCreateAudioRendererActivate(&activate);
        if(SUCCEEDED(hr)){
            hr = node->SetObject(activate);
            activate->Release();
        }
    } else {
        // Unsupported media type
        node->Release();
        return E_FAIL;
    }

    if(FAILED(hr)){
        node->Release();
        return hr;
    }

    *ppNode = node;
    return S_OK;
}

// Helper to ensure video display control is obtained
HRESULT MFMediaViewImpl::ensureVideoDisplayControl()
{
    if(videoDisplayControl || !videoRendererActivate){
        return S_OK;  // Already have it, or no video
    }

    // Activate the EVR and get the video display control directly from the activate object
    HRESULT hr = videoRendererActivate->ActivateObject(IID_IMFVideoDisplayControl, (void**)&videoDisplayControl);

    if(SUCCEEDED(hr) && videoDisplayControl){
        // Set video window
        if(hwnd){
            videoDisplayControl->SetVideoWindow(hwnd);
            adjustVideoWindow();
        }
    }

    return hr;
}

// Phase 3: Playback control
HRESULT MFMediaViewImpl::playMedia()
{
    if(!mediaSession || !bLoaded){
        return E_FAIL;
    }

    // Ensure video display control is obtained before playing
    ensureVideoDisplayControl();

    PROPVARIANT varStart;
    PropVariantInit(&varStart);
    varStart.vt = VT_EMPTY;  // Start from current position

    HRESULT hr = mediaSession->Start(&GUID_NULL, &varStart);
    PropVariantClear(&varStart);

    if(SUCCEEDED(hr)){
        currentState = Running;
    }

    return hr;
}

HRESULT MFMediaViewImpl::pauseMedia()
{
    if(!mediaSession || !bLoaded){
        return E_FAIL;
    }

    // Ensure video display control is obtained before pausing
    ensureVideoDisplayControl();

    HRESULT hr = mediaSession->Pause();

    if(SUCCEEDED(hr)){
        currentState = Paused;
    }

    return hr;
}

HRESULT MFMediaViewImpl::stopMedia()
{
    if(!mediaSession || !bLoaded){
        return E_FAIL;
    }

    HRESULT hr = mediaSession->Stop();

    if(SUCCEEDED(hr)){
        currentState = Stopped;
    }

    return hr;
}

HRESULT MFMediaViewImpl::seekMedia(double time)
{
    if(!mediaSession || !bLoaded || !bCanSeek){
        return E_FAIL;
    }

    // Add offset time from MediaItem
    double adjustedTime = time;
    if(currentMediaItem){
        adjustedTime += currentMediaItem->offsetTime();
    }

    // Convert to 100-nanosecond units
    const LONGLONG ONE_SECOND = 10000000;
    LONGLONG position = (LONGLONG)(adjustedTime * ONE_SECOND);

    // Seek using Start method with specified position
    PROPVARIANT varStart;
    PropVariantInit(&varStart);
    varStart.vt = VT_I8;
    varStart.hVal.QuadPart = position;

    HRESULT hr = mediaSession->Start(&GUID_NULL, &varStart);
    PropVariantClear(&varStart);

    return hr;
}

// Phase 4: TimeBar integration
void MFMediaViewImpl::connectTimeBarSignals()
{
    if(timeBarConnections.empty()){
        TimeBar* timeBar = TimeBar::instance();
        timeBarConnections.add(
            timeBar->sigPlaybackInitialized().connect(
                [this](double time){ return onPlaybackInitialized(time); }));
        timeBarConnections.add(
            timeBar->sigPlaybackStarted().connect(
                [this](double time){ onPlaybackStarted(time); }));
        timeBarConnections.add(
            timeBar->sigPlaybackStoppedEx().connect(
                [this](double time, bool isStoppedManually){
                    return onPlaybackStopped(time, isStoppedManually); }));
        timeBarConnections.add(
            timeBar->sigTimeChanged().connect(
                [this](double time){ return onTimeChanged(time); }));
    }
}

void MFMediaViewImpl::disconnectTimeBarSignals()
{
    timeBarConnections.disconnect();
}

bool MFMediaViewImpl::onPlaybackInitialized(double time)
{
    if(!bLoaded){
        return false;
    }

    // Ensure video display control is obtained
    ensureVideoDisplayControl();

    // Seek to the start time and pause
    seekMedia(time);
    pauseMedia();

    return true;
}

void MFMediaViewImpl::onPlaybackStarted(double time)
{
    if(bLoaded){
        playMedia();
    }
}

double MFMediaViewImpl::onPlaybackStopped(double time, bool isStoppedManually)
{
    double lastValidTime = time;

    if(bLoaded && mediaSource && currentMediaItem){
        // Get presentation descriptor to query duration
        IMFPresentationDescriptor* presDesc = nullptr;
        HRESULT hr = mediaSource->CreatePresentationDescriptor(&presDesc);

        if(SUCCEEDED(hr)){
            UINT64 duration100ns = 0;
            hr = presDesc->GetUINT64(MF_PD_DURATION, &duration100ns);

            if(SUCCEEDED(hr)){
                // Convert 100-nanosecond units to seconds
                double mediaDuration = (double)duration100ns / 10000000.0;
                // Subtract offset time to get the media end time in TimeBar coordinate
                double mediaEndTime = mediaDuration - currentMediaItem->offsetTime();
                lastValidTime = mediaEndTime;
            }

            presDesc->Release();
        }
    }

    if(bLoaded){
        pauseMedia();
    }

    return lastValidTime;
}

bool MFMediaViewImpl::onTimeChanged(double time)
{
    if(!bLoaded){
        return false;
    }

    if(currentState == Running){
        // Continue playback, don't seek
        return true;
    } else {
        // Seek to new time while paused
        seekMedia(time);
        return false;
    }
}

// Phase 5: Video window management with high DPI support
HRESULT MFMediaViewImpl::adjustVideoWindow()
{
    if(!videoDisplayControl || !bLoaded){
        return S_OK;
    }

    HRESULT hr = S_OK;

    // Get device pixel ratio for high DPI support
    qreal dpr = 1.0;
    if(auto window = self->windowHandle()){
        dpr = window->devicePixelRatio();
    }

    // Get logical dimensions from Qt widget
    int logicalWidth = self->width();
    int logicalHeight = self->height();

    // Convert to physical pixels for Media Foundation
    LONG screenWidth = qRound(logicalWidth * dpr);
    LONG screenHeight = qRound(logicalHeight * dpr);

    // Don't update if widget doesn't have valid size yet
    if(screenWidth <= 0 || screenHeight <= 0){
        return S_OK;
    }

    SIZE videoSize, arSize;
    hr = videoDisplayControl->GetNativeVideoSize(&videoSize, &arSize);

    if(hr == E_NOINTERFACE || FAILED(hr)){
        // No video (audio-only file) or not ready yet
        return S_OK;
    }

    LONG videoWidth = videoSize.cx;
    LONG videoHeight = videoSize.cy;

    if(videoWidth <= 0 || videoHeight <= 0){
        return S_OK;
    }

    // videoWidth and videoHeight are already in physical pixels
    RECT rcDest;

    if(orgSizeCheck && orgSizeCheck->isChecked()){
        // Keep original size - center video if it fits, otherwise position at (0,0)
        if(videoWidth <= screenWidth && videoHeight <= screenHeight){
            LONG x = (screenWidth - videoWidth) / 2;
            LONG y = (screenHeight - videoHeight) / 2;
            rcDest = {x, y, x + videoWidth, y + videoHeight};
        } else {
            rcDest = {0, 0, videoWidth, videoHeight};
        }
    } else if(aspectRatioCheck && aspectRatioCheck->isChecked()){
        // Scale to fit while maintaining aspect ratio
        double r1 = (double)screenWidth / videoWidth;
        double r2 = (double)screenHeight / videoHeight;
        double scale = (std::min)(r1, r2);

        LONG scaledWidth = (LONG)(videoWidth * scale);
        LONG scaledHeight = (LONG)(videoHeight * scale);

        LONG x = (screenWidth - scaledWidth) / 2;
        LONG y = (screenHeight - scaledHeight) / 2;

        rcDest = {x, y, x + scaledWidth, y + scaledHeight};
    } else {
        // Stretch to fill entire window
        rcDest = {0, 0, screenWidth, screenHeight};
    }

    // Set border color to black to fill non-video areas
    videoDisplayControl->SetBorderColor(RGB(0, 0, 0));

    // Set new position
    hr = videoDisplayControl->SetVideoPosition(nullptr, &rcDest);

    // Repaint video at new position
    if(SUCCEEDED(hr)){
        hr = videoDisplayControl->RepaintVideo();
    }

    return hr;
}

HRESULT MFMediaViewImpl::handleSessionEvent()
{
    return S_OK;
}
