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
#include <filesystem>
#include <QBoxLayout>
#include <QLabel>
#include <QResizeEvent>
#include <QWindow>
#include <QPainter>
#include <QTimer>
#include <windows.h>
#include <mfapi.h>
#include <mfidl.h>
#include <mfreadwrite.h>
#include <mferror.h>
#include <evr.h>
#include <Shlwapi.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
namespace filesystem = std::filesystem;

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

    // Source Reader for frame extraction (used for seeking without audio)
    IMFSourceReader* sourceReader;

    // Frame buffer for display
    BYTE* frameBuffer;
    LONG frameBufferSize;
    LONG frameWidth;
    LONG frameHeight;
    LONG frameStride;

    // State flags
    bool bMapped;
    bool bLoaded;
    bool bCanSeek;
    bool bIgnoreTimeChange;  // Ignore time change signals during initial load
    bool bWasMinimized;      // Track if window was minimized (needs reload on restore)

    // Seek debouncing
    QTimer* seekDebounceTimer;
    double pendingSeekTime;

    // Window handles
    HWND hwnd;           // Parent window (Qt widget)
    HWND hwndVideo;      // Child window for video rendering
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
    void onWindowShown();
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
    void createVideoWindow();
    void destroyVideoWindow();

    // Playback control
    HRESULT playMedia(double startTime = -1.0);  // Start playback at specified time (-1 = current position)
    HRESULT pauseMedia();
    HRESULT stopMedia();
    void seekMedia(double time);      // Debounced seek (schedules actual seek)

    // Frame extraction (silent seek)
    HRESULT createSourceReader(const std::wstring& filePath);
    void releaseSourceReader();
    HRESULT displayFrameAtTime(double time);
    void renderFrameToWindow();

    // TimeBar callbacks
    void connectTimeBarSignals();
    void disconnectTimeBarSignals();
    bool onPlaybackInitialized(double time);
    void onPlaybackStarted(double time);
    double onPlaybackStopped(double time, bool isStoppedManually);
    bool onTimeChanged(double time);
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
        // Window ID changed - this happens when view is rearranged, docked/undocked, etc.
        // Mark for reload since EVR/MediaSession needs to be reinitialized
        if(impl->bLoaded){
            impl->bWasMinimized = true;
        }
        impl->onWindowIdChanged();
        return true;
    }
    if(event->type() == QEvent::Hide){
        // Window is being hidden (e.g., main window minimized)
        // Mark for reload when restored
        if(impl->bLoaded){
            impl->bWasMinimized = true;
        }
    }
    if(event->type() == QEvent::Show){
        impl->onWindowShown();
    }
    if(event->type() == QEvent::WindowActivate){
        impl->onWindowShown();
    }
    return QWidget::event(event);
}

void MFMediaView::resizeEvent(QResizeEvent* event)
{
    impl->adjustVideoWindow();
}

void MFMediaView::paintEvent(QPaintEvent* event)
{
    // Fill background with black using Win32 GDI
    // This is more compatible with EVR's direct window rendering than QPainter
    HWND hwnd = (HWND)winId();
    if(hwnd){
        HDC hdc = GetDC(hwnd);
        if(hdc){
            RECT rect;
            GetClientRect(hwnd, &rect);
            HBRUSH brush = CreateSolidBrush(RGB(0, 0, 0));
            FillRect(hdc, &rect, brush);
            DeleteObject(brush);
            ReleaseDC(hwnd, hdc);
        }
    }
}

QPaintEngine* MFMediaView::paintEngine() const
{
    return nullptr;
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
    } else if(impl->bWasMinimized && impl->currentMediaItem){
        impl->bWasMinimized = false;

        if(impl->currentState == MFMediaViewImpl::Running){
            // Was playing during minimize - just re-establish video window
            // (MediaSession was not unloaded, audio continued)
            impl->destroyVideoWindow();
            impl->hwnd = (HWND)winId();
            impl->createVideoWindow();
            if(impl->videoDisplayControl && impl->hwndVideo){
                impl->videoDisplayControl->SetVideoWindow(impl->hwndVideo);
                impl->adjustVideoWindow();
                impl->videoDisplayControl->RepaintVideo();
            }
        } else {
            // Was paused/stopped - need to reload and restore position
            impl->load();
            // Restore TimeBar position
            QTimer::singleShot(150, [this](){
                if(impl->bLoaded){
                    impl->displayFrameAtTime(TimeBar::instance()->time());
                }
            });
        }
    }
}

void MFMediaView::onDeactivated()
{
    // If playing, don't unload - let audio continue during minimize
    // We'll re-establish video on reactivate
    if(impl->bLoaded && impl->currentState != MFMediaViewImpl::Running){
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

    sourceReader = nullptr;
    frameBuffer = nullptr;
    frameBufferSize = 0;
    frameWidth = 0;
    frameHeight = 0;
    frameStride = 0;

    bMapped = false;
    bLoaded = false;
    bCanSeek = true;
    bIgnoreTimeChange = false;
    bWasMinimized = false;

    // Initialize seek debounce timer
    seekDebounceTimer = new QTimer(self);
    seekDebounceTimer->setSingleShot(true);
    seekDebounceTimer->setInterval(50);  // 50ms debounce
    QObject::connect(seekDebounceTimer, &QTimer::timeout, [this](){
        if(bLoaded && bCanSeek){
            // Use SourceReader for silent frame display when not playing
            displayFrameAtTime(pendingSeekTime);
        }
    });
    pendingSeekTime = 0.0;

    hwnd = nullptr;
    hwndVideo = nullptr;
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
    releaseSourceReader();
    if(frameBuffer){
        delete[] frameBuffer;
        frameBuffer = nullptr;
    }
    destroyVideoWindow();
}

void MFMediaViewImpl::onWindowIdChanged()
{
    HWND newHwnd = (HWND)self->winId();

    // If parent HWND changed and we have a video window, destroy it
    // (it was created with the old parent)
    if(newHwnd != hwnd && hwndVideo){
        destroyVideoWindow();
    }

    hwnd = newHwnd;

    // If bWasMinimized is set (by event handler), trigger reload via onWindowShown
    // This handles view rearrangement, docking changes, etc.
    if(bWasMinimized && bLoaded && bMapped){
        onWindowShown();
        return;
    }

    // Recreate video window if needed and set video display
    if(bLoaded && videoDisplayControl && hwnd){
        if(!hwndVideo){
            createVideoWindow();
        }
        if(hwndVideo){
            videoDisplayControl->SetVideoWindow(hwndVideo);
            adjustVideoWindow();
        }
    }
}

void MFMediaViewImpl::onWindowShown()
{
    if(!bMapped){
        return;
    }

    // If window was minimized and is now being restored, do a full reload
    // This is necessary because EVR/MediaSession enters a bad state after minimize
    if(bWasMinimized && bLoaded && currentMediaItem){
        bWasMinimized = false;
        unload();
        load();
        return;
    }

    if(!bLoaded){
        return;
    }

    // Update parent HWND (it may have changed)
    HWND currentHwnd = (HWND)self->winId();
    if(currentHwnd != hwnd){
        hwnd = currentHwnd;
    }

    // Check if parent window is valid
    if(!hwnd || !IsWindow(hwnd)){
        return;
    }

    // Check if child window was destroyed or became invalid
    if(hwndVideo && !IsWindow(hwndVideo)){
        hwndVideo = nullptr;
        orgWinProc = nullptr;
    }

    // Recreate child window if needed
    if(!hwndVideo && hwnd){
        createVideoWindow();
    }

    // Re-establish EVR video window if needed
    if(videoDisplayControl && hwndVideo && IsWindow(hwndVideo)){
        videoDisplayControl->SetVideoWindow(hwndVideo);
    }

    adjustVideoWindow();

    // Redisplay the current frame
    if(currentState != Running){
        // First, try to render cached frame if available
        if(frameBuffer && frameWidth > 0 && frameHeight > 0){
            renderFrameToWindow();
        } else {
            // No cached frame, extract from SourceReader
            QTimer::singleShot(50, [this](){
                if(bLoaded && bMapped && hwndVideo && IsWindow(hwndVideo)){
                    TimeBar* tb = TimeBar::instance();
                    displayFrameAtTime(tb->time());
                }
            });
        }
    } else {
        // If playing, repaint video
        if(videoDisplayControl){
            videoDisplayControl->RepaintVideo();
        }
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

        // Create SourceReader for silent frame extraction
        createSourceReader(wFilePath);

        // Ignore time change signals during initial load.
        // This prevents duplicate seeks from TimeBar restoration during project loading.
        bIgnoreTimeChange = true;

        // Connect to TimeBar
        connectTimeBarSignals();

        // Adjust video window size and position
        adjustVideoWindow();

        // Display initial frame at current TimeBar position (using SourceReader - no audio)
        // Use QTimer::singleShot to defer until the window is fully created and shown
        QTimer::singleShot(100, [this](){
            if(bLoaded){
                TimeBar* timeBar = TimeBar::instance();
                displayFrameAtTime(timeBar->time());
            }
        });

        // Clear the ignore flag after event loop processes pending events
        QTimer::singleShot(0, [this](){ bIgnoreTimeChange = false; });

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

    // Release SourceReader
    releaseSourceReader();

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

    // Destroy video child window
    destroyVideoWindow();

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
        // Create child window for video rendering before creating EVR
        createVideoWindow();
        HWND targetHwnd = hwndVideo ? hwndVideo : hwnd;

        // Create EVR (Enhanced Video Renderer) with child window
        IMFActivate* activate = nullptr;
        hr = MFCreateVideoRendererActivate(targetHwnd, &activate);
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
        // Set video window (prefer child window if available)
        HWND targetHwnd = hwndVideo ? hwndVideo : hwnd;
        if(targetHwnd){
            videoDisplayControl->SetVideoWindow(targetHwnd);
        }
    }

    return hr;
}

// Phase 3: Playback control
HRESULT MFMediaViewImpl::playMedia(double startTime)
{
    if(!mediaSession || !bLoaded){
        return E_FAIL;
    }

    // Ensure video display control is obtained before playing
    ensureVideoDisplayControl();

    PROPVARIANT varStart;
    PropVariantInit(&varStart);

    if(startTime >= 0){
        // Start playback at specified position (ensures precise synchronization with TimeBar)
        double adjustedTime = startTime;
        if(currentMediaItem){
            adjustedTime += currentMediaItem->offsetTime();
        }
        const LONGLONG ONE_SECOND = 10000000;
        varStart.vt = VT_I8;
        varStart.hVal.QuadPart = (LONGLONG)(adjustedTime * ONE_SECOND);
    } else {
        varStart.vt = VT_EMPTY;  // Start from current position
    }

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

// Debounced seek - only the last seek request within the debounce interval is executed
void MFMediaViewImpl::seekMedia(double time)
{
    pendingSeekTime = time;
    seekDebounceTimer->start();  // Restart timer on each call
}

// SourceReader-based frame extraction (silent seeking without audio)
HRESULT MFMediaViewImpl::createSourceReader(const std::wstring& filePath)
{
    HRESULT hr = S_OK;

    // Release any existing SourceReader
    releaseSourceReader();

    // Create attributes for SourceReader
    IMFAttributes* attributes = nullptr;
    hr = MFCreateAttributes(&attributes, 1);
    if(FAILED(hr)){
        return hr;
    }

    // Request video processing for easy RGB conversion
    hr = attributes->SetUINT32(MF_SOURCE_READER_ENABLE_VIDEO_PROCESSING, TRUE);

    // Create SourceReader from file
    hr = MFCreateSourceReaderFromURL(filePath.c_str(), attributes, &sourceReader);
    attributes->Release();

    if(FAILED(hr)){
        return hr;
    }

    // Select only video stream (disable audio to avoid any audio processing)
    sourceReader->SetStreamSelection(MF_SOURCE_READER_ALL_STREAMS, FALSE);
    sourceReader->SetStreamSelection(MF_SOURCE_READER_FIRST_VIDEO_STREAM, TRUE);

    // Set output format to RGB32 for easy rendering
    IMFMediaType* outputType = nullptr;
    hr = MFCreateMediaType(&outputType);
    if(SUCCEEDED(hr)){
        hr = outputType->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
        if(SUCCEEDED(hr)){
            hr = outputType->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_RGB32);
        }
        if(SUCCEEDED(hr)){
            hr = sourceReader->SetCurrentMediaType(
                MF_SOURCE_READER_FIRST_VIDEO_STREAM, nullptr, outputType);
        }
        outputType->Release();
    }

    // Get the actual output format to determine frame dimensions
    IMFMediaType* actualType = nullptr;
    hr = sourceReader->GetCurrentMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM, &actualType);
    if(SUCCEEDED(hr)){
        UINT32 width = 0, height = 0;
        hr = MFGetAttributeSize(actualType, MF_MT_FRAME_SIZE, &width, &height);
        if(SUCCEEDED(hr)){
            frameWidth = width;
            frameHeight = height;

            // Get stride (may be negative for bottom-up DIB)
            LONG stride = 0;
            hr = actualType->GetUINT32(MF_MT_DEFAULT_STRIDE, (UINT32*)&stride);
            if(FAILED(hr)){
                // Calculate stride if not provided (RGB32 = 4 bytes per pixel)
                stride = width * 4;
            }
            frameStride = stride;

            // Allocate frame buffer
            LONG bufferSize = abs(stride) * height;
            if(bufferSize > frameBufferSize){
                if(frameBuffer){
                    delete[] frameBuffer;
                }
                frameBuffer = new BYTE[bufferSize];
                frameBufferSize = bufferSize;
            }
        }
        actualType->Release();
    }

    return hr;
}

void MFMediaViewImpl::releaseSourceReader()
{
    if(sourceReader){
        sourceReader->Release();
        sourceReader = nullptr;
    }
}

HRESULT MFMediaViewImpl::displayFrameAtTime(double time)
{
    if(!sourceReader || !frameBuffer || frameWidth <= 0 || frameHeight <= 0){
        return E_FAIL;
    }

    // Check if we have a valid window to render to
    HWND targetHwnd = hwndVideo ? hwndVideo : hwnd;
    if(!targetHwnd || !IsWindow(targetHwnd)){
        return E_FAIL;
    }

    // Add offset time from MediaItem
    double adjustedTime = time;
    if(currentMediaItem){
        adjustedTime += currentMediaItem->offsetTime();
    }

    // Convert to 100-nanosecond units
    const LONGLONG ONE_SECOND = 10000000;
    LONGLONG targetPosition = (LONGLONG)(adjustedTime * ONE_SECOND);
    if(targetPosition < 0) targetPosition = 0;

    // Seek to the specified position (seeks to nearest keyframe before target)
    PROPVARIANT varPosition;
    PropVariantInit(&varPosition);
    varPosition.vt = VT_I8;
    varPosition.hVal.QuadPart = targetPosition;
    HRESULT hr = sourceReader->SetCurrentPosition(GUID_NULL, varPosition);
    PropVariantClear(&varPosition);

    if(FAILED(hr)){
        return hr;
    }

    // Read samples until we reach or pass the target position
    // This is necessary because SetCurrentPosition only seeks to keyframes
    IMFSample* lastSample = nullptr;
    const int MAX_FRAMES_TO_DECODE = 300;  // Safety limit

    for(int i = 0; i < MAX_FRAMES_TO_DECODE; i++){
        DWORD streamIndex = 0;
        DWORD flags = 0;
        LONGLONG timestamp = 0;
        IMFSample* sample = nullptr;

        hr = sourceReader->ReadSample(
            MF_SOURCE_READER_FIRST_VIDEO_STREAM,
            0,
            &streamIndex,
            &flags,
            &timestamp,
            &sample);

        if(FAILED(hr) || (flags & MF_SOURCE_READERF_ENDOFSTREAM)){
            // End of stream or error - use last sample if we have one
            if(sample){
                sample->Release();
            }
            break;
        }

        if(!sample){
            continue;
        }

        // Release previous sample and keep this one
        if(lastSample){
            lastSample->Release();
        }
        lastSample = sample;

        // Check if we've reached or passed the target position
        if(timestamp >= targetPosition){
            break;
        }
    }

    // Render the last sample we got
    if(lastSample){
        IMFMediaBuffer* buffer = nullptr;
        hr = lastSample->ConvertToContiguousBuffer(&buffer);
        if(SUCCEEDED(hr)){
            BYTE* data = nullptr;
            DWORD maxLen = 0, curLen = 0;
            hr = buffer->Lock(&data, &maxLen, &curLen);
            if(SUCCEEDED(hr)){
                // Copy frame data to our buffer
                LONG copySize = (std::min)((LONG)curLen, frameBufferSize);
                memcpy(frameBuffer, data, copySize);
                buffer->Unlock();

                // Render the frame to the window
                renderFrameToWindow();
            }
            buffer->Release();
        }
        lastSample->Release();
    }

    return hr;
}

void MFMediaViewImpl::renderFrameToWindow()
{
    if(!frameBuffer || frameWidth <= 0 || frameHeight <= 0){
        return;
    }

    // Use child window if available, otherwise parent window
    HWND targetHwnd = hwndVideo ? hwndVideo : hwnd;
    if(!targetHwnd){
        return;
    }

    HDC hdc = GetDC(targetHwnd);
    if(!hdc){
        return;
    }

    // Get window size
    RECT clientRect;
    GetClientRect(targetHwnd, &clientRect);
    int windowWidth = clientRect.right - clientRect.left;
    int windowHeight = clientRect.bottom - clientRect.top;

    if(windowWidth <= 0 || windowHeight <= 0){
        ReleaseDC(targetHwnd, hdc);
        return;
    }

    // Calculate destination rectangle based on display options
    int destX, destY, destW, destH;

    if(orgSizeCheck && orgSizeCheck->isChecked()){
        // Keep original size - center video
        destX = (windowWidth - frameWidth) / 2;
        destY = (windowHeight - frameHeight) / 2;
        destW = frameWidth;
        destH = frameHeight;
    } else if(aspectRatioCheck && aspectRatioCheck->isChecked()){
        // Scale to fit while maintaining aspect ratio
        double r1 = (double)windowWidth / frameWidth;
        double r2 = (double)windowHeight / frameHeight;
        double scale = (std::min)(r1, r2);

        destW = (int)(frameWidth * scale);
        destH = (int)(frameHeight * scale);
        destX = (windowWidth - destW) / 2;
        destY = (windowHeight - destH) / 2;
    } else {
        // Stretch to fill entire window
        destX = 0;
        destY = 0;
        destW = windowWidth;
        destH = windowHeight;
    }

    // Fill background with black first
    HBRUSH blackBrush = (HBRUSH)GetStockObject(BLACK_BRUSH);
    FillRect(hdc, &clientRect, blackBrush);

    // Create BITMAPINFO structure for RGB32 data
    BITMAPINFO bmi = {};
    bmi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    bmi.bmiHeader.biWidth = frameWidth;
    bmi.bmiHeader.biHeight = -frameHeight;  // Negative for top-down DIB
    bmi.bmiHeader.biPlanes = 1;
    bmi.bmiHeader.biBitCount = 32;
    bmi.bmiHeader.biCompression = BI_RGB;

    // Use StretchDIBits to render the frame
    SetStretchBltMode(hdc, HALFTONE);
    StretchDIBits(
        hdc,
        destX, destY, destW, destH,           // Destination rectangle
        0, 0, frameWidth, frameHeight,        // Source rectangle
        frameBuffer,
        &bmi,
        DIB_RGB_COLORS,
        SRCCOPY);

    ReleaseDC(targetHwnd, hdc);
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

    // Cancel any pending debounced seek to avoid interference
    seekDebounceTimer->stop();

    // Display frame at start position (playMedia will handle actual seeking)
    displayFrameAtTime(time);

    return true;
}

void MFMediaViewImpl::onPlaybackStarted(double time)
{
    if(bLoaded){
        // Start playback at specified time for precise synchronization with TimeBar
        playMedia(time);
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

    // Ignore time change during initial load to avoid audio glitch
    if(bIgnoreTimeChange){
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

// Video child window procedure
static LRESULT CALLBACK videoChildWindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    MFMediaViewImpl* impl = (MFMediaViewImpl*)GetWindowLongPtr(hwnd, GWLP_USERDATA);

    switch(uMsg){
    case WM_PAINT:
        if(impl){
            if(impl->currentState == MFMediaViewImpl::Running && impl->videoDisplayControl){
                // Let EVR repaint the video during playback
                impl->videoDisplayControl->RepaintVideo();
            } else if(impl->frameBuffer && impl->frameWidth > 0 && impl->frameHeight > 0){
                // Not playing - render cached frame using GDI
                impl->renderFrameToWindow();
            }
        }
        // Validate the window to prevent continuous WM_PAINT messages
        ValidateRect(hwnd, nullptr);
        return 0;

    case WM_ERASEBKGND:
        // Return non-zero to indicate we handled background erasing
        return 1;
    }

    return DefWindowProcW(hwnd, uMsg, wParam, lParam);
}

// Video child window management
void MFMediaViewImpl::createVideoWindow()
{
    if(hwndVideo || !hwnd){
        return;
    }

    // Register window class for video child window
    static bool classRegistered = false;
    static const wchar_t* className = L"CnoidMFVideoWindow";

    if(!classRegistered){
        WNDCLASSEXW wc = {};
        wc.cbSize = sizeof(WNDCLASSEXW);
        wc.style = CS_OWNDC;
        wc.lpfnWndProc = videoChildWindowProc;
        wc.hInstance = GetModuleHandle(nullptr);
        wc.hbrBackground = (HBRUSH)GetStockObject(BLACK_BRUSH);
        wc.lpszClassName = className;
        RegisterClassExW(&wc);
        classRegistered = true;
    }

    // Create child window for video rendering
    hwndVideo = CreateWindowExW(
        0,
        className,
        L"",
        WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS,
        0, 0, 1, 1,  // Initial size, will be adjusted later
        hwnd,
        nullptr,
        GetModuleHandle(nullptr),
        nullptr);

    // Store pointer to impl for use in window procedure
    if(hwndVideo){
        SetWindowLongPtr(hwndVideo, GWLP_USERDATA, (LONG_PTR)this);
    }
}

void MFMediaViewImpl::destroyVideoWindow()
{
    if(hwndVideo){
        DestroyWindow(hwndVideo);
        hwndVideo = nullptr;
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

    // Child window should already exist (created in createOutputNode)
    // If not, create it now as fallback
    if(!hwndVideo){
        createVideoWindow();
        if(hwndVideo){
            videoDisplayControl->SetVideoWindow(hwndVideo);
        } else {
            return S_OK;
        }
    }

    SIZE videoSize, arSize;
    hr = videoDisplayControl->GetNativeVideoSize(&videoSize, &arSize);

    LONG videoWidth = 0;
    LONG videoHeight = 0;

    if(SUCCEEDED(hr) && hr != E_NOINTERFACE){
        videoWidth = videoSize.cx;
        videoHeight = videoSize.cy;
    }

    // If video size is not yet available, use screen size as default
    // This ensures the child window is properly sized even before video is ready
    if(videoWidth <= 0 || videoHeight <= 0){
        videoWidth = screenWidth;
        videoHeight = screenHeight;
    }

    // Calculate child window position and size
    LONG childX, childY, childW, childH;

    if(orgSizeCheck && orgSizeCheck->isChecked()){
        // Keep original size - center video
        childX = (screenWidth - videoWidth) / 2;
        childY = (screenHeight - videoHeight) / 2;
        childW = videoWidth;
        childH = videoHeight;
    } else if(aspectRatioCheck && aspectRatioCheck->isChecked()){
        // Scale to fit while maintaining aspect ratio
        double r1 = (double)screenWidth / videoWidth;
        double r2 = (double)screenHeight / videoHeight;
        double scale = (std::min)(r1, r2);

        childW = (LONG)(videoWidth * scale);
        childH = (LONG)(videoHeight * scale);
        childX = (screenWidth - childW) / 2;
        childY = (screenHeight - childH) / 2;
    } else {
        // Stretch to fill entire window
        childX = 0;
        childY = 0;
        childW = screenWidth;
        childH = screenHeight;
    }

    // Move and resize child window
    // This automatically clips video to child window bounds
    SetWindowPos(hwndVideo, nullptr, childX, childY, childW, childH,
                 SWP_NOZORDER | SWP_NOACTIVATE);

    // Set aspect ratio mode based on user preference
    // Note: This is only needed for stretch mode since EVR defaults to preserving aspect ratio
    if(!orgSizeCheck || !orgSizeCheck->isChecked()){
        if(aspectRatioCheck && aspectRatioCheck->isChecked()){
            videoDisplayControl->SetAspectRatioMode(MFVideoARMode_PreservePicture);
        } else {
            videoDisplayControl->SetAspectRatioMode(MFVideoARMode_None);
        }
    }

    // Set video to fill entire child window
    RECT rcDest = {0, 0, childW, childH};
    hr = videoDisplayControl->SetVideoPosition(nullptr, &rcDest);

    // Repaint video at new position
    if(SUCCEEDED(hr)){
        hr = videoDisplayControl->RepaintVideo();
    }

    return hr;
}
