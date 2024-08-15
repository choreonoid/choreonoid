#include "CaptureBar.h"
#include "ExtensionManager.h"
#include "MessageView.h"
#include "MainWindow.h"
#include "AppConfig.h"
#include "SceneView.h"
#include "SceneWidget.h"
#include "FileDialog.h"
#include "QtEventUtil.h"
#include <QApplication>
#include <QPainter>
#include "gettext.h"

using namespace std;
using namespace cnoid;


void CaptureBar::initialize(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        ext->addToolBar(instance());
        initialized = true;
    }
}


CaptureBar* CaptureBar::instance()
{
    static CaptureBar* captureBar = new CaptureBar;
    return captureBar;
}


CaptureBar::CaptureBar()
    : ToolBar(N_("CaptureBar"))
{
    lastCaptureWidget = nullptr;
    
    captureButton = addButton(":/Base/icon/scenecapture.svg");
    captureButton->setToolTip(_("Capture the image of a view or toolbar"));
    captureButton->sigClicked().connect( [this](){ grabMouse(); });
    captureButton->installEventFilter(this);
}


CaptureBar::~CaptureBar()
{
    if(config){
        if(config->empty()){
            AppConfig::archive()->remove("CaptureBar");
        } else {
            config->setFlowStyle(false);
        }
    }
}


void CaptureBar::mouseMoveEvent(QMouseEvent*)
{
    
}


void CaptureBar::mousePressEvent(QMouseEvent* event)
{
    if(event->button() == Qt::LeftButton){
        releaseMouse();
        QWidget* widget = QApplication::widgetAt(getGlobalPosition(event));
        while(widget){
            if(ToolBar* bar = dynamic_cast<ToolBar*>(widget)){
                captureToolbar(bar);
                break;
            } else if(View* view = dynamic_cast<View*>(widget)){
                captureView(view);
                break;
            }
            widget = widget->parentWidget();
        }
    }
}


bool CaptureBar::eventFilter(QObject* obj, QEvent* event)
{
    if(obj == captureButton && event->type() == QEvent::MouseButtonPress){
        auto mouseEvent = static_cast<QMouseEvent*>(event);
        if(mouseEvent->button() == Qt::RightButton){
            onCaptureButtonRightClicked(mouseEvent);
            return true;
        }
    }
    // standard event processing
    return QObject::eventFilter(obj, event);
}


Mapping* CaptureBar::getConfig()
{
    if(!config){
        config = AppConfig::archive()->openMapping("CaptureBar");
    }
    return config;
}


void CaptureBar::onCaptureButtonRightClicked(QMouseEvent* event)
{
    menuManager.setNewPopupMenu(captureButton);
    auto check = menuManager.addCheckItem(_("Include Tab"));
    if(!isTabInclusionMode){
        isTabInclusionMode = getConfig()->get("include_tab", false);
    }
    check->setChecked(isTabInclusionMode && *isTabInclusionMode);
    check->sigToggled().connect([this](bool on){ setTabInclusionMode(on); });
    menuManager.popupMenu()->popup(getGlobalPosition(event));
}


void CaptureBar::setTabInclusionMode(bool on)
{
    isTabInclusionMode = on;
    getConfig()->write("include_tab", on);
}


void CaptureBar::captureToolbar(ToolBar* bar)
{
    save(bar, [this, bar](const QString& filename){ return saveWidgetImage(bar, filename); });
}


void CaptureBar::captureView(View* view)
{
    auto owner = view->parentWidget();

    if(isTabInclusionMode && *isTabInclusionMode){
        for(int i=0; i < 2; ++i){
            auto tab = dynamic_cast<QTabWidget*>(owner);
            if(tab){
                save(view,
                     [this, tab, view](const QString& filename){
                         return saveTabViewImage(tab, view, filename);
                     });
                return;
            }
            owner = owner->parentWidget();
            if(!owner){
                break;
            }
        }
    }

    save(view, [this, view](const QString& filename){ return saveWidgetImage(view, filename); });
}


bool CaptureBar::saveWidgetImage(QWidget* widget, const QString& filename)
{
    if(auto sceneView = dynamic_cast<SceneView*>(widget)){
        return sceneView->sceneWidget()->saveImage(filename.toStdString());
    } else {
        QPixmap pixmap(widget->size());
        widget->render(&pixmap);
        return pixmap.save(filename);
    }
}


bool CaptureBar::saveTabViewImage(QTabWidget* tab, View* view, const QString& filename)
{
    const int n = tab->count();
    vector<QWidget*> widgets(n);
    for(int i=0; i < n; ++i){
        widgets[i] = tab->widget(i);
    }
    tab->clear();
    tab->addTab(view, view->windowTitle());

    MessageView::instance()->flush();
        
    QPixmap pixmap(tab->size());
    tab->render(&pixmap);
        
    auto sceneView = dynamic_cast<SceneView*>(view);
    if(sceneView){
        QPainter painter(&pixmap);
        QImage image = sceneView->sceneWidget()->getImage();
        QPoint pos = sceneView->mapTo(tab, QPoint(0, 0));
        painter.drawImage(pos, image);
    }

    bool saved = pixmap.save(filename);
        
    tab->clear();
    for(int i=0; i < n; ++i){
        tab->addTab(widgets[i], widgets[i]->windowTitle());
    }
    tab->setCurrentWidget(view);

    return saved;
}


void CaptureBar::save(QWidget* widget, std::function<bool(const QString& filename)> saveImage)
{
    const QString name(widget->windowTitle());
    QString filename;
        
    FileDialog dialog(MainWindow::instance());
    dialog.setWindowTitle(QString(_("Save the image of %1")).arg(name));
    dialog.setFileMode(QFileDialog::AnyFile);
    dialog.setAcceptMode(QFileDialog::AcceptSave);
    dialog.setViewMode(QFileDialog::List);
    dialog.setLabelText(QFileDialog::Accept, _("Save"));
    dialog.setLabelText(QFileDialog::Reject, _("Cancel"));
        
    QStringList filters;
    filters << _("Any files (*)");
    dialog.setNameFilters(filters);

    dialog.updatePresetDirectories(true);

    dialog.setDirectory(getConfig()->get("directory", QDir::currentPath().toStdString()));
        
    if(widget != lastCaptureWidget){
        lastCaptureFile = QString("%1.png").arg(name);
        lastCaptureWidget = widget;
    }
    dialog.selectFile(lastCaptureFile);
        
    if(dialog.exec()){
        getConfig()->writePath("directory", dialog.directory().absolutePath().toStdString());
            
        filename = dialog.selectedFiles().front();
            
        if(!filename.isEmpty()){
            if(saveImage(filename)){
                lastCaptureFile = filename;
                /*
                MessageView::instance()->putln(
                    QString(_("The image of %1 has successfully been saved into \"%2\".")).arg(name).arg(filename));
                */
            } else {
                MessageView::instance()->putln(
                    QString(_("The image of %1 cannot be saved into \"%2\".")).arg(name).arg(filename));
            }
        }
    }
}
