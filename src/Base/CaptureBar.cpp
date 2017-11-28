/**
   @author Shin'ichiro Nakaoka
*/

#include "CaptureBar.h"
#include "ExtensionManager.h"
#include "MenuManager.h"
#include "MessageView.h"
#include "MainWindow.h"
#include "AppConfig.h"
#include "SceneView.h"
#include "SceneWidget.h"
#include <QApplication>
#include <QMouseEvent>
#include <QPainter>
#include <QFileDialog>
#include <QTabWidget>
#include <functional>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using namespace std::placeholders;

namespace {

Action* includeTabCheck;
QString lastCaptureFile;
QWidget* lastCaptureWidget;


void onIncludeTabToggled(bool on)
{
    AppConfig::archive()->openMapping("CaptureBar")->write("includeTab", on);
}


bool saveWidgetImage(QWidget* widget, const QString& filename)
{
    SceneView* sceneView = dynamic_cast<SceneView*>(widget);
    if(sceneView){
        return sceneView->sceneWidget()->saveImage(filename.toStdString());
    } else {
        QPixmap pixmap(widget->size());
        widget->render(&pixmap);
        return pixmap.save(filename);
    }
}


bool saveTabViewImage(QTabWidget* tab, View* view, const QString& filename)
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
        
    SceneView* sceneView = dynamic_cast<SceneView*>(view);
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


void save(QWidget* widget, std::function<bool(const QString& filename)> saveImage)
{
    const QString name(widget->windowTitle());
    QString filename;
        
    QFileDialog dialog(MainWindow::instance());
    dialog.setWindowTitle(QString(_("Save the image of %1")).arg(name));
    dialog.setFileMode(QFileDialog::AnyFile);
    dialog.setAcceptMode(QFileDialog::AcceptSave);
    dialog.setViewMode(QFileDialog::List);
    dialog.setLabelText(QFileDialog::Accept, _("Save"));
    dialog.setLabelText(QFileDialog::Reject, _("Cancel"));
        
    QStringList filters;
    filters << _("Any files (*)");
    dialog.setNameFilters(filters);

    MappingPtr config = AppConfig::archive()->openMapping("CaptureBar");
        
    dialog.setDirectory(config->get("directory", QDir::currentPath().toStdString()).c_str());
        
    if(widget != lastCaptureWidget){
        lastCaptureFile = QString("%1.png").arg(name);
        lastCaptureWidget = widget;
    }
    dialog.selectFile(lastCaptureFile);
        
    if(dialog.exec()){
        config->writePath("directory", dialog.directory().absolutePath().toStdString());
            
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


void captureView(View* view)
{
    QWidget* owner = view->parentWidget();
        
    if(includeTabCheck->isChecked()){
        for(int i=0; i < 2; ++i){
            QTabWidget* tab = dynamic_cast<QTabWidget*>(owner);
            if(tab){
                save(view, std::bind(saveTabViewImage, tab, view, _1));
                return;
            }
            owner = owner->parentWidget();
            if(!owner){
                break;
            }
        }
    }

    save(view, std::bind(saveWidgetImage, view, _1));
}


void captureToolbar(ToolBar* bar)
{
    save(bar, std::bind(saveWidgetImage, bar, _1));
}
}


void CaptureBar::initialize(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        ext->addToolBar(instance());

        MenuManager& mm = ext->menuManager();
        MappingPtr config = AppConfig::archive()->openMapping("CaptureBar");
        mm.setPath("/Options").setPath(N_("Capture Bar"));
        
        includeTabCheck = mm.addCheckItem(_("Include Tab"));
        includeTabCheck->setChecked(config->get("includeTab", false));
        includeTabCheck->sigToggled().connect(onIncludeTabToggled);
        
        lastCaptureWidget = 0;
        
        initialized = true;
    }
}


CaptureBar* CaptureBar::instance()
{
    static CaptureBar* captureBar = new CaptureBar();
    return captureBar;
}


CaptureBar::CaptureBar()
    : ToolBar(N_("CaptureBar"))
{
    addButton(QIcon(":/Base/icons/scenecapture.png"), _("Capture the image of a view or toolbar"))
        ->sigClicked().connect(
            std::bind(static_cast<void(CaptureBar::*)()>(&CaptureBar::grabMouse), this));
}


CaptureBar::~CaptureBar()
{

}


void CaptureBar::mouseMoveEvent(QMouseEvent*)
{
    

}


void CaptureBar::mousePressEvent(QMouseEvent* event)
{
    if(event->button() == Qt::LeftButton){
        releaseMouse();
        QWidget* widget = QApplication::widgetAt(event->globalPos());
        while(widget){
            if(View* view = dynamic_cast<View*>(widget)){
                captureView(view);
                break;
            } else if(ToolBar* bar = dynamic_cast<ToolBar*>(widget)){
                captureToolbar(bar);
                break;
            }
            widget = widget->parentWidget();
        }
    }
}
