#include "BodyLibraryView.h"
#include "BodyLibrarySelectionDialog.h"
#include "WorldItem.h"
#include "BodyItem.h"
#include <cnoid/ViewManager>
#include <cnoid/MenuManager>
#include <cnoid/TreeWidget>
#include <cnoid/RootItem>
#include <cnoid/SceneView>
#include <cnoid/MessageView>
#include <cnoid/MessageOut>
#include <cnoid/LazyCaller>
#include <cnoid/YAMLReader>
#include <cnoid/YAMLWriter>
#include <cnoid/ExecutablePath>
#include <cnoid/FilePathVariableProcessor>
#include <cnoid/FileUtil>
#include <cnoid/UTF8>
#include <cnoid/ZipArchiver>
#include <cnoid/Dialog>
#include <cnoid/FileDialog>
#include <cnoid/LineEdit>
#include <cnoid/PushButton>
#include <QBoxLayout>
#include <QLabel>
#include <QDialogButtonBox>
#include <QFileIconProvider>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QMimeData>
#include <QDrag>
#include <QPainter>
#include <QMessageBox>
#include <cnoid/stdx/filesystem>
#include <fmt/format.h>
#include <unordered_set>
#include <map>
#include <fstream>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;
namespace fs = stdx::filesystem;

namespace {

constexpr int LibraryItemType = QTreeWidgetItem::UserType;

class LibraryItem : public QTreeWidgetItem
{
public:
    string name;
    string file;
    string thumbnailFile;
    bool isGroup;
    MappingPtr extraInfo;

    LibraryItem();
    LibraryItem(const LibraryItem& other);
    virtual QTreeWidgetItem* clone() const override;
};

class ExTreeWidget : public TreeWidget
{
public:
    BodyLibraryView::Impl* viewImpl;
    
    ExTreeWidget(BodyLibraryView::Impl* viewImpl);
    LibraryItem* getLibraryItem(const QModelIndex& index);
    virtual QMimeData* mimeData(const QList<QTreeWidgetItem *> items) const override;
    virtual void startDrag(Qt::DropActions supportedActions) override;
    virtual void dropEvent(QDropEvent* event) override;
};

class ExMimeData : public QMimeData
{
public:
    vector<LibraryItem*> libraryItems;
};

class GroupNameDialog : public Dialog
{
public:
    LineEdit* nameEdit;
    PushButton* applyButton;

    GroupNameDialog(QWidget* parent);
};

}

namespace cnoid {

class BodyLibraryView::Impl
{
public:
    BodyLibraryView* self;
    ExTreeWidget treeWidget;
    Connection rowsInsertedConnection;
    string libraryDirectory;
    fs::path libraryDirPath;
    string indexFile;
    bool needToLoadIndexFile;
    bool needToSaveIndexFile;
    LazyCaller saveIndexFileLater;
    FilePathVariableProcessorPtr filePathVariableProcessor;
    fs::path orgBaseDirPath;
    fs::path orgProjectDirPath;
    QIcon noImageIcon;
    QPixmap noImagePixmap;
    int imageSize;
    MenuManager contextMenuManager;
    GroupNameDialog* groupNameDialog;
    // Used for buit-in drag&drop function inside the view
    map<LibraryItem*, fs::path> libraryItemToOrgDirPathMap;
    BodyLibrarySelectionDialog* selectionDialog;
    vector<fs::path> refDirPaths;

    static QTreeWidgetItem* getTreeWidgetItem(Element& element);
    static LibraryItem* getLibraryItem(Element& element);
    
    Impl(BodyLibraryView* self);
    ~Impl();
    bool setLibraryDirectory(const std::string& directory, bool ensureDirectoryAndIndexFile);
    LibraryItem* createGroupItem(const std::string& name);
    LibraryItem* createBodyItem(
        const std::string& name, const std::string& file, const std::string& thumbnailFile);
    bool setItemImage(LibraryItem* item);
    void selectAndScrollToItem(LibraryItem* item);
    bool checkNameDuplication(QTreeWidgetItem* parentItem, const std::string& name);
    fs::path getInternalItemDirPath(QTreeWidgetItem* item);
    fs::path getInternalItemDirPath(QTreeWidgetItem* parentItem, const std::string& itemName);
    bool ensureDirectory(const fs::path& dirPath, bool doCleanExistingDir = false);
    bool checkIfInternalFilePath(const fs::path& path);
    bool checkIfExistingInternalFilePath(const fs::path& path);
    bool copyFile(const fs::path& srcPath, const fs::path& destPath);
    bool renameLibraryItem(LibraryItem* item, const string& newName);
    bool moveLibraryItem(LibraryItem* item, QTreeWidgetItem* destGroupItem);
    bool relocateItemFiles(LibraryItem* item, const fs::path& orgDirPath, QTreeWidgetItem* destGroupItem);
    void relocateItemFileInformationRecursively(LibraryItem* item, fs::path basePath);
    void removeLibraryItem(LibraryItem* item);
    GroupNameDialog* getOrCreateGroupNameDialog();
    bool createGroupWithDialog(ElementPtr parentGroup);
    bool createGroupWithDialog(QTreeWidgetItem* parentItem);
    bool registerBodyWithDialog(
        BodyItem* bodyItem, Mapping* extraInfo, QTreeWidgetItem* groupItem, QTreeWidgetItem* position, bool doStoreFiles);
    string exportBodyFileToLibraryDirectory(BodyItem* bodyItem, QTreeWidgetItem* groupItem);
    bool storeItemFilesToLibraryDirectory(
        LibraryItem* libraryItem, BodyItemPtr bodyItem, QTreeWidgetItem* groupItem, MessageOut* mout);
    bool storeItemFilesToLibraryDirectoryRecursively(QTreeWidgetItem* item, MessageOut* mout);
    bool setBodyThumbnailWithDialog(LibraryItem* item);
    void clearBodyThumbnail(LibraryItem* item);
    bool renameLibraryItemWithDialog(LibraryItem* item);
    void forEachTopItems(const QList<QTreeWidgetItem*>& items, function<bool(LibraryItem*)> callback);
    void removeSelectedItemsWithDialog();
    void showDialogToMoveSelectedItems();
    void moveSelectedItemsWithDialog(QTreeWidgetItem* destGroup);
    void onTreeWidgetRowsInserted(const QModelIndex& parent, int start, int end);
    void updateItemImagesRecursively(LibraryItem* item);
    void onLibraryItemDoubleClicked(QTreeWidgetItem* item);
    void onContextMenuRequested(const QPoint& pos);
    bool onSceneViewDragEnterEvent(SceneView* sceneView, QDragEnterEvent* event);
    bool onSceneViewDropEvent(SceneView* sceneView, QDropEvent* event);
    void initializeFilePathVariableProcessor();
    void finalizeFilePathVariableProcessor();
    bool loadIndexFile();
    void readLibraryElements(Listing* elements, QTreeWidgetItem* parentItem);
    LibraryItem* readLibraryItem(Mapping* info);
    bool saveIndexFileIfUpdated();
    bool saveIndexFile();
    bool saveIndexFile(QTreeWidgetItem* rootItem);
    void writeLibraryElements(YAMLWriter& writer, QTreeWidgetItem* groupItem);
    bool importLibrary(const std::string& filename, MessageOut* mout);
    bool exportLibrary(const std::string& filename, MessageOut* mout);
    bool exportLibraryToDirectory(const std::string& directory, MessageOut* mout);
};

}


LibraryItem::LibraryItem()
    : QTreeWidgetItem(LibraryItemType)
{
    setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled | Qt::ItemIsDragEnabled);
}


LibraryItem::LibraryItem(const LibraryItem& other)
    : QTreeWidgetItem(LibraryItemType),
      name(other.name),
      file(other.file),
      thumbnailFile(other.file),
      isGroup(other.isGroup)
{
    (*this) = other;

    if(other.extraInfo){
        extraInfo = other.extraInfo->cloneMapping();
    }
}


QTreeWidgetItem* LibraryItem::clone() const
{
    auto itemClone = new LibraryItem(*this);
    int n = childCount();
    for(int i=0; i < n; ++i){
        itemClone->addChild(child(i)->clone());
    }
    return itemClone;
}


ExTreeWidget::ExTreeWidget(BodyLibraryView::Impl* viewImpl)
    : viewImpl(viewImpl)
{

}


LibraryItem* ExTreeWidget::getLibraryItem(const QModelIndex& index)
{
    if(index.isValid()){
        return dynamic_cast<LibraryItem*>(itemFromIndex(index));
    }
    return nullptr;
}


QMimeData* ExTreeWidget::mimeData(const QList<QTreeWidgetItem*> items) const
{
    auto mimeData = new ExMimeData;

    auto orgMimeData = TreeWidget::mimeData(items);
    mimeData->setData(
        "application/x-qabstractitemmodeldatalist",
        orgMimeData->data("application/x-qabstractitemmodeldatalist"));
    delete orgMimeData;

    for(auto& item : items){
        if(auto libraryItem = dynamic_cast<LibraryItem*>(item)){
            mimeData->libraryItems.push_back(libraryItem);
        }
    }
        
    return mimeData;
}


void ExTreeWidget::startDrag(Qt::DropActions supportedActions)
{
    auto drag = new QDrag(this);
    drag->setMimeData(model()->mimeData(selectedIndexes()));
    QPixmap pixmap(viewport()->visibleRegion().boundingRect().size());
    pixmap.fill(Qt::transparent);
    QPainter painter(&pixmap);
    for(QModelIndex index: selectedIndexes()){
        painter.drawPixmap(visualRect(index), viewport()->grab(visualRect(index)));
    }
    drag->setPixmap(pixmap);
    drag->setHotSpot(viewport()->mapFromGlobal(QCursor::pos()));
    drag->exec(supportedActions, Qt::MoveAction);
}


void ExTreeWidget::dropEvent(QDropEvent* event)
{
    if(auto data = dynamic_cast<const ExMimeData*>(event->mimeData())){
        QTreeWidgetItem* itemAtDrop = itemAt(event->pos());
        if(auto libraryItem = dynamic_cast<LibraryItem*>(itemAtDrop)){
            if(!libraryItem->isGroup){
                itemAtDrop = libraryItem->parent();
            }
        }
        if(!itemAtDrop){
            itemAtDrop = invisibleRootItem();
        }
        bool duplicated = false;
        string duplicatedName;
        for(auto& item : data->libraryItems){
            if(item->parent() != itemAtDrop){ // Drop to another group?
                if(viewImpl->checkNameDuplication(itemAtDrop, item->name)){
                    duplicatedName = item->name;
                    duplicated = true;
                    break;
                }
            }
        }
        if(duplicated){
            showErrorDialog(
                format(_("Drop was canceled because item \"{0}\" is duplicated in the drop position."),
                       duplicatedName));
            return; // Ignore the drop
        }

        viewImpl->libraryItemToOrgDirPathMap.clear();
        for(auto& item : data->libraryItems){
            viewImpl->libraryItemToOrgDirPathMap[item] = viewImpl->getInternalItemDirPath(item);
        }
        
    }
    TreeWidget::dropEvent(event);
}


GroupNameDialog::GroupNameDialog(QWidget* parent)
{
    auto vbox = new QVBoxLayout;
    setLayout(vbox);

    auto hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Group Name:")));
    nameEdit = new LineEdit;
    hbox->addWidget(nameEdit);
    vbox->addLayout(hbox);

    auto buttonBox = new QDialogButtonBox(this);
    applyButton = new PushButton;
    applyButton->setDefault(true);
    applyButton->sigClicked().connect([this]{ accept(); });
    buttonBox->addButton(applyButton, QDialogButtonBox::AcceptRole);

    auto cancelButton = new PushButton(_("&Cancel"));
    cancelButton->sigClicked().connect([this]{ reject(); });
    buttonBox->addButton(cancelButton, QDialogButtonBox::RejectRole);

    vbox->addWidget(buttonBox);

    nameEdit->sigTextChanged().connect(
        [this](const QString& text){
            applyButton->setEnabled(!text.isEmpty());
        });
    applyButton->setEnabled(false);
}


QTreeWidgetItem* BodyLibraryView::Impl::getTreeWidgetItem(Element& element)
{
    return reinterpret_cast<QTreeWidgetItem*>(element.handle);
}


LibraryItem* BodyLibraryView::Impl::getLibraryItem(Element& element)
{
    return dynamic_cast<LibraryItem*>(reinterpret_cast<QTreeWidgetItem*>(element.handle));
}


BodyLibraryView::Element::Element(intptr_t handle)
    : handle(handle)
{

}


bool BodyLibraryView::Element::isGroup() const
{
    auto item = BodyLibraryView::Impl::getTreeWidgetItem(*const_cast<Element*>(this));
    auto libraryItem = dynamic_cast<LibraryItem*>(item);
    if(item->type() == LibraryItemType){
        return static_cast<LibraryItem*>(item)->isGroup;
    }
    return true;
}


BodyLibraryView::ElementPtr BodyLibraryView::Element::create(intptr_t handle)
{
    struct ElementX : Element {
        ElementX(intptr_t handle) : Element(handle) { }
    };
    return std::make_shared<ElementX>(handle);
}


bool BodyLibraryView::Element::isRoot() const
{
    auto item = reinterpret_cast<QTreeWidgetItem*>(handle);
    return item->type() != LibraryItemType;
}


bool BodyLibraryView::Element::isBody() const
{
    return !isGroup();
}


string BodyLibraryView::Element::name() const
{
    auto item = reinterpret_cast<QTreeWidgetItem*>(handle);
    return item->text(0).toStdString();
}


std::vector<BodyLibraryView::ElementPtr> BodyLibraryView::Element::elements() const
{
    std::vector<ElementPtr> elements;
    auto item = reinterpret_cast<QTreeWidgetItem*>(handle);
    int n = item->childCount();
    for(int i=0; i < n; ++i){
        elements.push_back(create(reinterpret_cast<intptr_t>(item->child(i))));
    }
    return elements;
}


void BodyLibraryView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<BodyLibraryView>(N_("BodyLibraryView"), N_("Body Library"));
}


BodyLibraryView* BodyLibraryView::instance()
{
    static BodyLibraryView* instance_ = ViewManager::getOrCreateView<BodyLibraryView>();
    return instance_;
}


BodyLibraryView::BodyLibraryView()
{
    impl = new Impl(this);
}


BodyLibraryView::Impl::Impl(BodyLibraryView* self)
    : self(self),
      treeWidget(this),
      saveIndexFileLater([this]{ saveIndexFile(); }, LazyCaller::NormalPriority)
{
    self->setDefaultLayoutArea(BottomLeftArea);
    self->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

    treeWidget.setHeaderHidden(true);
    treeWidget.setFrameShape(QFrame::NoFrame);
    treeWidget.setColumnCount(1);
    treeWidget.setSelectionMode(QAbstractItemView::ExtendedSelection);
    treeWidget.setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    treeWidget.setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
    treeWidget.setDragDropMode(QAbstractItemView::InternalMove);
    treeWidget.setContextMenuPolicy(Qt::CustomContextMenu);

    rowsInsertedConnection =
        treeWidget.sigRowsInserted().connect(
            [this](const QModelIndex& parent, int start, int end){
                onTreeWidgetRowsInserted(parent, start, end);
            });
    
    treeWidget.sigItemDoubleClicked().connect(
        [this](QTreeWidgetItem* item, int /* column */){
            onLibraryItemDoubleClicked(item);
        });

    treeWidget.sigCustomContextMenuRequested().connect(
        [this](const QPoint& pos){
            onContextMenuRequested(pos);
        });

    auto vbox = new QVBoxLayout;
    vbox->setSpacing(0);
    vbox->addWidget(&treeWidget);
    self->setLayout(vbox);
    
    needToLoadIndexFile = false;;
    needToSaveIndexFile = false;
    imageSize = 120;

    groupNameDialog = nullptr;
    selectionDialog = nullptr;
}


BodyLibraryView::~BodyLibraryView()
{
    delete impl;
}


BodyLibraryView::Impl::~Impl()
{
    if(groupNameDialog){
        delete groupNameDialog;
    }
}


bool BodyLibraryView::setLibraryDirectory(const std::string& directory, bool ensureDirectoryAndIndexFile)
{
    return impl->setLibraryDirectory(directory, ensureDirectoryAndIndexFile);
}


bool BodyLibraryView::Impl::setLibraryDirectory(const std::string& directory, bool ensureDirectoryAndIndexFile)
{
    bool failed = false;
    
    if(directory != libraryDirectory){

        saveIndexFileIfUpdated();

        fs::path dirPath(fromUTF8(directory));
        fs::path filePath = dirPath / "index.yaml";

        if(ensureDirectoryAndIndexFile){
            try {
                bool directoryExists = false;
                auto fileStatus = fs::status(filePath);
                bool fileExists = fs::is_regular_file(fileStatus);
                if(!fileExists){
                    if(!fs::exists(fileStatus)){
                        auto dirStatus = fs::status(dirPath);
                        bool directoryExists = is_directory(dirStatus);
                        if(!directoryExists){
                            if(!fs::exists(dirStatus)){
                                directoryExists = fs::create_directories(dirPath);
                            }
                        }
                        if(directoryExists){
                            // Create an empty index file
                            std::ofstream file(filePath.string());
                            file << "elements: [ ]\n";
                            fileExists = true;
                        }
                    }
                }
                if(!fileExists){
                    failed = true;
                }
            }
            catch(const fs::filesystem_error&) {
                failed = true;
            }
        }

        if(!failed){
            libraryDirectory = directory;
            libraryDirPath = dirPath;
            indexFile = toUTF8(filePath.string());

            needToLoadIndexFile = true;
            if(self->isActive()){
                loadIndexFile();
            }
        }
    }
    
    return !failed;
}


const std::string& BodyLibraryView::libraryDirectory() const
{
    return impl->libraryDirectory;
}


void BodyLibraryView::addReferenceDirectory(const std::string& directory)
{
    impl->refDirPaths.emplace_back(fromUTF8(directory));
}


void BodyLibraryView::onActivated()
{
    if(impl->needToLoadIndexFile){
        impl->loadIndexFile();
    }
    
    auto sceneView = SceneView::instance();
    sceneView->setAcceptDrops(true);
    sceneView->installEventFilter(this);
}


void BodyLibraryView::onDeactivated()
{
    SceneView::instance()->removeEventFilter(this);
}


void BodyLibraryView::onAttachedMenuRequest(MenuManager& menuManager)
{

}


LibraryItem* BodyLibraryView::Impl::createGroupItem(const std::string& name)
{
    LibraryItem* item = new LibraryItem;
    item->name = name;
    item->isGroup = true;
    item->setText(0, name.c_str());
    item->setFlags(item->flags() | Qt::ItemIsDropEnabled);
    static QFileIconProvider provider;
    item->setIcon(0, provider.icon(QFileIconProvider::Folder));
    return item;
}


LibraryItem* BodyLibraryView::Impl::createBodyItem
(const std::string& name, const std::string& file, const std::string& thumbnailFile)
{
    LibraryItem* item = new LibraryItem;
    item->name = name;
    item->file = file;
    item->thumbnailFile = thumbnailFile;
    item->isGroup = false;
    return item;
}


bool BodyLibraryView::Impl::setItemImage(LibraryItem* item)
{
    if(item->isGroup){
        return false;
    }
    
    auto panel = new QWidget;
    auto hbox = new QHBoxLayout(panel);
    auto vbox = new QVBoxLayout;
    hbox->addLayout(vbox);
    hbox->addStretch();
    auto imageLabel = new QLabel;
    imageLabel->setAlignment(Qt::AlignCenter);
    vbox->addWidget(imageLabel);
    auto nameLabel = new QLabel(item->name.c_str());
    nameLabel->setAlignment(Qt::AlignCenter);
    vbox->addWidget(nameLabel);
        
    bool hasThumbnail = false;
    if(!item->thumbnailFile.empty()){
        QIcon icon(item->thumbnailFile.c_str());
        auto pixmap = icon.pixmap(QSize(imageSize, imageSize));
        if(!pixmap.isNull()){
            imageLabel->setPixmap(pixmap);
            hasThumbnail = true;
        }
    }
    if(!hasThumbnail){
        imageLabel->setPixmap(noImagePixmap);
    }
    treeWidget.setItemWidget(item, 0, panel);

    return hasThumbnail;
}


void BodyLibraryView::Impl::selectAndScrollToItem(LibraryItem* item)
{
    if(self->isActive()){
        treeWidget.expandItem(item);
        treeWidget.scrollToItem(item);
        treeWidget.selectionModel()->clear();
        item->setSelected(true);
    }
}


BodyLibraryView::ElementPtr BodyLibraryView::getRootElement()
{
    if(impl->needToLoadIndexFile){
        impl->loadIndexFile();
    }
    return Element::create(reinterpret_cast<intptr_t>(impl->treeWidget.invisibleRootItem()));
}


bool BodyLibraryView::hasLibraryElements() const
{
    return impl->treeWidget.topLevelItemCount() > 0;
}


bool BodyLibraryView::Impl::checkNameDuplication(QTreeWidgetItem* parentItem, const std::string& name)
{
    bool duplicated = false;
    int n = parentItem->childCount();
    for(int i=0; i < n; ++i){
        auto libraryItem = static_cast<LibraryItem*>(parentItem->child(i));
        if(libraryItem->name == name){
            return true;
        }
    }
    return false;
}


fs::path BodyLibraryView::Impl::getInternalItemDirPath(QTreeWidgetItem* item)
{
    vector<string> dirs;
    LibraryItem* upperItem = dynamic_cast<LibraryItem*>(item);
    while(upperItem){
        if(!upperItem->name.empty()){
            dirs.push_back(upperItem->name);
        }
        upperItem = dynamic_cast<LibraryItem*>(upperItem->parent());
    }
    fs::path dirPath = libraryDirPath;
    for(auto it = dirs.rbegin(); it != dirs.rend(); ++it){
        dirPath /= fs::path(fromUTF8(*it));
    }
    return dirPath;
}


fs::path BodyLibraryView::Impl::getInternalItemDirPath(QTreeWidgetItem* parentItem, const std::string& itemName)
{
    auto dirPath = getInternalItemDirPath(parentItem);
    dirPath /= itemName;
    return dirPath;
}


bool BodyLibraryView::Impl::ensureDirectory(const fs::path& dirPath, bool doCleanExistingDir)
{
    stdx::error_code ec;
    bool dirExists = fs::exists(dirPath, ec);
    if(ec){
        return false;
    }
    if(dirExists){
        if(!doCleanExistingDir && fs::is_directory(dirPath)){
            return true;
        }
        if(!fs::remove_all(dirPath, ec)){
            return false;
        }
    }
    if(!fs::create_directories(dirPath, ec)){
        return false;
    }
    return true;
}


bool BodyLibraryView::Impl::checkIfInternalFilePath(const fs::path& path)
{
    return checkIfSubFilePath(path, libraryDirPath);
}


bool BodyLibraryView::Impl::checkIfExistingInternalFilePath(const fs::path& path)
{
    return checkIfInternalFilePath(path) && fs::exists(path);
}


bool BodyLibraryView::Impl::copyFile(const fs::path& srcPath, const fs::path& destPath)
{
    if(!ensureDirectory(destPath.parent_path())){
        return false;
    }

    stdx::error_code ec;

#if __cplusplus > 201402L
    fs::copy_file(srcPath, destPath, fs::copy_options::overwrite_existing, ec);
#else
    fs::copy_file(srcPath, destPath, fs::copy_option::overwrite_if_exists, ec);
#endif

    return !ec;
}


bool BodyLibraryView::Impl::renameLibraryItem(LibraryItem* item, const string& newName)
{
    bool failed = false;
    fs::path orgDirPath = getInternalItemDirPath(item);
    fs::path newNamePath(fromUTF8(newName));
    fs::path newDirPath = orgDirPath.parent_path() / newNamePath;

    stdx::error_code ec;
    if(!fs::is_directory(orgDirPath, ec)){
        if(ec){
            failed = true;
        }
    } else {
        fs::rename(orgDirPath, newDirPath, ec);
        if(ec){
            failed = true;
        }
    }

    if(!failed){
        item->name = newName;
        relocateItemFileInformationRecursively(item, newDirPath);
        updateItemImagesRecursively(item);
        saveIndexFile();
    }

    return !failed;
}


bool BodyLibraryView::Impl::moveLibraryItem(LibraryItem* item, QTreeWidgetItem* destGroupItem)
{
    fs::path orgDirPath = getInternalItemDirPath(item);

    if(!relocateItemFiles(item, orgDirPath, destGroupItem)){
        return false;
    }

    auto block = rowsInsertedConnection.scopedBlock();
    if(auto parent = item->parent()){
        parent->removeChild(item);
    } else {
        treeWidget.takeTopLevelItem(treeWidget.indexOfTopLevelItem(item));
    }
    destGroupItem->addChild(item);

    updateItemImagesRecursively(item);
    selectAndScrollToItem(item);
    saveIndexFileLater();

    return true;
}


bool BodyLibraryView::Impl::relocateItemFiles
(LibraryItem* item, const fs::path& orgDirPath, QTreeWidgetItem* destGroupItem)
{
    bool failed = false;
    fs::path destDirPath = getInternalItemDirPath(destGroupItem);

    if(destDirPath != orgDirPath){
        if(checkIfExistingInternalFilePath(orgDirPath)){
            if(!ensureDirectory(destDirPath)){
                failed = true;
            } else {
                destDirPath /= orgDirPath.filename();
                stdx::error_code ec;
                fs::rename(orgDirPath, destDirPath, ec);
                if(ec){
                    failed = true;
                }
            }
        }
        if(!failed){
            relocateItemFileInformationRecursively(item, destDirPath);
        }
    }

    return !failed;
}


void BodyLibraryView::Impl::relocateItemFileInformationRecursively(LibraryItem* item, fs::path basePath)
{
    if(!item->isGroup){
        fs::path filePath(fromUTF8(item->file));
        if(checkIfInternalFilePath(filePath)){
            filePath = basePath / filePath.filename();
            item->file = toUTF8(filePath.generic_string());
        }
        fs::path thumbnailFilePath(fromUTF8(item->thumbnailFile));
        if(checkIfInternalFilePath(thumbnailFilePath)){
            thumbnailFilePath = basePath / thumbnailFilePath.filename();
            item->thumbnailFile = toUTF8(thumbnailFilePath.generic_string());
        }
    } else {
        int n = item->childCount();
        for(int i=0; i < n; ++i){
            auto childItem = static_cast<LibraryItem*>(item->child(i));
            relocateItemFileInformationRecursively(childItem, basePath / fromUTF8(childItem->name));
        }
    }
}


void BodyLibraryView::Impl::removeLibraryItem(LibraryItem* item)
{
    stdx::error_code ec;
    auto path = getInternalItemDirPath(item);
    if(checkIfExistingInternalFilePath(path)){
        fs::remove_all(path, ec);
    }

    if(auto parent = item->parent()){
        parent->removeChild(item);
    } else {
        treeWidget.takeTopLevelItem(treeWidget.indexOfTopLevelItem(item));
    }
    delete item;

    saveIndexFileLater();
}


GroupNameDialog* BodyLibraryView::Impl::getOrCreateGroupNameDialog()
{
    if(!groupNameDialog){
        groupNameDialog = new GroupNameDialog(&treeWidget);
    }
    return groupNameDialog;
}


bool BodyLibraryView::createTopGroupWithDialog()
{
    return impl->createGroupWithDialog(nullptr);
}


bool BodyLibraryView::createGroupWithDialog(ElementPtr parentGroup)
{
    return impl->createGroupWithDialog(parentGroup);
}


bool BodyLibraryView::Impl::createGroupWithDialog(ElementPtr parentGroup)
{
    QTreeWidgetItem* parentItem;
    if(parentGroup){
        parentItem = getTreeWidgetItem(*parentGroup);
    } else {
        parentItem = treeWidget.invisibleRootItem();
    }
    return createGroupWithDialog(parentItem);

}

bool BodyLibraryView::Impl::createGroupWithDialog(QTreeWidgetItem* parentItem)
{
    if(needToLoadIndexFile){
        loadIndexFile();
    }
    
    bool created = false;

    if(!parentItem){
        parentItem = treeWidget.invisibleRootItem();
    }
    
    auto dialog = getOrCreateGroupNameDialog();
    if(parentItem == treeWidget.invisibleRootItem()){
        dialog->setWindowTitle(_("Create Group"));
    } else {
        dialog->setWindowTitle(_("Create Sub Group"));
    }
    dialog->applyButton->setText(_("&Create"));
    
    if(dialog->exec() == QDialog::Accepted){
        string groupName = dialog->nameEdit->string();
        if(!checkNameDuplication(parentItem, groupName)){
            auto groupItem = createGroupItem(groupName);
            auto block = rowsInsertedConnection.scopedBlock();
            parentItem->addChild(groupItem);
            selectAndScrollToItem(groupItem);
            saveIndexFile();
            created = true;
        } else {
            showErrorDialog(
                format(_("Group name \"{0}\" is duplicated. Please use another name for a new group."),
                       groupName));
        }
    }

    return created;
}


bool BodyLibraryView::registerBodyWithDialog
(BodyItem* bodyItem, Mapping* extraInfo, ElementPtr group, bool doStoreFiles)
{
    if(impl->needToLoadIndexFile){
        if(!impl->loadIndexFile()){
            return false;
        }
    }
    auto groupItem = reinterpret_cast<LibraryItem*>(group->handle);
    return impl->registerBodyWithDialog(bodyItem, extraInfo, groupItem, nullptr, doStoreFiles);
}


bool BodyLibraryView::Impl::registerBodyWithDialog
(BodyItem* bodyItem, Mapping* extraInfo, QTreeWidgetItem* groupItem, QTreeWidgetItem* position, bool doStoreFiles)
{
    string name = bodyItem->name();

    if(name.empty()){
        showErrorDialog(_("The body cannot be registered in the body library due to the lack of its name."));
        return false;
    }
    
    if(checkNameDuplication(groupItem, name)){
        showErrorDialog(
            format(_("Name \"{0}\" of the body is already registered in the same group. "
                     "Please modify the name or group to avoid the name duplication."),
                   name));
        return false;
    }

    bool exported = false;
    string file = bodyItem->filePath();
    if(file.empty()){
        file = exportBodyFileToLibraryDirectory(bodyItem, groupItem);
        if(file.empty()){
            return false;
        }
        exported = true;
    }
    
    auto libraryItem = new LibraryItem;
    libraryItem->name = name;
    libraryItem->file = file;
    libraryItem->isGroup = false;
    libraryItem->extraInfo = extraInfo;

    if(!exported && doStoreFiles){
        if(!storeItemFilesToLibraryDirectory(libraryItem, bodyItem, groupItem, MessageOut::interactive())){
            delete libraryItem;
            return false;
        }
    }

    auto block = rowsInsertedConnection.scopedBlock();
    if(position){
        groupItem->insertChild(groupItem->indexOfChild(position), libraryItem);
    } else {
        groupItem->addChild(libraryItem);
    }

    setItemImage(libraryItem);
    selectAndScrollToItem(libraryItem);
    saveIndexFile();

    return true;
}


/*
  Try to export the body as a body file to register it in the body library
  \return The body file path string. An empty string when exporting failed.
*/
string BodyLibraryView::Impl::exportBodyFileToLibraryDirectory(BodyItem* bodyItem, QTreeWidgetItem* groupItem)
{
    string file;
    string name = bodyItem->name();

    bool confirmed =
        showConfirmDialog(
            _("Export to Body File"),
            format(
                _("Body item \"{0}\" cannot be registered in the body library because "
                  "it does not have its corresponding body file. "
                  "Do you want to export it as a body file for registration in the body library?"),
                name));
    if(!confirmed){
        return file;
    }

    auto dirPath = getInternalItemDirPath(groupItem, name);
    if(!ensureDirectory(dirPath, true)){
        showErrorDialog(
            format(_("The directory \"{0}\" to put the body file in the body library cannot be created. "
                     "The registration was canceled."),
                   toUTF8(dirPath.string())));
        return file;
    }
    
    auto filePath = dirPath / (name + ".body");
    file = toUTF8(filePath.generic_string());

    if(!bodyItem->save(file, "CHOREONOID-BODY")){
        showErrorDialog(
            format(_("The directory \"{0}\" in the body library cannot be created. "
                     "The registration was canceled."),
                   toUTF8(dirPath.string())));
        file.clear();
    }

    return file;
}


// Note that the bodyItem argument can be nullptr. In that case, a body item is temporarily loaded if necessary.
bool BodyLibraryView::Impl::storeItemFilesToLibraryDirectory
(LibraryItem* libraryItem, BodyItemPtr bodyItem, QTreeWidgetItem* groupItem, MessageOut* mout)
{
    // Note that files in the reference directories are not target
    
    string storedBodyFile;
    fs::path orgBodyFilePath(fromUTF8(libraryItem->file));
    fs::path orgBodyFileDirPath = orgBodyFilePath.parent_path();
    fs::path destDirPath = getInternalItemDirPath(groupItem, libraryItem->name);
    
    auto rp = getRelativePath(destDirPath, orgBodyFileDirPath);
    if(!rp){
        return false;
    }

    bool failedToCopyFiles = false;
    
    if(!(*rp).empty()){
         // The directory of the current file is different from the internal directory

        if(!bodyItem){
            // BodyItem object is temporarily used to detect dependent files
            bodyItem = new BodyItem;
            bodyItem->load(libraryItem->file);
        }
        vector<string> dependentFiles;
        bodyItem->getDependentFiles(dependentFiles);

        for(auto& file : dependentFiles){
            fs::path filePath(fromUTF8(file));
            if(!checkIfSubFilePath(filePath, orgBodyFileDirPath)){
                mout->putErrorln(
                    format(_("{0} cannot be stored in the library directory becasue its element file \"{1}\" "
                             "is not placed in the same or lower directory as the main model file \"{2}\"."),
                           libraryItem->name, file, libraryItem->file));
                return false;
            }
        }

        if(!ensureDirectory(destDirPath, true)){
            mout->putErrorln(
                format(_("{0} cannot be stored in the library directory due to an error in creating sub directory \"{1}\"."),
                       libraryItem->name, toUTF8(destDirPath.string())));
            return false;
        }

        if(!dependentFiles.empty()){
            fs::path srcDirPath = orgBodyFilePath.parent_path();
            for(auto& srcFile : dependentFiles){
                fs::path srcFilePath(fromUTF8(srcFile));
                if(auto relativePath = getRelativePath(srcFilePath, srcDirPath)){
                    fs::path destFilePath = destDirPath / *relativePath;
                    if(!copyFile(srcFilePath, destFilePath)){
                        failedToCopyFiles = true;
                        break;
                    }
                }
            }
        }
        if(!failedToCopyFiles){
            storedBodyFile = toUTF8((destDirPath / orgBodyFilePath.filename()).generic_string());
        }
    }

    string storedThumbnailFile;;    
    if(!failedToCopyFiles && !libraryItem->thumbnailFile.empty()){
        fs::path orgThumbnailFilePath(fromUTF8(libraryItem->thumbnailFile));
        fs::path ext = orgThumbnailFilePath.extension();
        fs::path destThumbnailFilePath = destDirPath / "thumbnail";
        destThumbnailFilePath += ext;
        if(orgThumbnailFilePath != destThumbnailFilePath){
            if(copyFile(orgThumbnailFilePath, destThumbnailFilePath)){
                storedThumbnailFile = toUTF8(destThumbnailFilePath.generic_string());
            } else {
                failedToCopyFiles = true;
            }
        }
    }

    string bodyFile;
    if(!failedToCopyFiles){
        if(!storedBodyFile.empty()){
            libraryItem->file = storedBodyFile;
        }
        if(!storedThumbnailFile.empty()){
            libraryItem->thumbnailFile = storedThumbnailFile;
        }
    } else {
        stdx::error_code ec;
        fs::remove_all(destDirPath, ec);
        mout->putErrorln(
            format(_("{0} cannot be stored in the library directory due to a file copy error."),
                   libraryItem->name));
    }

    return !failedToCopyFiles;
}


bool BodyLibraryView::Impl::storeItemFilesToLibraryDirectoryRecursively(QTreeWidgetItem* item, MessageOut* mout)
{
    // TODO: Ignore files in the reference directories

    bool isBody = false;
    if(auto libraryItem = dynamic_cast<LibraryItem*>(item)){
        if(!libraryItem->isGroup){
            isBody = true;
            if(!libraryItem->file.empty()){
                fs::path filePath(fromUTF8(libraryItem->file));
                bool isFileInRefDir = false;
                for(auto& refDirPath : refDirPaths){
                    if(checkIfSubFilePath(filePath, refDirPath)){
                        isFileInRefDir = true;
                        break;
                    }
                }
                if(!isFileInRefDir){
                    if(!storeItemFilesToLibraryDirectory(
                           libraryItem, nullptr, libraryItem->parent(), mout)){
                        return false;
                    }
                }
            }
        }
    }
    if(!isBody){ // Group
        int n = item->childCount();
        for(int i=0; i < n; ++i){
            if(!storeItemFilesToLibraryDirectoryRecursively(item->child(i), mout)){
                return false;
            }
        }
    }
    return true;
}


bool BodyLibraryView::setBodyThumbnailWithDialog(ElementPtr element)
{
    return impl->setBodyThumbnailWithDialog(impl->getLibraryItem(*element));
}


bool BodyLibraryView::Impl::setBodyThumbnailWithDialog(LibraryItem* item)
{
    if(!item || item->isGroup){
        return false;
    }

    FileDialog dialog;
    dialog.setWindowTitle(format(_("Select Thumbnail Image for {0}"), item->name));
    dialog.setViewMode(QFileDialog::List);
    dialog.setFileMode(QFileDialog::ExistingFile);
    dialog.setLabelText(QFileDialog::Accept, _("Select"));
    dialog.updatePresetDirectories();

    fs::path filePath(fromUTF8(item->file));
    bool isBodyCopiedInLibraryDirectory = checkIfInternalFilePath(filePath);

    if(!isBodyCopiedInLibraryDirectory){
        auto fileDirPath = filePath.parent_path();
        if(fs::exists(fileDirPath)){
            dialog.setDirectory(fileDirPath.string());
        }
    }

    QStringList filters;
    filters << _("Images files (*.png *.jpg *.jpeg)");
    filters << _("Any files (*)");
    dialog.setNameFilters(filters);

    bool result = false;
    if(dialog.exec()){
        string selectedFile = dialog.selectedFiles().at(0).toStdString();
        fs::path selectedFilePath(fromUTF8(selectedFile));

        bool copied = false;
        if(isBodyCopiedInLibraryDirectory || checkIfInternalFilePath(selectedFilePath)){
            fs::path ext = selectedFilePath.extension();
            fs::path destDir = getInternalItemDirPath(item);
            fs::path destFilePath = destDir / "thumbnail";
            destFilePath += ext;
            if(copyFile(selectedFilePath, destFilePath)){
                copied = true;
                item->thumbnailFile = toUTF8(destFilePath.generic_string());
            }
        }
        if(!copied){
            item->thumbnailFile = selectedFile;
        }
        setItemImage(item);
        saveIndexFile();
        result = true;
    }

    return result;
}


void BodyLibraryView::Impl::clearBodyThumbnail(LibraryItem* item)
{
    item->thumbnailFile.clear();
    setItemImage(item);
    saveIndexFile();
}


bool BodyLibraryView::renameLibraryItemWithDialog(ElementPtr group)
{
    return impl->renameLibraryItemWithDialog(impl->getLibraryItem(*group));
}


bool BodyLibraryView::Impl::renameLibraryItemWithDialog(LibraryItem* item)
{
    if(!item){
        return false;
    }

    bool renamed = false;
    
    auto dialog = getOrCreateGroupNameDialog();
    if(item->isGroup){
        dialog->setWindowTitle(_("Rename Group"));
    } else {
        dialog->setWindowTitle(_("Modify Registration Name"));
    }
    dialog->applyButton->setText(_("&Apply"));
    dialog->nameEdit->setText(item->name);

    if(dialog->exec() == QDialog::Accepted){
        string newName = dialog->nameEdit->string();
        if(newName != item->name){
            QTreeWidgetItem* parentItem = item->parent();
            if(!parentItem){
                parentItem = treeWidget.invisibleRootItem();
            }
            if(!checkNameDuplication(parentItem, newName)){
                if(!renameLibraryItem(item, newName)){
                    showErrorDialog(
                        _("The name cannot be modified because the corresponding files "
                          "in the library directory cannot be moved to the renamed ones."));
                } else {
                    if(item->isGroup){
                        item->setText(0, newName.c_str());
                    } else {
                        setItemImage(item);
                    }
                    renamed = true;
                }
            } else {
                showErrorDialog(
                    format(_("Name \"{0}\" is duplicated. Please use another name."), newName));
            }
        }
    }

    dialog->nameEdit->clear();

    return renamed;
}


void BodyLibraryView::Impl::forEachTopItems
(const QList<QTreeWidgetItem*>& items, function<bool(LibraryItem*)> callback)
{
    unordered_set<QTreeWidgetItem*> itemSet(items.size());
    for(auto& item : items){
        itemSet.insert(item);
    }
    for(auto& item : items){
        bool isChild = false;
        auto parent = item->parent();
        while(parent){
            if(itemSet.find(parent) != itemSet.end()){
                isChild = true;
                break;
            }
            parent = parent->parent();
        }
        if(!isChild){
            if(!callback(static_cast<LibraryItem*>(item))){
                break;
            }
        }
    }
}


void BodyLibraryView::Impl::removeSelectedItemsWithDialog()
{
    auto selected = treeWidget.selectedItems();
    if(selected.empty()){
        return;
    }

    bool confirmed =
        showConfirmDialog(
            _("Remove Selected Items"),
            _("Do you really want to remove the selected items? "
              "Note that files stored internally in the library directory will also be removed."));
    if(!confirmed){
        return;
    }

    forEachTopItems(
        selected,
        [this](LibraryItem* item){
            removeLibraryItem(item);
            return true;
        });
}


void BodyLibraryView::Impl::showDialogToMoveSelectedItems()
{
    auto selected = treeWidget.selectedItems();
    if(selected.empty()){
        return;
    }

    if(!selectionDialog){
        selectionDialog = new BodyLibrarySelectionDialog(self);
        selectionDialog->setWindowTitle(_("Move Selected Items"));
        selectionDialog->setAcceptButtonLabel(_("&Move"));
    }

    selectionDialog->updateToLatestBodyLibraryContents(self->getRootElement());
    selectionDialog->setFunctionOnAccepted(
        [this](BodyLibraryView::ElementPtr group){
            moveSelectedItemsWithDialog(getTreeWidgetItem(*group));
        });
    selectionDialog->show();
}


void BodyLibraryView::Impl::moveSelectedItemsWithDialog(QTreeWidgetItem* destItem)
{
    if(auto destLibraryItem = dynamic_cast<LibraryItem*>(destItem)){
        if(!destLibraryItem->isGroup){
            return;
        }
    }
            
    bool duplicated = false;
    vector<LibraryItem*> topItems;
    
    forEachTopItems(
        treeWidget.selectedItems(),
        [&](LibraryItem* item){
            auto parent = item->parent();
            if(!parent){
                parent = treeWidget.invisibleRootItem();
            }
            if(parent == destItem || !checkNameDuplication(destItem, item->name)){
                topItems.push_back(item);
                return true;
            }
            MessageOut::master()->putErrorln(
                format(_("Item \"{0}\" cannot be moved to \"{1}\" due to the name duplication."),
                       item->name, destItem->text(0).toStdString()));
            duplicated = true;
            return false;
        });

    if(!duplicated){
        for(auto& item : topItems){
            moveLibraryItem(item, destItem);
        }
    }
}


// Called when items are dropped inside the view
void BodyLibraryView::Impl::onTreeWidgetRowsInserted(const QModelIndex& parent, int start, int end)
{
    auto model = treeWidget.model();
    for(int row = start; row <= end; ++row){
        auto index = model->index(row, 0, parent);
        if(auto item = treeWidget.getLibraryItem(index)){
            auto it = libraryItemToOrgDirPathMap.find(item);
            if(it != libraryItemToOrgDirPathMap.end()){
                auto& orgDirPath = it->second;
                if(!relocateItemFiles(item, orgDirPath, item->parent())){
                    // Drop should be canceled when moving files failed.
                }
            }
            updateItemImagesRecursively(item);
            saveIndexFileLater();
        }
    }
}


void BodyLibraryView::Impl::updateItemImagesRecursively(LibraryItem* item)
{
    if(!item->isGroup){
        setItemImage(item);
    } else {
        int n = item->childCount();
        for(int i=0; i < n; ++i){
            updateItemImagesRecursively(static_cast<LibraryItem*>(item->child(i)));
        }
    }
}


void BodyLibraryView::Impl::onLibraryItemDoubleClicked(QTreeWidgetItem* item)
{
    if(auto libraryItem = dynamic_cast<LibraryItem*>(item)){
        if(!libraryItem->isGroup){
            self->loadBodyItem(libraryItem->name, libraryItem->file, libraryItem->extraInfo);
        }
    }
}


void BodyLibraryView::Impl::onContextMenuRequested(const QPoint& pos)
{
    bool isRoot = false;
    LibraryItem* libraryItem = nullptr;
    QTreeWidgetItem* item = treeWidget.itemAt(pos);
    if(!item){
        item = treeWidget.invisibleRootItem();
        isRoot = true;
    } else {
        libraryItem = static_cast<LibraryItem*>(item);
    }
    
    contextMenuManager.setNewPopupMenu(&treeWidget);

    if(libraryItem && !libraryItem->isGroup){
        contextMenuManager.addItem(_("Load"))->sigTriggered().connect(
            [this, libraryItem]{
                self->loadBodyItem(libraryItem->name, libraryItem->file, libraryItem->extraInfo); });
    } else {
        auto createAction = contextMenuManager.addItem(
            isRoot ? _("Create Group") : _("Create Sub Group"));
        createAction->sigTriggered().connect([this, item]{ createGroupWithDialog(item); });
    }

    contextMenuManager.addSeparator();

    auto moveAction = contextMenuManager.addItem(_("Move"));
    moveAction->sigTriggered().connect([this]{ showDialogToMoveSelectedItems(); });

    auto removeAction = contextMenuManager.addItem(_("Remove"));
    removeAction->sigTriggered().connect([this]{ removeSelectedItemsWithDialog(); });

    if(treeWidget.selectedItems().empty()){
        moveAction->setEnabled(false);
        removeAction->setEnabled(false);
    }

    if(libraryItem){
        contextMenuManager.addSeparator();
        contextMenuManager.addItem(_("Rename"))->sigTriggered().connect(
            [this, libraryItem]{ renameLibraryItemWithDialog(libraryItem); });

        if(!libraryItem->isGroup){
            contextMenuManager.addItem(_("Set Thumbnail"))->sigTriggered().connect(
                [this, libraryItem]{ setBodyThumbnailWithDialog(libraryItem); });
            contextMenuManager.addItem(_("Clear Thumbnail"))->sigTriggered().connect(
                [this, libraryItem]{ clearBodyThumbnail(libraryItem); });
        }
    }
    
    contextMenuManager.popupMenu()->popup(treeWidget.mapToGlobal(pos));
}


bool BodyLibraryView::eventFilter(QObject* obj, QEvent* event)
{
    if(auto sceneView = dynamic_cast<SceneView*>(obj)){
        switch(event->type()){
        case QEvent::DragEnter:
            return impl->onSceneViewDragEnterEvent(sceneView, static_cast<QDragEnterEvent*>(event));
        case QEvent::Drop:
            return impl->onSceneViewDropEvent(sceneView, static_cast<QDropEvent*>(event));
        default:
            break;
        }
    }
    return false;
}


bool BodyLibraryView::Impl::onSceneViewDragEnterEvent(SceneView* sceneView, QDragEnterEvent* event)
{
    if(auto data = dynamic_cast<const ExMimeData*>(event->mimeData())){
        event->acceptProposedAction();
        return true;
    }
    return false;
}


bool BodyLibraryView::Impl::onSceneViewDropEvent(SceneView* sceneView, QDropEvent* event)
{
    if(auto data = dynamic_cast<const ExMimeData*>(event->mimeData())){
        for(auto& item : data->libraryItems){
            if(!item->isGroup){
                self->loadBodyItem(item->name, item->file, item->extraInfo);
            }
        }
        return true;
    }
    return false;
}


void BodyLibraryView::Impl::initializeFilePathVariableProcessor()
{
    if(!filePathVariableProcessor){
        filePathVariableProcessor = FilePathVariableProcessor::systemInstance();
    }
    orgBaseDirPath = filePathVariableProcessor->baseDirPath();
    orgProjectDirPath = filePathVariableProcessor->projectDirPath();
    filePathVariableProcessor->setBaseDirPath(libraryDirPath);
    filePathVariableProcessor->clearProjectDirectory();
}


void BodyLibraryView::Impl::finalizeFilePathVariableProcessor()
{
    filePathVariableProcessor->setBaseDirPath(orgBaseDirPath);
    filePathVariableProcessor->setProjectDirPath(orgProjectDirPath);
}


BodyItem* BodyLibraryView::loadBodyItem
(const std::string& name, const std::string& filename, Mapping* /* extraInfo */)
{
    bool loaded = false;
    auto rootItem = RootItem::instance();
    Item* parentItem = nullptr;
    
    BodyItemPtr bodyItem = new BodyItem;
    bodyItem->setName(name);
    bodyItem->setChecked(true);
    
    if(bodyItem->load(filename)){
        for(auto& item : rootItem->selectedItems()){
            if(auto worldItem = dynamic_pointer_cast<WorldItem>(item)){
                parentItem = worldItem;
                break;
            }
        }
        if(!parentItem){
            if(auto worldItem = rootItem->findItem<WorldItem>()){
                parentItem = worldItem;
            }
        }
        if(!parentItem){
            parentItem = rootItem;
        }
        loaded = parentItem->addChildItem(bodyItem);
    }

    if(!loaded){
        bodyItem.reset();
    }

    return bodyItem;
}


bool BodyLibraryView::Impl::loadIndexFile()
{
    if(indexFile.empty()){
        return false;
    }
    
    if(noImageIcon.isNull()){
        noImageIcon = QIcon(":/Body/icon/question.svg");
        noImagePixmap = noImageIcon.pixmap(QSize(imageSize, imageSize));
    }
    
    treeWidget.clear();

    bool loaded = false;
    
    YAMLReader reader;
    bool doFinalizePathVariableProcessor = false;
    try {
        auto archive = reader.loadDocument(indexFile)->toMapping();
        auto elements = archive->findListing("elements");
        if(elements->isValid()){
            initializeFilePathVariableProcessor();
            doFinalizePathVariableProcessor = true;
            auto block = rowsInsertedConnection.scopedBlock();
            readLibraryElements(elements, treeWidget.invisibleRootItem());
            loaded = true;
        }
    } catch(const ValueNode::Exception& ex){
        MessageOut::master()->putErrorln(
            format(_("Failed to load the body library index file \"{0}\": {1}"),
                   indexFile, ex.message()));
    }
    if(doFinalizePathVariableProcessor){
        finalizeFilePathVariableProcessor();
    }

    needToLoadIndexFile = false;
    needToSaveIndexFile = false;

    return loaded;
}


void BodyLibraryView::Impl::readLibraryElements(Listing* elements, QTreeWidgetItem* parent)
{
    for(auto& node : *elements){
        auto info = node->toMapping();
        if(auto item = readLibraryItem(info)){
            parent->addChild(item);
            if(!item->isGroup){
                setItemImage(item);
            } else {
                if(auto elementsNode = info->extract("elements")){
                    readLibraryElements(elementsNode->toListing(), item);
                }
            }
        }
    }
}


LibraryItem* BodyLibraryView::Impl::readLibraryItem(Mapping* info)
{
    auto mout = MessageOut::master();

    string name;
    if(!info->extract("name", name)){
        mout->putErrorln(
            format(_("Name is not specified for item in the index file \"{0}\"."), indexFile));
        return nullptr;
    }
            
    string type;
    bool isGroup;
    if(info->extract("type", type)){
        if(type == "body"){
            isGroup = false;
        } else if(type == "group"){
            isGroup = true;
        } else {
            mout->putErrorln(
                format(_("Invalid type name \"{0}\" in the index file \"{1}\"."), type, indexFile));
            return nullptr;
        }
    }

    LibraryItem* item = nullptr;
    
    if(isGroup){
        item = createGroupItem(name);

    } else {
        string file;
        if(!info->extract("file", file)){
            mout->putErrorln(
                format(_("File is not specified for item \"{0}\" in the index file \"{1}\"."),
                       name, indexFile));
            return nullptr;
        }
        file = filePathVariableProcessor->expand(file, true);

        string thumbnailFile;
        if(info->extract("thumbnail", thumbnailFile)){
            thumbnailFile = filePathVariableProcessor->expand(thumbnailFile, true);
        }

        item = createBodyItem(name, file, thumbnailFile);
    }

    item->extraInfo = info;

    return item;
}


bool BodyLibraryView::saveIndexFileIfUpdated()
{
    return impl->saveIndexFileIfUpdated();
}


bool BodyLibraryView::Impl::saveIndexFileIfUpdated()
{
    bool saved = false;
    if(needToSaveIndexFile){
        if(indexFile.empty()){
            // Show the dialog to select a index file
        } else {
            saved = saveIndexFile();
        }
    }
    return saved;
}


bool BodyLibraryView::Impl::saveIndexFile()
{
    return saveIndexFile(treeWidget.invisibleRootItem());
}

bool BodyLibraryView::Impl::saveIndexFile(QTreeWidgetItem* rootItem)
{
    if(indexFile.empty()){
        return false;
    }

    bool saved = false;
    initializeFilePathVariableProcessor();
    
    try {
        YAMLWriter writer(indexFile);
        writer.putComment("Body library index file\n");
        writer.startMapping();
        writeLibraryElements(writer, rootItem);
        writer.endMapping();
        needToSaveIndexFile = false;
        saved = true;
    }
    catch(const ValueNode::Exception& ex){
        MessageOut::master()->putErrorln(ex.message());
    }

    finalizeFilePathVariableProcessor();

    return saved;
}


void BodyLibraryView::Impl::writeLibraryElements(YAMLWriter& writer, QTreeWidgetItem* groupItem)
{
    int n = groupItem->childCount();
    if(n > 0){
        writer.putKey("elements");
        writer.startListing();
        for(int i=0; i < n; ++i){
            auto item = static_cast<LibraryItem*>(groupItem->child(i));
            writer.startMapping();
            
            if(!item->isGroup){
                writer.putKeyValue("type", "body");
                writer.putKey("name");
                writer.putDoubleQuotedString(item->name);
                writer.putKey("file");
                writer.putDoubleQuotedString(filePathVariableProcessor->parameterize(item->file));
                if(!item->thumbnailFile.empty()){
                    writer.putKey("thumbnail");
                    writer.putDoubleQuotedString(filePathVariableProcessor->parameterize(item->thumbnailFile));
                }
                if(item->extraInfo && !item->extraInfo->empty()){
                    writer.putNode(item->extraInfo);
                }
            } else {
                writer.putKeyValue("type", "group");
                writer.putKey("name");
                writer.putDoubleQuotedString(item->name);
                writeLibraryElements(writer, item);
            }

            writer.endMapping();
        }
        writer.endListing();
    }
}


bool BodyLibraryView::importLibrary(const std::string& filename, MessageOut* mout)
{
    if(!mout){
        mout = MessageOut::master();
    }
    return impl->importLibrary(filename, mout);
}


bool BodyLibraryView::Impl::importLibrary(const std::string& filename, MessageOut* mout)
{
    if(libraryDirectory.empty()){
        return false;
    }

    fs::path extractionDirPath = libraryDirPath / "tmp";
    stdx::error_code ec;
    
    fs::create_directories(extractionDirPath, ec);
    if(ec){
        mout->putErrorln(
            format(_("Temporary directory for extracting a body library cannot be created in the library directory: {1}"),
                   toUTF8(ec.message())));
        return false;
    }

    fs::path filePath(fromUTF8(filename));
    ZipArchiver archiver;
    bool imported = false;
    
    if(!archiver.extractZipFile(filename, toUTF8(extractionDirPath.string()))){
        showErrorDialog(archiver.errorMessage());
        fs::remove_all(extractionDirPath);

    } else {
        for(const fs::directory_entry& entry : fs::directory_iterator(libraryDirPath)){
            if(!fs::equivalent(entry, extractionDirPath, ec)){
                fs::remove_all(entry.path(), ec);
            }
        }
        fs::path srcTopDirPath = extractionDirPath / filePath.stem();
        bool failed = false;
        for(const fs::directory_entry& entry : fs::directory_iterator(srcTopDirPath)){
            auto& entryPath = entry.path();
            fs::path destPath = libraryDirPath / entryPath.filename();
#if __cplusplus > 201402L
            fs::copy(
                entryPath, destPath,
                fs::copy_options::overwrite_existing | fs::copy_options::recursive, ec);
#else
            fs::copy_directory_recursively(entryPath, destPath, ec);
#endif
            if(ec){
                failed = true;
            }
        }
        if(!failed){
            needToLoadIndexFile = true;
            if(self->isActive()){
                loadIndexFile();
            }
            string fname = toUTF8(filePath.filename().string());
            mout->putln(
                format(_("Body library archive \"{0}\" has been imported."), fname));
            imported = true;
        } else {
            mout->putErrorln(
                format(_("Extracted files cannot be copied into the library directory: {1}"),
                       toUTF8(ec.message())));
        }
    }

    fs::remove_all(extractionDirPath, ec);
    
    return imported;
}


bool BodyLibraryView::exportLibrary(const std::string& filename, MessageOut* mout)
{
    if(!mout){
        mout = MessageOut::master();
    }
    return impl->exportLibrary(filename, mout);
}


bool BodyLibraryView::Impl::exportLibrary(const std::string& filename, MessageOut* mout)
{
    bool exported = false;
    auto directory = filename + ".tmp";
    if(exportLibraryToDirectory(directory, mout)){
        ZipArchiver archiver;
        if(archiver.createZipFile(filename, directory)){
            string fname = toUTF8(fs::path(fromUTF8(filename)).filename().string());
            mout->putln(
                format(_("The body library has been exported to \"{0}\"."), fname));
            exported = true;
        } else {
            mout->putErrorln(archiver.errorMessage());
        }
        fs::remove_all(directory);
    }
    return exported;
}


bool BodyLibraryView::Impl::exportLibraryToDirectory(const std::string& directory, MessageOut* mout)
{
    bool failed = false;
    stdx::error_code ec;
    fs::path exportDirPath(fromUTF8(directory));
    if(fs::exists(exportDirPath)){
        fs::remove_all(exportDirPath, ec);
        if(ec){
            failed = true;
        }
    }
    if(!failed){
        fs::create_directories(exportDirPath, ec);
        if(ec){
            failed = true;
        }
    }
    if(failed){
        mout->putErrorln(
            format(_("A temporary directory for exporting library files cannot be created: {0}"),
                   toUTF8(ec.message())));
        return false;
    }
    
#if __cplusplus > 201402L
    fs::copy(
        libraryDirPath, exportDirPath,
        fs::copy_options::overwrite_existing | fs::copy_options::recursive, ec);
#else
    fs::copy_directory_recursively(libraryDirPath, exportDirPath, ec);
#endif
    if(ec){
        mout->putErrorln(
            format(_("Library files cannot be copied to a temporary directory for exportation: {0}"),
                   toUTF8(ec.message())));
        return false;
    }

    auto orgLibraryDirPath = libraryDirPath;
    auto orgIndexFile = indexFile;
    libraryDirPath = exportDirPath;
    indexFile = toUTF8((libraryDirPath / "index.yaml").string());

    bool exported = false;
    
    /*
      Note that the following code does not used LibraryItem::clone for deep copied items.
      QTreeWidgetItem* root = treeWidget.invisibleRootItem()->clone();
    */
    LibraryItem* root = new LibraryItem;
    int n = treeWidget.topLevelItemCount();
    for(int i=0; i < n; ++i){
        root->addChild(treeWidget.topLevelItem(i)->clone());
    }
    
    if(storeItemFilesToLibraryDirectoryRecursively(root, mout)){
        saveIndexFile(root);
        exported = true;
    }

    delete root;
    libraryDirPath = orgLibraryDirPath;
    indexFile = orgIndexFile;

    return exported;
}


bool BodyLibraryView::storeState(Archive& archive)
{
    return true;
}


bool BodyLibraryView::restoreState(const Archive& archive)
{
    return true;
}
