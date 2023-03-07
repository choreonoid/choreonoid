#include "MessageLogItem.h"
#include "ExtensionManager.h"
#include "ItemManager.h"
#include "MessageView.h"
#include "PutPropertyFunction.h"
#include "Archive.h"
#include <cnoid/ExecutablePath>
#include <cnoid/UTF8>
#include <cnoid/stdx/filesystem>
#include <fstream>
#include <regex>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace cnoid {

class MessageLogItem::Impl
{
public:
    MessageLogItem* self;
    MessageView* mv;

    string filename;
    ofstream ofs;
    ScopedConnection mvConnection;
    Selection fileMode;
    bool skipEscapeSequence;

    Impl(MessageLogItem* self);
    Impl(MessageLogItem* self, const Impl& org);
    ~Impl();
    void startWriting();
    void openFile();
    void onMessageOut(const std::string& text);
    void setFileName(const string& filename_);
};

}


void MessageLogItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<MessageLogItem, AbstractTextItem>(N_("MessageLogItem"));
    ext->itemManager().addCreationPanel<MessageLogItem>();
}


MessageLogItem::MessageLogItem()
{
    impl = new Impl(this);
}


MessageLogItem::MessageLogItem(const MessageLogItem& org)
    : AbstractTextItem(org)
{
    impl = new Impl(this, *org.impl);
}


MessageLogItem::~MessageLogItem()
{
    delete impl;
}


MessageLogItem::Impl::Impl(MessageLogItem* self)
    : self(self),
      mv(MessageView::instance()),
      fileMode(MessageLogItem::N_FILE_MODES, CNOID_GETTEXT_DOMAIN_NAME),
      skipEscapeSequence(true)
{
    filename.clear();

    fileMode.setSymbol(MessageLogItem::APPEND,    _("Append"));
    fileMode.setSymbol(MessageLogItem::OVERWRITE, _("Overwrite"));
}


MessageLogItem::Impl::Impl(MessageLogItem* self, const Impl& org)
    : self(self),
      mv(MessageView::instance()),
      fileMode(org.fileMode),
      skipEscapeSequence(org.skipEscapeSequence)
{
    filename = self->name() + ".log";
}


MessageLogItem::Impl::~Impl()
{
    ofs.close();
}


const std::string& MessageLogItem::textFilename() const
{
    return impl->filename;
}


Item* MessageLogItem::doCloneItem(CloneMap* /* cloneMap */) const
{
    return new MessageLogItem(*this);
}


void MessageLogItem::onConnectedToRoot()
{
    impl->startWriting();
}


void MessageLogItem::Impl::startWriting()
{
    openFile();
    
    if(!mvConnection.connected()){
        mvConnection = mv->sigMessage().connect([this](const std::string& text){ onMessageOut(text); });
    }
}


void MessageLogItem::Impl::openFile()
{
    auto block = mvConnection.scopedBlock();

    if(ofs.is_open()){
        ofs.close();
    }

    string nativeFilename = fromUTF8(filename);
    if(fileMode.selectedIndex()==MessageLogItem::APPEND){
        ofs.open(nativeFilename, ios_base::out | ios_base::app | ios_base::binary);
    }else{
        stdx::filesystem::path path(nativeFilename);
        if(stdx::filesystem::exists(path)){
            bool ok = showConfirmDialog(
                _("Confirm"),
                format(_(" \"{}\" already exists.\n Do you want to replace it? " ), filename));
            if(!ok){
                return;
            }
        }
        ofs.open(nativeFilename, ios_base::out | ios_base::binary);
    }

    if(!ofs){
        mv->putln(format(_("Couldn't open file \"{}\" for writing.\n"), filename),
                  MessageView::Error);
    }else{
        mv->putln(format(_("Opened file \"{}\" for writing.\n"), filename));
    }
}


void MessageLogItem::onDisconnectedFromRoot()
{
    impl->mvConnection.disconnect();
    impl->ofs.close();
}


void MessageLogItem::Impl::onMessageOut(const std::string& text)
{
    if(ofs.is_open()){
        // skip escapeSequence
        if( skipEscapeSequence && text.find("\x1b", 0) != string::npos ){
            QString qtext(text.c_str());
            qtext.replace(QRegExp("\\\x1b\\[[0-9;]*[A-z]"), "");
            ofs << qtext.toStdString();

            /* std::regex version
            regex  escapePattern("\\\x1b\\[[0-9;]*[A-z]");
            ofs << regex_replace(text, escapePattern, "");
            */
        } else {
            ofs << text;
        }
        ofs.flush();
    }
}


void MessageLogItem::Impl::setFileName(const string& filename_)
{
    if(ofs.is_open() && filename==filename_)
        return;

    filename = filename_;
    stdx::filesystem::path path(fromUTF8(filename));
    string ext = path.extension().string();
    if(ext != ".log"){
        filename += ".log";
    }

    openFile();
}


void MessageLogItem::doPutProperties(PutPropertyFunction& putProperty)
{
    FilePathProperty filenameProperty(impl->filename, { _("Message view log file (*.log)") });
    filenameProperty.setExistingFileMode(false);
    putProperty(_("file name"), filenameProperty,
            [&](const string& filename){ impl->setFileName(filename); return true; });
    putProperty(_("File mode"), impl->fileMode,
                    [&](int index){ impl->fileMode.selectIndex(index); impl->openFile(); return true; });
    putProperty(_("skip escapeSequence"), impl->skipEscapeSequence, changeProperty(impl->skipEscapeSequence));

}


bool MessageLogItem::store(Archive& archive)
{
    archive.writeRelocatablePath("file", impl->filename);
    archive.write("file_mode", impl->fileMode.selectedSymbol());
    archive.write("skip_escape_sequence", impl->skipEscapeSequence);
    return true;
}


bool MessageLogItem::restore(const Archive& archive)
{
    string symbol;
    if(archive.read({ "file_mode", "fileMode" }, symbol)){
        impl->fileMode.select(symbol);
    }
    if(archive.read({ "file", "fileName" }, impl->filename)){
        impl->filename = archive.resolveRelocatablePath(impl->filename);
    }
    archive.read({ "skip_escape_sequence", "skipEscapeSequence" }, impl->skipEscapeSequence);
    return true;
}
