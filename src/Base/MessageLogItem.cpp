/**
   \file
   \author Shizuko Hattori
*/

#include "MessageLogItem.h"
#include <cnoid/ExtensionManager>
#include <cnoid/ItemManager>
#include <cnoid/MessageView>
#include <cnoid/ExecutablePath>
#include <cnoid/Archive>
#include <cnoid/stdx/filesystem>
#include <fstream>
#include <regex>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace cnoid {

class MessageLogItemImpl
{
public:
    MessageLogItem* self;
    MessageView* mv;

    string filename;
    ofstream ofs;
    Connection mvConnection;
    Selection fileMode;
    bool skipEscapeSequence;

    MessageLogItemImpl(MessageLogItem* self);
    MessageLogItemImpl(MessageLogItem* self, const MessageLogItemImpl& org);
    ~MessageLogItemImpl();
    void startWriting();
    void openFile();
    void onMessageOut(const std::string& text);
    void setFileName(const string& filename_);
};

}


void MessageLogItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<MessageLogItem>(N_("MessageLogItem"));
    ext->itemManager().addCreationPanel<MessageLogItem>();
}


MessageLogItem::MessageLogItem()
{
    impl = new MessageLogItemImpl(this);
}


MessageLogItem::MessageLogItem(const MessageLogItem& org)
    : AbstractTextItem(org)
{
    impl = new MessageLogItemImpl(this, *org.impl);
}


MessageLogItem::~MessageLogItem()
{
    delete impl;
}


MessageLogItemImpl::MessageLogItemImpl(MessageLogItem* self)
    : self(self),
      mv(MessageView::instance()),
      fileMode(MessageLogItem::N_FILE_MODES, CNOID_GETTEXT_DOMAIN_NAME),
      skipEscapeSequence(true)
{
    filename.clear();

    fileMode.setSymbol(MessageLogItem::APPEND,    _("Append"));
    fileMode.setSymbol(MessageLogItem::OVERWRITE, _("Overwrite"));
}


MessageLogItemImpl::MessageLogItemImpl(MessageLogItem* self, const MessageLogItemImpl& org)
    : self(self),
      mv(MessageView::instance()),
      fileMode(org.fileMode),
      skipEscapeSequence(org.skipEscapeSequence)
{
    filename = self->name() + ".log";
}


MessageLogItemImpl::~MessageLogItemImpl()
{
    mvConnection.disconnect();
    ofs.close();
}

const std::string& MessageLogItem::textFilename() const
{
    return impl->filename;
}


Item* MessageLogItem::doDuplicate() const
{
    return new MessageLogItem(*this);
}


void MessageLogItem::onConnectedToRoot()
{
    impl->startWriting();
}


void MessageLogItemImpl::startWriting()
{
    openFile();

    mvConnection = mv->sigMessage().connect([&](const std::string& text){ onMessageOut(text); });

}


void MessageLogItemImpl::openFile()
{
    mvConnection.block();

    if(ofs.is_open()){
        ofs.close();
    }

    if(fileMode.selectedIndex()==MessageLogItem::APPEND){
        ofs.open(filename, ios::app);
    }else{
        stdx::filesystem::path path(filename);
        if(stdx::filesystem::exists(path)){
            bool ok = showConfirmDialog(
                _("Confirm"),
                format(_(" \"{}\" already exists.\n Do you want to replace it? " ), filename));
            if(!ok){
                mvConnection.unblock();
                return;
            }
        }
        ofs.open(filename, ios::out);
    }

    if(!ofs){
        mv->putln(format(_("Couldn't open file \"{}\" for writing.\n"), filename),
                  MessageView::ERROR);
    }else{
        mv->putln(format(_("Opened file \"{}\" for writing.\n"), filename));
    }

    mvConnection.unblock();
}


void MessageLogItem::onDisconnectedFromRoot()
{
    impl->mvConnection.disconnect();
    impl->ofs.close();
}


void MessageLogItemImpl::onMessageOut(const std::string& text)
{
    if(ofs.is_open()){
        // skip escapeSequence
        if( skipEscapeSequence && text.find( "\e", 0 ) != string::npos ){

// Actually I'd like to use std :: regex,
// but since I can not compile with version of Ubuntu 14.04, I implemented it with Qt library
#if 1
            QString qtext(text.c_str());
            qtext.replace(QRegExp("\\\e\\[[0-9;]*[A-z]"), "");
            ofs << qtext.toStdString();
#else
            regex  escapePattern("\\\e\\[[0-9;]*[A-z]");
            ofs << regex_replace(text, escapePattern, "");
#endif

        }else{
            ofs << text;
        }
        ofs.flush();
    }
}


void MessageLogItemImpl::setFileName(const string& filename_)
{
    if(ofs.is_open() && filename==filename_)
        return;

    filename = filename_;
    stdx::filesystem::path path(filename);
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
    archive.writeRelocatablePath("fileName", impl->filename);
    archive.write("fileMode", impl->fileMode.selectedSymbol());
    archive.write("skipEscapeSequence", impl->skipEscapeSequence);
    return true;
}

bool MessageLogItem::restore(const Archive& archive)
{
    string symbol;
    if(archive.read("fileMode", symbol)){
        impl->fileMode.select(symbol);
    }
    archive.readRelocatablePath("fileName", impl->filename);
    archive.read("skipEscapeSequence", impl->skipEscapeSequence);
    return true;
}




