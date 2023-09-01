#include "UriSchemeProcessor.h"
#include "FilePathVariableProcessor.h"
#include "UTF8.h"
#include <cnoid/stdx/filesystem>
#include <fmt/format.h>
#include <sstream>
#include <mutex>
#include <regex>
#include <unordered_map>
#include "gettext.h"

using namespace std;
using namespace cnoid;
namespace filesystem = stdx::filesystem;

namespace {

mutex uriSchemeHandlerMutex;
unordered_map<string, UriSchemeProcessor::UriSchemeHandler> uriSchemeHandlerMap;

}

namespace cnoid {

class UriSchemeProcessor::Impl
{
public:
    string scheme;
    string uriPath;
    ostringstream errorStream;
    bool hasFileScheme;
    bool hasSupportedScheme;
    regex uriSchemeRegex;
    bool isUriSchemeRegexReady;
    FilePathVariableProcessorPtr filePathVariableProcessor;
    stdx::filesystem::path baseDirPath;
    string baseDirectory;

    void ensureUriSchemeRegex();
    bool detectScheme(const std::string& uri);
    std::string getFilePath(const std::string& uri);
};

}


void UriSchemeProcessor::registerUriSchemeHandler(const std::string& scheme, UriSchemeHandler handler)
{
    std::lock_guard<std::mutex> guard(uriSchemeHandlerMutex);
    uriSchemeHandlerMap[scheme] = handler;
}


UriSchemeProcessor::UriSchemeProcessor()
{
    impl = new Impl;
    impl->isUriSchemeRegexReady = false;
}


UriSchemeProcessor::~UriSchemeProcessor()
{
    delete impl;
}


void UriSchemeProcessor::setFilePathVariableProcessor(FilePathVariableProcessor* processor)
{
    impl->filePathVariableProcessor = processor;
}


FilePathVariableProcessor* UriSchemeProcessor::filePathVariableProcessor()
{
    return impl->filePathVariableProcessor;
}


void UriSchemeProcessor::setBaseDirectory(const std::string& directory)
{
    impl->baseDirPath = fromUTF8(directory);
    impl->baseDirectory = directory;
}


void UriSchemeProcessor::setBaseDirectoryFor(const std::string& filename)
{
    impl->baseDirPath = fromUTF8(filename);
    impl->baseDirPath = impl->baseDirPath.parent_path();
    if(impl->baseDirPath.is_relative()){
        impl->baseDirPath = filesystem::current_path() /  impl->baseDirPath;
    }
    impl->baseDirectory = toUTF8(impl->baseDirPath.generic_string());
}


std::string UriSchemeProcessor::baseDirectory() const
{
    return impl->baseDirectory;
}


void UriSchemeProcessor::Impl::ensureUriSchemeRegex()
{
    if(!isUriSchemeRegexReady){
        uriSchemeRegex.assign("^(.+)://(.+)$");
        isUriSchemeRegexReady = true;
    }
}    


bool UriSchemeProcessor::detectScheme(const std::string& uri)
{
    return impl->detectScheme(uri);
}


bool UriSchemeProcessor::Impl::detectScheme(const std::string& uri)
{
    ensureUriSchemeRegex();
    scheme.clear();
    hasSupportedScheme = false;
    hasFileScheme = false;
    errorStream.str("");

    smatch match;
    if(!regex_match(uri, match, uriSchemeRegex)){
        uriPath = uri;
    } else {
        scheme = match.str(1);
        uriPath = match.str(2);
        if(scheme == "file"){
            hasFileScheme = true;
        }
    }
    
    return !scheme.empty();
}


std::string UriSchemeProcessor::getFilePath(const std::string& uri)
{
    return impl->getFilePath(uri);
}


std::string UriSchemeProcessor::Impl::getFilePath(const std::string& uri)
{
    string filePath;
    
    bool hasScheme = detectScheme(uri);
    
    if(!hasScheme){
        filePath = uri;
    } else {
        if(hasFileScheme){
            filePath = uriPath;
            hasSupportedScheme = true;
        } else {
            std::lock_guard<std::mutex> guard(uriSchemeHandlerMutex);
            auto it = uriSchemeHandlerMap.find(scheme);
            if(it == uriSchemeHandlerMap.end()){
                errorStream <<
                    fmt::format(_("The \"{0}\" scheme in URI \"{1}\" is not supported."), scheme, uri);
                errorStream.flush();
            } else {
                auto& handler = it->second;
                filePath = handler(uriPath,  errorStream);
                hasSupportedScheme = true;
            }
        }
    }

    if(!filePath.empty()){
        if(filePathVariableProcessor){
            filePath = filePathVariableProcessor->expand(filePath, true);
            if(filePath.empty()){
                errorStream << filePathVariableProcessor->errorMessage();
                errorStream.flush();
            }
        } else if(!baseDirPath.empty()){
            stdx::filesystem::path path(fromUTF8(filePath));
            if(!path.is_absolute()){
                path = filesystem::lexically_normal(baseDirPath / path);
                filePath = toUTF8(path.generic_string());
            }
        }
    }

    return filePath;
}


std::string UriSchemeProcessor::getParameterizedFilePath(const std::string& uri)
{
    auto filePath = impl->getFilePath(uri);
    if(impl->filePathVariableProcessor && isFileScheme() && !filePath.empty()){
        filePath = impl->filePathVariableProcessor->parameterize(filePath);
    }
    return filePath;
}


const std::string& UriSchemeProcessor::scheme() const
{
    return impl->scheme;
}


bool UriSchemeProcessor::isFileScheme() const
{
    return impl->scheme.empty() || impl->hasFileScheme;
}


bool UriSchemeProcessor::isSupportedScheme() const
{
    return impl->scheme.empty() || impl->hasSupportedScheme;
}


const std::string& UriSchemeProcessor::path()
{
    return impl->uriPath;
}


std::string UriSchemeProcessor::errorMessage() const
{
    return impl->errorStream.str();
}
