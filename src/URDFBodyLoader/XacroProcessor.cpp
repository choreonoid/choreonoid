#include "XacroProcessor.h"
#include <cnoid/Format>
#include <cnoid/UTF8>
#include <cnoid/ExecutablePath>
#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <ostream>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <cwctype>
#include <thread>
#include <vector>
#else
#include <cerrno>
#include <cstring>
#include <sys/select.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

using namespace cnoid;
using std::endl;
using std::string;
using std::vector;

namespace filesystem = std::filesystem;

namespace {

// Separator used to join entries of ROS_PACKAGE_PATH / PYTHONPATH style path lists. Windows
// drive letters contain ':', so the platform list separator must be used instead.
#ifdef _WIN32
const char pathListSeparator = ';';
#else
const char pathListSeparator = ':';
#endif

void addDirectoryIfUnique(vector<string>& directories, const filesystem::path& path)
{
    if (path.empty()) {
        return;
    }
    filesystem::path normalized = filesystem::absolute(path).lexically_normal();
    string directory = toUTF8(normalized.generic_string());
    if (std::find(directories.begin(), directories.end(), directory) == directories.end()) {
        directories.push_back(directory);
    }
}

// Builds the ROS_PACKAGE_PATH passed only to the xacro child process. This lets self-contained
// packages resolve $(find pkg) without changing the Choreonoid process environment.
string makeRosPackagePath(const vector<string>& localPackageSearchDirectories)
{
    string path;
    // Local paths inferred from the file being loaded take precedence over the process
    // environment so a self-contained model resolves its own package first.
    for (const auto& directory : localPackageSearchDirectories) {
        if (directory.empty()) {
            continue;
        }
        if (!path.empty()) {
            path += pathListSeparator;
        }
        path += directory;
    }

    // Preserve the user's ROS_PACKAGE_PATH as a fallback for references outside the local
    // model package.
    const char* existingPath = getenv("ROS_PACKAGE_PATH");
    if (existingPath && *existingPath) {
        if (!path.empty()) {
            path += pathListSeparator;
        }
        path += existingPath;
    }

    return path;
}

// Result of running cnoid-xacro.
struct XacroResult
{
    // Expanded URDF XML from stdout.
    string output;
    // Warnings and errors from stderr, kept separate from XML.
    string errorOutput;
    // Error from the process/pipe primitives (pipe(), fork(), select(), waitpid(), or the
    // corresponding Win32 calls).
    string systemError;
    // Exit code when the child terminates normally.
    int exitCode = -1;
    // Signal number when the child is killed by a signal (POSIX only).
    int signal = 0;
    // True after the child process has been created.
    bool started = false;
    // True when the child is reported to have terminated normally.
    bool exited = false;
};

#ifndef _WIN32

// Reads one ready pipe chunk. stdout and stderr are read alternately by select() below so the
// xacro child cannot block when either pipe buffer fills.
void appendPipeOutput(int fd, string& out, bool& open)
{
    char buffer[4096];
    ssize_t n = read(fd, buffer, sizeof(buffer));
    if (n > 0) {
        out.append(buffer, n);
    } else if (n == 0) {
        close(fd);
        open = false;
    } else if (errno != EINTR) {
        close(fd);
        open = false;
    }
}

// Executes cnoid-xacro without a shell so filenames are passed as exact arguments.
//
// A single popen() pipe is not enough here. stdout contains the expanded URDF XML, while
// stderr may contain warnings or errors from xacro. Merging stderr into stdout would corrupt
// the XML stream even when xacro succeeds with only a warning, and reading stdout alone would
// lose the real error message and exit status. This uses two pipes so stdout can be parsed as
// XML and stderr can be shown only as diagnostics.
//
// The two pipes must also be drained in parallel. If the parent reads only one pipe while the
// child writes enough data to fill the other pipe buffer, the child can block before exiting.
// The select() loop below waits for whichever pipe is ready, appends that chunk, and closes
// each pipe on EOF before waitpid() collects the final exit status.
XacroResult executeXacro(
    const string& xacroCommand, const string& filename,
    const vector<string>& localPackageSearchDirectories)
{
    XacroResult result;
    int stdoutPipe[2];
    int stderrPipe[2];
    if (pipe(stdoutPipe) != 0) {
        result.systemError = strerror(errno);
        return result;
    }
    if (pipe(stderrPipe) != 0) {
        result.systemError = strerror(errno);
        close(stdoutPipe[0]);
        close(stdoutPipe[1]);
        return result;
    }

    pid_t pid = fork();
    if (pid < 0) {
        result.systemError = strerror(errno);
        close(stdoutPipe[0]);
        close(stdoutPipe[1]);
        close(stderrPipe[0]);
        close(stderrPipe[1]);
        return result;
    }

    if (pid == 0) {
        // Child process: connect stdout/stderr to the pipe write ends, set the package
        // search path only in this process, and replace this process with cnoid-xacro.
        close(stdoutPipe[0]);
        close(stderrPipe[0]);
        dup2(stdoutPipe[1], STDOUT_FILENO);
        dup2(stderrPipe[1], STDERR_FILENO);
        close(stdoutPipe[1]);
        close(stderrPipe[1]);

        string rosPackagePath = makeRosPackagePath(localPackageSearchDirectories);
        if (!rosPackagePath.empty()) {
            setenv("ROS_PACKAGE_PATH", rosPackagePath.c_str(), 1);
        }

        execl(xacroCommand.c_str(), xacroCommand.c_str(), filename.c_str(), (char*) nullptr);

        string message = formatC(
            "Failed to execute \"{0}\": {1}\n", xacroCommand, strerror(errno));
        ssize_t n = write(STDERR_FILENO, message.c_str(), message.size());
        (void)n;
        _exit(127);
    }

    // Parent process: close the write ends and read the child's stdout and stderr from the
    // read ends below. Closing the write ends here is also needed for reliable EOF detection.
    result.started = true;
    close(stdoutPipe[1]);
    close(stderrPipe[1]);

    bool stdoutOpen = true;
    bool stderrOpen = true;
    while (stdoutOpen || stderrOpen) {
        // Wait until at least one pipe can be read. Reading both pipes through select()
        // prevents the child from blocking if either stdout or stderr fills its pipe buffer.
        fd_set readSet;
        FD_ZERO(&readSet);
        int maxFd = -1;
        if (stdoutOpen) {
            FD_SET(stdoutPipe[0], &readSet);
            maxFd = std::max(maxFd, stdoutPipe[0]);
        }
        if (stderrOpen) {
            FD_SET(stderrPipe[0], &readSet);
            maxFd = std::max(maxFd, stderrPipe[0]);
        }

        int selected = select(maxFd + 1, &readSet, nullptr, nullptr, nullptr);
        if (selected < 0) {
            if (errno == EINTR) {
                continue;
            }
            result.systemError = strerror(errno);
            if (stdoutOpen) {
                close(stdoutPipe[0]);
                stdoutOpen = false;
            }
            if (stderrOpen) {
                close(stderrPipe[0]);
                stderrOpen = false;
            }
            break;
        }

        if (stdoutOpen && FD_ISSET(stdoutPipe[0], &readSet)) {
            appendPipeOutput(stdoutPipe[0], result.output, stdoutOpen);
        }
        if (stderrOpen && FD_ISSET(stderrPipe[0], &readSet)) {
            appendPipeOutput(stderrPipe[0], result.errorOutput, stderrOpen);
        }
    }

    int status;
    while (waitpid(pid, &status, 0) < 0) {
        if (errno != EINTR) {
            result.systemError = strerror(errno);
            return result;
        }
    }
    if (WIFEXITED(status)) {
        result.exited = true;
        result.exitCode = WEXITSTATUS(status);
    } else if (WIFSIGNALED(status)) {
        result.signal = WTERMSIG(status);
    }

    return result;
}

#else // _WIN32

// Converts a UTF-8 string to UTF-16 for the wide-character Win32 process and environment APIs.
std::wstring utf8ToWide(const string& text)
{
    if (text.empty()) {
        return std::wstring();
    }
    int length = MultiByteToWideChar(
        CP_UTF8, 0, text.data(), static_cast<int>(text.size()), nullptr, 0);
    std::wstring result(length, L'\0');
    MultiByteToWideChar(
        CP_UTF8, 0, text.data(), static_cast<int>(text.size()), &result[0], length);
    return result;
}

// Quotes a single argument so the child's CRT argv parser recovers it verbatim, following the
// rules in the CommandLineToArgvW documentation. cnoid-xacro is run without a shell, so spaces
// and quotes in the file name must be preserved exactly.
std::wstring quoteArgument(const std::wstring& argument)
{
    if (!argument.empty()
        && argument.find_first_of(L" \t\n\v\"") == std::wstring::npos) {
        return argument;
    }
    std::wstring quoted = L"\"";
    for (size_t i = 0;; ++i) {
        unsigned int backslashes = 0;
        while (i < argument.size() && argument[i] == L'\\') {
            ++i;
            ++backslashes;
        }
        if (i == argument.size()) {
            // Escape all backslashes before the closing quote.
            quoted.append(backslashes * 2, L'\\');
            break;
        } else if (argument[i] == L'"') {
            // Escape all backslashes and the following quote.
            quoted.append(backslashes * 2 + 1, L'\\');
            quoted.push_back(argument[i]);
        } else {
            quoted.append(backslashes, L'\\');
            quoted.push_back(argument[i]);
        }
    }
    quoted.push_back(L'"');
    return quoted;
}

// Case-insensitive comparison of an environment variable name. Windows environment variable
// names are case-insensitive, so existing PYTHONPATH/ROS_PACKAGE_PATH entries must be matched
// regardless of case before they are overridden.
bool equalsEnvName(const std::wstring& name, const wchar_t* expected)
{
    size_t i = 0;
    for (; i < name.size() && expected[i] != L'\0'; ++i) {
        if (towupper(name[i]) != towupper(expected[i])) {
            return false;
        }
    }
    return i == name.size() && expected[i] == L'\0';
}

// Builds the environment block for the xacro child process. The current environment is copied
// so the child inherits the user's settings, then PYTHONPATH is augmented so the bundled
// cnoid.xacro module can be imported and ROS_PACKAGE_PATH is set for $(find pkg) resolution.
// Both variables are changed only for the child, never for the Choreonoid process.
std::vector<wchar_t> buildEnvironmentBlock(
    const string& pythonModuleDir, const vector<string>& localPackageSearchDirectories)
{
    vector<std::wstring> entries;

    if (LPWCH envStrings = GetEnvironmentStringsW()) {
        for (LPWCH p = envStrings; *p != L'\0';) {
            std::wstring entry(p);
            p += entry.size() + 1;
            size_t eq = entry.find(L'=');
            std::wstring name = (eq == std::wstring::npos) ? entry : entry.substr(0, eq);
            // Skip the variables overridden below; they are re-added with the augmented values.
            if (equalsEnvName(name, L"PYTHONPATH")
                || equalsEnvName(name, L"ROS_PACKAGE_PATH")
                || equalsEnvName(name, L"PYTHONIOENCODING")) {
                continue;
            }
            entries.push_back(std::move(entry));
        }
        FreeEnvironmentStringsW(envStrings);
    }

    // Force the child interpreter to emit UTF-8 so the captured stdout is decodable URDF XML
    // regardless of the console code page, matching the UTF-8 output produced on POSIX.
    entries.push_back(L"PYTHONIOENCODING=utf-8");

    // Expose the bundled cnoid.xacro module to the child interpreter. On Windows the embedded
    // interpreter does not export PYTHONPATH to the process environment, so the module path is
    // added here explicitly, ahead of any inherited PYTHONPATH.
    std::wstring pythonPath = utf8ToWide(pythonModuleDir);
    const char* existingPythonPath = getenv("PYTHONPATH");
    if (existingPythonPath && *existingPythonPath) {
        if (!pythonPath.empty()) {
            pythonPath += L';';
        }
        pythonPath += utf8ToWide(existingPythonPath);
    }
    entries.push_back(L"PYTHONPATH=" + pythonPath);

    string rosPackagePath = makeRosPackagePath(localPackageSearchDirectories);
    if (!rosPackagePath.empty()) {
        entries.push_back(L"ROS_PACKAGE_PATH=" + utf8ToWide(rosPackagePath));
    }

    vector<wchar_t> block;
    for (const auto& entry : entries) {
        block.insert(block.end(), entry.begin(), entry.end());
        block.push_back(L'\0');
    }
    block.push_back(L'\0'); // terminating null of the environment block
    return block;
}

// Drains a pipe until the write end is closed. Each pipe is read on its own thread so the
// child cannot block when either stdout or stderr fills its pipe buffer.
void readPipe(HANDLE pipe, string& out)
{
    char buffer[4096];
    DWORD bytesRead = 0;
    while (ReadFile(pipe, buffer, sizeof(buffer), &bytesRead, nullptr) && bytesRead > 0) {
        out.append(buffer, bytesRead);
    }
}

// Executes cnoid-xacro through the Python interpreter and captures its stdout and stderr.
//
// cnoid-xacro is a Python script without an executable extension, so it cannot be started
// directly on Windows; it is run as "<python> cnoid-xacro <file>". As on POSIX, stdout (the
// expanded URDF XML) and stderr (xacro diagnostics) are captured through separate pipes so the
// XML stream is never corrupted by warnings, and both pipes are drained concurrently so the
// child cannot block when either pipe buffer fills.
XacroResult executeXacro(
    const string& pythonExecutable, const string& scriptPath, const string& filename,
    const string& pythonModuleDir, const vector<string>& localPackageSearchDirectories)
{
    XacroResult result;

    SECURITY_ATTRIBUTES securityAttributes;
    securityAttributes.nLength = sizeof(securityAttributes);
    securityAttributes.bInheritHandle = TRUE;
    securityAttributes.lpSecurityDescriptor = nullptr;

    HANDLE stdoutRead = nullptr;
    HANDLE stdoutWrite = nullptr;
    if (!CreatePipe(&stdoutRead, &stdoutWrite, &securityAttributes, 0)) {
        result.systemError = "failed to create the stdout pipe";
        return result;
    }
    // The read ends stay in this process and must not be inherited by the child.
    SetHandleInformation(stdoutRead, HANDLE_FLAG_INHERIT, 0);

    HANDLE stderrRead = nullptr;
    HANDLE stderrWrite = nullptr;
    if (!CreatePipe(&stderrRead, &stderrWrite, &securityAttributes, 0)) {
        result.systemError = "failed to create the stderr pipe";
        CloseHandle(stdoutRead);
        CloseHandle(stdoutWrite);
        return result;
    }
    SetHandleInformation(stderrRead, HANDLE_FLAG_INHERIT, 0);

    // The child reads its input file directly, so connect its stdin to the null device.
    HANDLE stdinRead = CreateFileW(
        L"NUL", GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE,
        &securityAttributes, OPEN_EXISTING, 0, nullptr);

    STARTUPINFOW startupInfo;
    ZeroMemory(&startupInfo, sizeof(startupInfo));
    startupInfo.cb = sizeof(startupInfo);
    startupInfo.dwFlags = STARTF_USESTDHANDLES;
    startupInfo.hStdInput = (stdinRead != INVALID_HANDLE_VALUE) ? stdinRead : nullptr;
    startupInfo.hStdOutput = stdoutWrite;
    startupInfo.hStdError = stderrWrite;

    std::wstring commandLine = quoteArgument(utf8ToWide(pythonExecutable));
    commandLine += L' ';
    commandLine += quoteArgument(utf8ToWide(scriptPath));
    commandLine += L' ';
    commandLine += quoteArgument(utf8ToWide(filename));
    vector<wchar_t> commandLineBuffer(commandLine.begin(), commandLine.end());
    commandLineBuffer.push_back(L'\0'); // CreateProcessW requires a writable, null-terminated buffer

    vector<wchar_t> environmentBlock =
        buildEnvironmentBlock(pythonModuleDir, localPackageSearchDirectories);

    PROCESS_INFORMATION processInfo;
    ZeroMemory(&processInfo, sizeof(processInfo));

    BOOL created = CreateProcessW(
        nullptr,
        commandLineBuffer.data(),
        nullptr,
        nullptr,
        TRUE, // inherit the pipe write ends and the null stdin handle
        CREATE_NO_WINDOW | CREATE_UNICODE_ENVIRONMENT,
        environmentBlock.data(),
        nullptr,
        &startupInfo,
        &processInfo);

    // The parent does not write to the child, so close the inherited write ends and the null
    // stdin handle now. Closing the write ends is also required for the read ends to report EOF.
    CloseHandle(stdoutWrite);
    CloseHandle(stderrWrite);
    if (stdinRead != INVALID_HANDLE_VALUE) {
        CloseHandle(stdinRead);
    }

    if (!created) {
        result.systemError = formatC(
            "failed to execute \"{0}\" (error code {1})",
            pythonExecutable, static_cast<unsigned int>(GetLastError()));
        CloseHandle(stdoutRead);
        CloseHandle(stderrRead);
        return result;
    }
    result.started = true;

    // Drain stderr on a separate thread while stdout is read here so a full pipe buffer on
    // either stream cannot block the child before it exits.
    std::thread stderrReader([stderrRead, &result]() {
        readPipe(stderrRead, result.errorOutput);
    });
    readPipe(stdoutRead, result.output);
    stderrReader.join();

    CloseHandle(stdoutRead);
    CloseHandle(stderrRead);

    WaitForSingleObject(processInfo.hProcess, INFINITE);
    DWORD exitCode = 0;
    if (GetExitCodeProcess(processInfo.hProcess, &exitCode)) {
        result.exited = true;
        result.exitCode = static_cast<int>(exitCode);
    }
    CloseHandle(processInfo.hProcess);
    CloseHandle(processInfo.hThread);

    return result;
}

#endif // _WIN32

} // namespace

// Finds the nearest ROS package that owns the input file and adds its parent directory as a
// local ROS_PACKAGE_PATH entry.
vector<string> cnoid::findLocalPackageSearchDirectories(const string& filename)
{
    vector<string> directories;

    filesystem::path path = filesystem::absolute(fromUTF8(filename)).lexically_normal();
    filesystem::path dir = path.parent_path();
    while (!dir.empty()) {
        if (filesystem::exists(dir / "package.xml")) {
            addDirectoryIfUnique(directories, dir.parent_path());
            break;
        }
        filesystem::path parent = dir.parent_path();
        if (parent == dir) {
            break;
        }
        dir = parent;
    }

    return directories;
}


bool cnoid::expandXacro(
    const string& filename,
    const string& executableDir,
    const vector<string>& localPackageSearchDirectories,
    std::ostream& os,
    string& out_urdfContent,
    string& out_errorOutput)
{
    XacroResult xacroResult;

#ifdef _WIN32
    // cnoid-xacro is a Python script, so it is run through "python" resolved from PATH by
    // CreateProcess. This mirrors how the POSIX path relies on the cnoid-xacro shebang resolving
    // python3 from PATH, and keeps the installed binaries relocatable. The bundled cnoid.xacro
    // module is exposed to that interpreter through PYTHONPATH in the child's environment (see
    // buildEnvironmentBlock).
    string pythonExecutable = "python";
    string scriptPath = formatC("{0}/cnoid-xacro", executableDir);
    string pythonModuleDir = toUTF8((pluginDirPath() / "python").generic_string());
    xacroResult = executeXacro(
        pythonExecutable, scriptPath, filename, pythonModuleDir,
        localPackageSearchDirectories);
#else
    string xacroCommand = formatC("{0}/cnoid-xacro", executableDir);
    xacroResult = executeXacro(xacroCommand, filename, localPackageSearchDirectories);
#endif

    if (!xacroResult.started) {
        os << "Error: failed to start xacro parser";
        if (!xacroResult.systemError.empty()) {
            os << ": " << xacroResult.systemError;
        }
        os << endl;
        return false;
    }
    if (!xacroResult.systemError.empty()) {
        os << "Error: xacro parser failed: " << xacroResult.systemError << endl;
        return false;
    }
    out_errorOutput = xacroResult.errorOutput;
    if (!xacroResult.exited || xacroResult.exitCode != 0) {
        os << "Error: xacro parser failed";
        if (xacroResult.exited) {
            os << " with exit code " << xacroResult.exitCode;
        } else if (xacroResult.signal != 0) {
            os << " by signal " << xacroResult.signal;
        }
        os << "." << endl;
        if (!xacroResult.errorOutput.empty()) {
            os << xacroResult.errorOutput;
            if (xacroResult.errorOutput.back() != '\n') {
                os << endl;
            }
        }
        return false;
    }

    out_urdfContent = xacroResult.output;
    return true;
}
