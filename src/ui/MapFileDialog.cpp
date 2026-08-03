#include "MapFileDialog.h"

#include <array>
#include <cstdio>
#include <string>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <commdlg.h>
#endif

namespace {

#if defined(_WIN32)
// Convert a UTF-16 Windows path to a UTF-8 std::string.
std::string wideToUtf8(const std::wstring& wide) {
    if (wide.empty()) {
        return std::string();
    }
    const int size = WideCharToMultiByte(
        CP_UTF8, 0, wide.data(), static_cast<int>(wide.size()),
        nullptr, 0, nullptr, nullptr);
    if (size <= 0) {
        return std::string();
    }
    std::string result(static_cast<std::size_t>(size), '\0');
    WideCharToMultiByte(
        CP_UTF8, 0, wide.data(), static_cast<int>(wide.size()),
        &result[0], size, nullptr, nullptr);
    return result;
}
#endif

} // namespace

bool openMapFileDialog(std::string& selectedPath) {
#if defined(_WIN32)
    // Native Windows file picker (works with both MSVC and MinGW).
    OPENFILENAMEW ofn{};
    std::wstring fileName(32768u, L'\0');

    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = nullptr;
    ofn.lpstrFilter = L"Map JSON files (*.json)\0*.json\0All files (*.*)\0*.*\0";
    ofn.lpstrFile = fileName.data();
    ofn.nMaxFile = static_cast<DWORD>(fileName.size());
    ofn.lpstrTitle = L"Select a map JSON file";
    ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_HIDEREADONLY;

    if (GetOpenFileNameW(&ofn) == FALSE) {
        return false; // cancelled or failed
    }

    const std::wstring selected(fileName.data());
    if (selected.empty()) {
        return false;
    }
    selectedPath = wideToUtf8(selected);
    return !selectedPath.empty();
#elif defined(__linux__)
    const std::string command = "zenity --file-selection --title='Select map JSON' 2>/dev/null";
    std::array<char, 2048> buffer{};
    FILE* pipe = popen(command.c_str(), "r");
    if (pipe == nullptr) {
        return false;
    }

    std::string result;
    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr) {
        result += buffer.data();
    }

    const int status = pclose(pipe);
    if (status != 0) {
        return false;
    }

    while (!result.empty() && (result.back() == '\n' || result.back() == '\r')) {
        result.pop_back();
    }

    if (!result.empty()) {
        selectedPath = result;
        return true;
    }
#else
    (void)selectedPath;
#endif
    return false;
}