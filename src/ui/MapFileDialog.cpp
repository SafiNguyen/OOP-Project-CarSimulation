#include "MapFileDialog.h"

#include <array>
#include <cstdio>

bool openMapFileDialog(std::string& selectedPath) {
#if defined(__linux__)
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
