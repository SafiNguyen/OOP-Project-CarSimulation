#ifndef MAP_FILE_DIALOG_H
#define MAP_FILE_DIALOG_H

#include <string>

// Opens a native "pick a file" dialog (GetOpenFileNameW on Windows,
// zenity on Linux). Returns true and fills selectedPath if the user
// picked a file; returns false if the dialog is unavailable, was
// cancelled, or failed.
bool openMapFileDialog(std::string& selectedPath);

#endif
