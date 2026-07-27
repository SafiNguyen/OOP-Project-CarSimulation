#ifndef UI_THEME_H
#define UI_THEME_H

#include <imgui.h>

namespace UiTheme {

extern const ImVec4 Background;
extern const ImVec4 Surface;
extern const ImVec4 SurfaceRaised;
extern const ImVec4 Border;
extern const ImVec4 Text;
extern const ImVec4 TextMuted;
extern const ImVec4 Accent;
extern const ImVec4 AccentStrong;
extern const ImVec4 Success;
extern const ImVec4 Warning;
extern const ImVec4 Error;

void apply();
void tooltip(const char* text);

bool actionButton(const char* label, const ImVec2& size = ImVec2(0.0f, 0.0f));
bool selectionButton(const char* label, bool selected,
                     const ImVec2& size = ImVec2(0.0f, 0.0f));
bool toggleButton(const char* id, const char* label, bool enabled,
                  const ImVec2& size = ImVec2(0.0f, 0.0f));

} // namespace UiTheme

#endif
