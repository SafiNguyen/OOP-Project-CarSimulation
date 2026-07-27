#include "UiTheme.h"

#include <string>

namespace UiTheme {

const ImVec4 Background(0.055f, 0.090f, 0.120f, 0.98f);
const ImVec4 Surface(0.075f, 0.125f, 0.160f, 0.98f);
const ImVec4 SurfaceRaised(0.105f, 0.170f, 0.215f, 0.99f);
const ImVec4 Border(0.250f, 0.390f, 0.470f, 0.95f);
const ImVec4 Text(0.950f, 0.970f, 0.985f, 1.0f);
const ImVec4 TextMuted(0.670f, 0.750f, 0.800f, 1.0f);
const ImVec4 Accent(0.100f, 0.680f, 0.830f, 1.0f);
const ImVec4 AccentStrong(0.160f, 0.780f, 0.920f, 1.0f);
const ImVec4 Success(0.270f, 0.820f, 0.580f, 1.0f);
const ImVec4 Warning(0.980f, 0.690f, 0.280f, 1.0f);
const ImVec4 Error(0.960f, 0.400f, 0.420f, 1.0f);

void apply() {
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowPadding = ImVec2(15.0f, 13.0f);
    style.FramePadding = ImVec2(11.0f, 8.0f);
    style.CellPadding = ImVec2(9.0f, 7.0f);
    style.ItemSpacing = ImVec2(9.0f, 9.0f);
    style.ItemInnerSpacing = ImVec2(6.0f, 5.0f);
    style.ScrollbarSize = 12.0f;
    style.GrabMinSize = 10.0f;
    style.WindowBorderSize = 1.0f;
    style.ChildBorderSize = 1.0f;
    style.PopupBorderSize = 1.0f;
    style.FrameBorderSize = 0.0f;
    style.TabBorderSize = 0.0f;
    style.WindowRounding = 8.0f;
    style.ChildRounding = 7.0f;
    style.FrameRounding = 6.0f;
    style.PopupRounding = 7.0f;
    style.ScrollbarRounding = 8.0f;
    style.GrabRounding = 5.0f;
    style.TabRounding = 6.0f;

    ImVec4* colors = style.Colors;
    colors[ImGuiCol_Text] = Text;
    colors[ImGuiCol_TextDisabled] = TextMuted;
    colors[ImGuiCol_WindowBg] = Background;
    colors[ImGuiCol_ChildBg] = Surface;
    colors[ImGuiCol_PopupBg] = SurfaceRaised;
    colors[ImGuiCol_Border] = Border;
    colors[ImGuiCol_BorderShadow] = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
    colors[ImGuiCol_FrameBg] = ImVec4(0.100f, 0.160f, 0.200f, 1.0f);
    colors[ImGuiCol_FrameBgHovered] = ImVec4(0.140f, 0.230f, 0.280f, 1.0f);
    colors[ImGuiCol_FrameBgActive] = ImVec4(0.150f, 0.290f, 0.350f, 1.0f);
    colors[ImGuiCol_TitleBg] = Surface;
    colors[ImGuiCol_TitleBgActive] = SurfaceRaised;
    colors[ImGuiCol_TitleBgCollapsed] = Surface;
    colors[ImGuiCol_MenuBarBg] = Surface;
    colors[ImGuiCol_ScrollbarBg] = ImVec4(0.045f, 0.075f, 0.100f, 0.85f);
    colors[ImGuiCol_ScrollbarGrab] = ImVec4(0.260f, 0.410f, 0.490f, 1.0f);
    colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.320f, 0.520f, 0.610f, 1.0f);
    colors[ImGuiCol_ScrollbarGrabActive] = Accent;
    colors[ImGuiCol_CheckMark] = AccentStrong;
    colors[ImGuiCol_SliderGrab] = Accent;
    colors[ImGuiCol_SliderGrabActive] = AccentStrong;
    colors[ImGuiCol_Button] = ImVec4(0.120f, 0.190f, 0.240f, 1.0f);
    colors[ImGuiCol_ButtonHovered] = ImVec4(0.160f, 0.290f, 0.350f, 1.0f);
    colors[ImGuiCol_ButtonActive] = ImVec4(0.100f, 0.410f, 0.500f, 1.0f);
    colors[ImGuiCol_Header] = ImVec4(0.115f, 0.205f, 0.255f, 1.0f);
    colors[ImGuiCol_HeaderHovered] = ImVec4(0.145f, 0.315f, 0.370f, 1.0f);
    colors[ImGuiCol_HeaderActive] = ImVec4(0.110f, 0.450f, 0.535f, 1.0f);
    colors[ImGuiCol_Separator] = Border;
    colors[ImGuiCol_SeparatorHovered] = Accent;
    colors[ImGuiCol_SeparatorActive] = AccentStrong;
    colors[ImGuiCol_ResizeGrip] = ImVec4(0.12f, 0.28f, 0.34f, 0.35f);
    colors[ImGuiCol_ResizeGripHovered] = Accent;
    colors[ImGuiCol_ResizeGripActive] = AccentStrong;
    colors[ImGuiCol_Tab] = ImVec4(0.085f, 0.145f, 0.185f, 1.0f);
    colors[ImGuiCol_TabHovered] = ImVec4(0.140f, 0.340f, 0.405f, 1.0f);
    colors[ImGuiCol_TabActive] = ImVec4(0.100f, 0.440f, 0.525f, 1.0f);
    colors[ImGuiCol_TabUnfocused] = colors[ImGuiCol_Tab];
    colors[ImGuiCol_TabUnfocusedActive] = ImVec4(0.105f, 0.285f, 0.335f, 1.0f);
    colors[ImGuiCol_TableHeaderBg] = SurfaceRaised;
    colors[ImGuiCol_TableBorderStrong] = Border;
    colors[ImGuiCol_TableBorderLight] = ImVec4(0.18f, 0.29f, 0.35f, 1.0f);
    colors[ImGuiCol_TableRowBgAlt] = ImVec4(0.10f, 0.17f, 0.21f, 0.75f);
    colors[ImGuiCol_TextSelectedBg] = ImVec4(0.09f, 0.55f, 0.68f, 0.45f);
    colors[ImGuiCol_NavHighlight] = AccentStrong;
}

void tooltip(const char* text) {
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort |
                             ImGuiHoveredFlags_AllowWhenDisabled)) {
        ImGui::BeginTooltip();
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 28.0f);
        ImGui::TextUnformatted(text);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

bool actionButton(const char* label, const ImVec2& size) {
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.070f, 0.410f, 0.500f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.090f, 0.540f, 0.640f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.060f, 0.335f, 0.415f, 1.0f));
    const bool clicked = ImGui::Button(label, size);
    ImGui::PopStyleColor(3);
    return clicked;
}

bool selectionButton(const char* label, bool selected, const ImVec2& size) {
    if (selected) {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.080f, 0.540f, 0.645f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.110f, 0.650f, 0.750f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.070f, 0.440f, 0.535f, 1.0f));
    }
    const bool clicked = ImGui::Button(label, size);
    if (selected) {
        ImGui::PopStyleColor(3);
    }
    return clicked;
}

bool toggleButton(const char* id, const char* label, bool enabled, const ImVec2& size) {
    ImGui::PushID(id);
    const ImVec4 base = enabled
        ? ImVec4(0.080f, 0.500f, 0.405f, 1.0f)
        : ImVec4(0.115f, 0.175f, 0.215f, 1.0f);
    const ImVec4 hovered = enabled
        ? ImVec4(0.105f, 0.620f, 0.500f, 1.0f)
        : ImVec4(0.155f, 0.245f, 0.290f, 1.0f);
    ImGui::PushStyleColor(ImGuiCol_Button, base);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, hovered);
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.070f, 0.400f, 0.335f, 1.0f));
    const std::string text = std::string(label) + (enabled ? "  ON" : "  OFF");
    const bool clicked = ImGui::Button(text.c_str(), size);
    ImGui::PopStyleColor(3);
    ImGui::PopID();
    return clicked;
}

} // namespace UiTheme
