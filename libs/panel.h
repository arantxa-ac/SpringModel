#pragma once

#include <iosfwd>
#include <string>

#include "givio.h"
#include "givr.h"
#include "imgui/imgui.h"

namespace panel {

extern bool showPanel;
extern ImVec4 clear_color;

// animation
extern bool playModel;
extern bool resetModel;
extern bool stepModel;
extern float dt;
extern bool loadMassSpringModel;
extern bool loadChainPendulumModel;
extern bool loadClothModel;
extern bool loadCubeModel;

// reset
extern bool resetView;

void updateMenu();

} // namespace panel
