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
	extern bool loadSingleSpringModel;
	extern bool loadSpringChainModel;
	extern bool loadCloth;
	extern bool loadJellyCubeModel;
	extern bool loadTableCloth;
	extern bool loadFlag;

	// reset
	extern bool resetView;

	void updateMenu();

} // namespace panel
