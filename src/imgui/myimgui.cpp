#include "myimgui.h"


namespace physicalEngine
{
	ImVec4 EngineGUI::clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
	ImGuiIO* EngineGUI::ioStatic = nullptr;
	bool EngineGUI::show_bvhTree = false;
	bool EngineGUI::show_contacts = true;
	int EngineGUI::scene_id = 0;

	int EngineGUI:: simulate_speed = 3;
}