#include "myimgui.h"


namespace physicalEngine
{
	ImVec4 EngineGUI::clear_color = ImVec4(50 / 255.0, 50 / 255.0, 50 / 255.0, 0.8f);
	ImGuiIO* EngineGUI::ioStatic = nullptr;
	bool EngineGUI::show_bvhTree = false;
	bool EngineGUI::show_contacts = false;
	bool EngineGUI::resetScene  =false;
	bool EngineGUI::pauseWorld = false;

	int EngineGUI::scene_id = 0;
	ImVec2 EngineGUI:: gravity = ImVec2(0.f, -9.8f);
	int EngineGUI:: simulate_speed = 6;
	int EngineGUI::velocityInterationNumPreFrame = 8;
	int EngineGUI::positionInterationNumPreFrame = 6;
	int EngineGUI::sceneNum = 9;
	const char* EngineGUI:: items[] = { "scene-fallDown", 
										"scene-restitution",  
										"scene-friction" , 
										"scene-pyramid",
										"scene-Newton's pendulum", 
										"scene-bridge", 
										"scene-wreckingBall", 
										"scene-DominoScene", 
										"scene-seesawScene" 
										}; //"scene-dowblePendulum"

	int EngineGUI::pyramidLayerNum = 12;
	float EngineGUI::pyramidrestitution = 0.f;
	float EngineGUI::pyramidBoxSize = 3.5f;

	//Å£¶Ù°Ú³¡¾°
	int EngineGUI::newtonPendunlumCircleNum = 7;
	float EngineGUI::newtonPendunlumCircleRadius = 2.5f;
	float EngineGUI::newtonPendunlumLineLength = 22.f;
	float EngineGUI::newtonPendunlumRestitution = 1.0f;;

	//ËÙ¶È»Ö¸´³¡¾°
	int EngineGUI::restitutionCircleNum = 10.f;
	float EngineGUI::restitutionCircleRadius = 2.2f;

	//ÇÅ³¡¾°
	int EngineGUI::BridgejointNum = 13;
	float EngineGUI::BridgejointWidth = 2.2f;

	int EngineGUI::BridgeboxLayerNum = 3;
	int EngineGUI::BridgeboxNumPreLayer = 3;
	float EngineGUI::BridgeboxSize = 2.2f;

}