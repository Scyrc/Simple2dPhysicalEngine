#pragma once
#include <stdio.h>
#include "../../include/body/world.h"
#include "../../include/imgui/imgui.h"
#include "../../include/imgui/imgui_impl_glfw.h"
#include "../../include/imgui/imgui_impl_opengl3.h"

namespace physicalEngine
{

	static class EngineGUI
	{
	private:
		static ImGuiIO* ioStatic;
		static ImVec4 clear_color;
		static bool show_bvhTree;
		static bool show_contacts;
		static bool resetScene;
		static bool pauseWorld;

		static int simulate_speed;
		static int scene_id;
		static int velocityInterationNumPreFrame;
		static int positionInterationNumPreFrame;
		static ImVec2 gravity;
		static const char* items[];
		static int sceneNum;

		static int pyramidLayerNum;
		static float pyramidrestitution;
		static float pyramidBoxSize;

		//Å£¶Ù°Ú³¡¾°
		static int newtonPendunlumCircleNum;
		static float newtonPendunlumCircleRadius;
		static float newtonPendunlumLineLength;
		static float newtonPendunlumRestitution;

		//ËÙ¶È»Ö¸´³¡¾°
		static int restitutionCircleNum;
		static float restitutionCircleRadius;

		//ÇÅ³¡¾°
		static int BridgejointNum;
		static float BridgejointWidth;

		static int BridgeboxLayerNum;
		static int BridgeboxNumPreLayer;
		static float BridgeboxSize;

	public:
		static inline void Setup(GLFWwindow* window)
		{
			const char* glsl_version = "#version 130";

			// Setup Dear ImGui context
			IMGUI_CHECKVERSION();
			ImGui::CreateContext();
			ImGuiIO& io = ImGui::GetIO();
			(void)io;
			io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
			io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls


			ioStatic = &io;
			// Setup Dear ImGui style
			ImGui::StyleColorsDark();
			ImGui::StyleColorsLight();
			ImGui::StyleColorsClassic();

			// Setup Platform/Renderer back-ends
			ImGui_ImplGlfw_InitForOpenGL(window, true);
			ImGui_ImplOpenGL3_Init(glsl_version);
		}

		static inline void  RenderImGui(World& staticWorld)
		{
			// Start the Dear ImGui frame
			ImGui_ImplOpenGL3_NewFrame();
			ImGui_ImplGlfw_NewFrame();
			ImGui::NewFrame();
			{
				ImGui::Begin("World Settings", 0);                          // Create a window called "Hello, world!" and append into it.

				//ImGui::Begin("Hello 2D Engine!", 0, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);                          // Create a window called "Hello, world!" and append into it.
				
				ImGui::SetNextItemWidth(200);
				ImGui::Combo("selected", &scene_id, items, sceneNum, 200);

				resetScene = false;
				if (ImGui::Button("reset scene"))
					resetScene = true;
				ImGui::Checkbox("pause world", &pauseWorld);
				ImGui::SameLine();
				ImGui::Checkbox("show bvhTree", &show_bvhTree);    
				ImGui::SameLine();
				ImGui::Checkbox("show contacts", &show_contacts);
				
				ImGui::PushItemWidth(150); // ÉèÖÃ¿í¶ÈÎª100ÏñËØ
				ImGui::InputFloat2("gravity", (float*)&gravity);
				ImGui::PopItemWidth(); 
				ImGui::PushItemWidth(200); // »Ö¸´Ä¬ÈÏ¿í¶È
				ImGui::SliderInt("simulate speed", &simulate_speed, 0, 20);
				ImGui::SliderInt("velInterationNum", &velocityInterationNumPreFrame, 1, 20);
				ImGui::SliderInt("posInterationNum", &positionInterationNumPreFrame, 1, 20);
				ImGui::ColorEdit3("background color", (float*)&clear_color); // Edit 3 floats representing a color
				ImGui::PopItemWidth(); 
				
				ImGui::Text("Body Num = %d", staticWorld.getBodyNums());

				ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ioStatic->Framerate, ioStatic->Framerate);

				ImGui::End();
			}
			if (scene_id == 1)
			{
				ImGui::Begin("restitution scene settings", 0);
				ImGui::PushItemWidth(200); // »Ö¸´Ä¬ÈÏ¿í¶È
				ImGui::SliderInt("circle num", &restitutionCircleNum, 6, 15);
				ImGui::SliderFloat("circle radius", &restitutionCircleRadius, 1.0f, 5.0f);
				ImGui::End();
			}
			else if(scene_id == 3)
			{
				ImGui::Begin("pyramid scene settings", 0);
				ImGui::PushItemWidth(200); // »Ö¸´Ä¬ÈÏ¿í¶È
				ImGui::SliderInt("layer num", &pyramidLayerNum, 3, 20);
				ImGui::SliderFloat("box size", &pyramidBoxSize, 2.0f, 5.0f);
				ImGui::SliderFloat("restitution", &pyramidrestitution, 0.0f, 1.0f);
				ImGui::End();
			}
			else if (scene_id == 4)
			{
				ImGui::Begin("newton's pendulum scene settings", 0);
				ImGui::PushItemWidth(200); // »Ö¸´Ä¬ÈÏ¿í¶È
				ImGui::SliderInt("circle num", &newtonPendunlumCircleNum, 5, 10);
				ImGui::SliderFloat("circle radius", &newtonPendunlumCircleRadius, 1.0f, 5.0f);
				ImGui::SliderFloat("line length", &newtonPendunlumLineLength, 10.f, 30.f);
				ImGui::SliderFloat("restitution", &newtonPendunlumRestitution, 0.f, 1.f);

				ImGui::End();
			}
			else if (scene_id == 5)
			{
				ImGui::Begin("bridge scene settings", 0);

				ImGui::PushItemWidth(200); // »Ö¸´Ä¬ÈÏ¿í¶È
				ImGui::SliderInt("joint num", &BridgejointNum, 10, 30);
				ImGui::SliderFloat("joint width", &BridgejointWidth, 1.0f, 2.5f);
				ImGui::SliderInt("box layer num", &BridgeboxLayerNum, 1, 6);
				ImGui::SliderInt("box num pre layer", &BridgeboxNumPreLayer, 1, 6);
				ImGui::SliderFloat("box size", &BridgeboxSize, 1.0f, 3.0f);

				ImGui::End();
			}
			ImGui::Render();
			ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

			staticWorld.setInterationNum(velocityInterationNumPreFrame, positionInterationNumPreFrame);
			if (resetScene)
			{
				staticWorld.resetScene(scene_id);
			}
			else
			{
				staticWorld.changeScene(scene_id);
			}
			staticWorld.setBackGround(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
			staticWorld.setDispalyProperties(show_bvhTree, show_contacts);
			staticWorld.setPause(pauseWorld);
			staticWorld.setSimluateSpeed(simulate_speed);
			staticWorld.setGravity(gravity.x, gravity.y);
			staticWorld.setpyramidParam(pyramidLayerNum, pyramidrestitution, pyramidBoxSize);
			staticWorld.setnewtonPendunlumParam(newtonPendunlumCircleNum, newtonPendunlumCircleRadius, newtonPendunlumLineLength, newtonPendunlumRestitution);
			staticWorld.setrestitutionParam(restitutionCircleNum, restitutionCircleRadius);
			staticWorld.setBridgeParam(BridgejointNum, BridgejointWidth, BridgeboxLayerNum, BridgeboxNumPreLayer, BridgeboxSize);

		}

		static inline void Clearup()
		{
			ImGui_ImplOpenGL3_Shutdown();
			ImGui_ImplGlfw_Shutdown();
			ImGui::DestroyContext();
		}
	};
}
