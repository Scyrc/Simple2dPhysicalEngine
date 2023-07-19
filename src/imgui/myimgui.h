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
		static int simulate_speed;
		static int scene_id;
		//static int old_scene_id;

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
				static int counter = 0;
				ImGui::Begin("Hello 2D Engine!", 0, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);                          // Create a window called "Hello, world!" and append into it.

				const char* items[] = { "scene1-random", "scene2-box" };
				ImGui::SetNextItemWidth(200);
				ImGui::Combo("selected", &scene_id, items, IM_ARRAYSIZE(items));

				ImGui::Checkbox("show bvhTree", &show_bvhTree);      
				ImGui::Checkbox("show contacts", &show_contacts);

				ImGui::SliderInt("simulate speed", &simulate_speed, 0, 10);            
				ImGui::ColorEdit3("background color", (float*)&clear_color); // Edit 3 floats representing a color

				if (ImGui::Button("Button"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
					counter++;
				ImGui::SameLine();
				ImGui::Text("body num = %d", staticWorld.getBodyNums());

				ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ioStatic->Framerate, ioStatic->Framerate);

				

				//ImGui::PushItemWidth(200);

				//ImGui::Text("combo select:%d", item_current);

				ImGui::End();
			}

			ImGui::Render();
			ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

			staticWorld.changeScene(scene_id);
			staticWorld.setBackGround(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
			staticWorld.setDispalyProperties(show_bvhTree, show_contacts);
			staticWorld.setSimluateSpeed(simulate_speed);
		}

		static inline void Clearup()
		{
			ImGui_ImplOpenGL3_Shutdown();
			ImGui_ImplGlfw_Shutdown();
			ImGui::DestroyContext();
		}
	};

}
