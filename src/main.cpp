#include <gl/glut.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <windows.h>
#include <vector>
#include <atomic>
#include <chrono>
#include <thread>
#include <SOIL.h>
#include "../include/body/world.h"
#include "./imgui/myimgui.h"
#include "./imgui/stb_image.h"
static physicalEngine::World staticWorld;
constexpr double windowTop = 900.f;
constexpr double windowBottom = 0.f;

constexpr double windowL = 0.0f;
constexpr double windowR = windowTop / 0.618f;

constexpr double sceneMaxXpos = 1000.f;


static GLFWwindow* window;

void MouseFun(GLFWwindow* window1, int button, int action, int mods)
{
	double x, y;
	glfwGetCursorPos(window, &x, &y);
	if ( x > sceneMaxXpos) return;
	y = windowTop - y;
	


	if (button == GLFW_MOUSE_BUTTON_RIGHT) {
		if (action == GLFW_PRESS)
		{
			//std::cout << "右键点击 x: " << x << " y: " << y << std::endl;

			staticWorld.addRandomBody(x, y);
		}
		else if (action == GLFW_RELEASE)
		{
			//std::cout << "右键释放 x: " << x << " y: " << y << std::endl;
		}
	}
	else if (button == GLFW_MOUSE_BUTTON_LEFT)
	{
		if (action == GLFW_PRESS)
		{
			//std::cout << "左键点击 x: " << x << " y: " << y << std::endl;

			staticWorld.selectBody(x, y);
		}
		else if (action == GLFW_RELEASE)
		{
			//std::cout << "左键释放: " << x << " y: " << y << std::endl;

			staticWorld.clearSelectBody();
		}
	}
}

void MouseMoveFun(GLFWwindow* window, double x, double y)
{
	if (x > sceneMaxXpos) return;
	y = windowTop - y;
	
	x /= 10.f;
	y /= 10.f;
	staticWorld.dragBodyToPos(x, y);
	//std::cout << "移动至x: " << x << " y: " << y << std::endl;
}

void display() {
	//staticWorld.Lock();
	glViewport(0, 0, windowR - windowL, windowTop - windowBottom);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(windowL, windowR, windowBottom, windowTop, 1, -1);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_STENCIL);
	//glfwWindowHint(GLFW_SAMPLES, 8);
	//glEnable(GL_MULT);
	glEnable(GL_BLEND);
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);

	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
	glHint(GL_POLYGON_SMOOTH, GL_NICEST);

	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	staticWorld.draw();
	//staticWorld.Unlock();
	//glDisable(GL_MULT);

}
void InitScene()
{
	staticWorld.initScene(0);
}

static auto DiffTime() {
	using namespace std::chrono;
	using Seconds = std::chrono::duration<double>;
	static auto last_clock = high_resolution_clock::now();
	auto now = high_resolution_clock::now();
	auto dt = duration_cast<Seconds>(now - last_clock);
	last_clock = now;
	return dt;
}

static void PhysicalEngineRun()
{
	using namespace std::chrono_literals;
	while (true) {
		// We give up some time for drawing
		std::this_thread::sleep_for(10ms);
		auto dt = DiffTime().count();
		//std::cout << dt << std::endl;
		staticWorld.step(dt);
	}
}

int main()
{
	srand(time(0));
	InitScene();
	//实例化glfw窗口，并告诉glfw使用的opengl版本：major 3，minor 3（3.3）
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);


	window = glfwCreateWindow(windowR - windowL, windowTop - windowBottom, "2DPhysicalEngine", nullptr, nullptr);
	if (window == nullptr)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return -1;
	}

	

	//glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1); // Enable vsync

	glfwSetMouseButtonCallback(window, MouseFun);
	glfwSetCursorPosCallback(window, MouseMoveFun);

	//GLFWimage icons[1];
	//icons[0].pixels = SOIL_load_image("‪C:\\Users\\sc\\Desktop\\2\\logo.ico", &icons[0].width, &icons[0].height, 0, 4);
	//glfwSetWindowIcon(window, 1, icons);
	////SOIL_free_image_data(icons[0].pixels);

	GLFWimage icon;
	int channels = 0;
	icon.pixels = SOIL_load_image("D:\\2d2.png", &icon.width, &icon.height, &channels, SOIL_LOAD_RGBA);
	if (icon.pixels == NULL) {
		printf("\nError loading image: %s\n", SOIL_last_result());
	}
	else
	{
		glfwSetWindowIcon(window, 1, &icon);
	}

	//GLFWimage images[1];
	//images[0].pixels = stbi_load("‪D:\\logo2.png", &images[0].width, &images[0].height, 0, 4); //rgba channels 
	//glfwSetWindowIcon(window, 1, images);



	std::thread physical_thread(PhysicalEngineRun);

	physicalEngine::EngineGUI::Setup(window);

	while (!glfwWindowShouldClose(window))
	{
	
		glfwPollEvents();
		display();
		physicalEngine::EngineGUI::RenderImGui(staticWorld);

		glfwSwapBuffers(window);

		//processInput(window);//输入检测
	}
	physical_thread.join();
	physicalEngine::EngineGUI::Clearup();
	glfwDestroyWindow(window);
	glfwTerminate();
	return 0;
}
