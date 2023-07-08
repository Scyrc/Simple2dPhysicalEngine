// 2DPhysicalEngine.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include <windows.h>
#include <gl/glut.h>
#include <vector>
#include "world.h"
static physicalEngine::World staticWorld;

void reshape(GLsizei w, GLsizei h)
{
	if (h == 0)
		h = 1;
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	if (w <= h)
	{
		glOrtho(0.0f,w, 0.0f,h * h / w, 1.0f, -1.0f);
	}
	else
	{
		glOrtho(0.0f, w * w / h, 0.0f, h, 1.0f, -1.0f);
	}

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}
void display() {
	
	static auto lastTime = GetTickCount64() * 0.001f;
	auto currentTime = GetTickCount64() * 0.001f;

	if (currentTime - lastTime > (1.0 / 60.0))
	{
		glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // Set background color to black and opaque
		glClear(GL_COLOR_BUFFER_BIT);         // Clear the color buffer (background)
		staticWorld.step();
		glFlush();
		lastTime = currentTime;
	}
}
void InitScene()
{
	staticWorld.addPolygon(1, { { -10, -10 }, {0,5}, {60, 0}, }, { 200,400 });

	staticWorld.addPolygon(1, { { 10, 0 }, {-10,0}, {0, 10}, }, { 300,600 });

	staticWorld.addPolygon(1, { { 10, 0 }, {-10,0}, {-10,20}, {10, 30} }, { 400,600 });

	staticWorld.addPolygon(1, { { -22, -10 }, {-30, 15}, {-20, 30}, {10, 20}, {20, -20} }, { 500,600 });

	//staticWorld.addPolygon(1, { { 10, 0 }, {-10,0}, {-10,20}}, { 150,200 });

}

int main(int argc, char** argv)
{
	InitScene();
	glutInit(&argc, argv);                 // Initialize GLUT
	glutInitWindowSize(800, 800);   // Set the window's initial width & height
	glutInitWindowPosition(500, 500); // Position the window's initial top-left corner
	glutCreateWindow("Setup"); // Create a window with the given title
	//glOrtho(0.0f, 300.0f, 0.0f, 300.0f, 1.0, -1.0);
	glutReshapeFunc(reshape);
	glutDisplayFunc(display); // Register display callback handler for window re-paint
	glutIdleFunc(display); // 没有事件输入时调用，这里不用它
	glutMainLoop();           // Enter the event-processing loop
	return 0;
}

// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

// 入门使用技巧: 
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件
