// 2DPhysicalEngine.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

//#include <iostream>
//#include <windows.h>
//#include <gl/glut.h>
//#include <vector>
//#include "world.h"
//#include <atomic>
//#include <chrono>
//#include <thread>
//static physicalEngine::World staticWorld;
//constexpr double windowL =  0.0f;
//constexpr double windowR =  1000.f;
//
//constexpr double windowTop = 1000.f;
//constexpr double windowBottom = 0.f;
//
//void reshape(GLsizei w, GLsizei h)
//{
//	if (h == 0)
//		h = 1;
//	glViewport(0, 0, w, h);
//	glMatrixMode(GL_PROJECTION);
//	glLoadIdentity();
//
//	if (w <= h)
//	{
//		glOrtho(windowL, windowR, windowBottom, windowTop * h / w, 1.0f, -1.0f);
//	}
//	else
//	{
//		glOrtho(windowL, windowR * w / h, windowBottom, windowTop, 1.0f, -1.0f);
//	}
//
//	glMatrixMode(GL_MODELVIEW);
//	glLoadIdentity();
//}
//
//void myMouseFun(int button, int state, int x, int y)
//{
//	y = windowTop - y;
//	if (button == GLUT_RIGHT_BUTTON) {
//		if (state == GLUT_DOWN)
//		{
//			//std::cout << "右键点击 x: " << x << " y: " << y << std::endl;
//
//			staticWorld.addRandomBody(x, y);
//		}
//		else if (state == GLUT_UP)
//		{
//			//std::cout << "右键点击 x: " << x << " y: " << y << std::endl;
//		}
//	}
//	else if(button == GLUT_LEFT_BUTTON)
//	{
//		if (state == GLUT_DOWN)
//		{
//			staticWorld.selectBody(x, y);
//		}
//		else if (state == GLUT_UP)
//		{
//			//std::cout << "右键点击 x: " << x << " y: " << y << std::endl;
//			staticWorld.clearSelectBody();
//		}
//	}
//}
//void myMovedMouse(int x, int y)
//{
//	y = windowTop - y;
//	staticWorld.dragBodyToPos(x, y);
//	//std::cout << "移动至x: " << x << " y: " << y << std::endl;
//}
//
//void myKeyboard(unsigned char key, int mouseX, int mouseY) {
//	GLint x = mouseX;
//	GLint y = mouseY;
//	switch (key)
//	{
//	case 'p':
//		break;
//	case 'E':
//		exit(0);
//
//	default:
//		break;
//	}
//}
//void displayold() {
//	
//	static auto lastTime = GetTickCount64() * 0.001f;
//	auto currentTime = GetTickCount64() * 0.001f;
//
//	if (currentTime - lastTime > (1.0 / 240.0))
//	{
//		glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // Set background color to black and opaque
//		glClear(GL_COLOR_BUFFER_BIT);         // Clear the color buffer (background)
//		staticWorld.step(currentTime - lastTime);
//		glFlush();
//		lastTime = currentTime;
//	}
//}
//
//void display() {
//	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_STENCIL);
//
//	glClearColor(129 / 255.0, 119 / 255.0, 172 / 255.0, 1.0f); // Set background color to black and opaque
//	glClear(GL_COLOR_BUFFER_BIT);         // Clear the color buffer (background)
//	staticWorld.draw();
//	glFlush();
//}
//void InitScene()
//{
//	staticWorld.initScene(0);
//}
//
//static auto DiffTime() {
//	using namespace std::chrono;
//	using Seconds = std::chrono::duration<double>;
//	static auto last_clock = high_resolution_clock::now();
//	auto now = high_resolution_clock::now();
//	auto dt = duration_cast<Seconds>(now - last_clock);
//	last_clock = now;
//	return dt;
//}
//
//static void PhysicalEngineRun()
//{
//	using namespace std::chrono_literals;
//	while (true) {
//		// We give up some time for drawing
//		std::this_thread::sleep_for(10ms);
//		auto dt = DiffTime().count();
//		std::cout << dt << std::endl;
//		staticWorld.step(dt);
//	}
//}
//void timerProc(int id)
//{
//	display();
//	glutTimerFunc(10, timerProc, 1);//需要在函数中再调用一次，才能保证循环
//}
//
//int main1(int argc, char** argv)
//{
//	srand(time(0));
//	InitScene();
//	std::thread physical_thread(PhysicalEngineRun);
//	glutInit(&argc, argv);                 // Initialize GLUT
//	glutInitWindowSize(windowR - windowL, windowTop - windowBottom);   // Set the window's initial width & height
//	glutInitWindowPosition(500, 500); // Position the window's initial top-left corner
//	glutCreateWindow("Setup"); // Create a window with the given title
//	//glOrtho(0.0f, 300.0f, 0.0f, 300.0f, 1.0, -1.0);
//	glutReshapeFunc(reshape);
//	glutDisplayFunc(display); // Register display callback handler for window re-paint
//	glutTimerFunc(100, timerProc, 1); //glutTimerFunc(毫秒数, 回调函数指针, 区别值);
//	//glutIdleFunc(); // 没有事件输入时调用，这里不用它
//	glutMouseFunc(myMouseFun);
//	glutMotionFunc(myMovedMouse);
//	glutKeyboardFunc(myKeyboard);
//	glutMainLoop();           // Enter the event-processing loop
//
//	physical_thread.join();
//
//	return 0;
//}
