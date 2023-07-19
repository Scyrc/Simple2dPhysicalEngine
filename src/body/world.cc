#include "../../include/body/world.h"
#include "../../include/collision/collision.h"
#include <math.h>

#define _USE_MATH_DEFINES
namespace physicalEngine
{

	World::World()
	{
		bvhTree = new BVHTree();
	}

	physicalEngine::Polygon* World::addPolygon(float density, const VerticesListType& verticesList, const Vec2& position /*= { 0, 0 }*/, double rotateAngle, bool isStatic)
	{
		auto body = new Polygon(density, verticesList);
		body->isStatic = isStatic;
		body->setPos(position);
		body->RotateTo(rotateAngle);
		body->id = bodyList.size();
		bodyList.push_back(body);
		return body;
	}

	void World::GenerateTree()
	{
		bvhTree->generate(bodyList);
	}

	void World::step(Double dt)
	{
		std::lock_guard<std::mutex> lock(mutex_);
		
		//if (simulateSpeed == 0) return;

		/*bodyList[0]->setVel(bodyList[0]->getVel() + Vec2(-1, 0));
		bodyList[1]->setVel(bodyList[1]->getVel() + Vec2(-1, 0));
		bodyList[7]->setVel(bodyList[7]->getVel() + Vec2(-0.6, 0));*/

		for (int i = 0; i < simulateSpeed; ++i)
		{
			StepVelocity(dt);
			StepPosition(dt);

			bvhTree->update(bodyList);
			CollisionManager::Detect(bvhTree);
			CollisionManager::solveCollision();	

			//draw();
		}
	}

	void World::draw()
	{
		std::lock_guard<std::mutex> lock(mutex_);

		//glClearColor(129 / 255.0, 119 / 255.0, 172 / 255.0, 1.0f); // Set background color to black and opaque

		glClearColor(backgroundR, backgroundG, backgroundB, backgroundA); // Set background color to black and opaque
		glClear(GL_COLOR_BUFFER_BIT);         // Clear the color buffer (background)

		for (auto& body : bodyList)
		{
			body->Draw();
		}
		if(showBVH)
			bvhTree->draw();

		if (showContact)
			CollisionManager::drawContacts();

		glFlush();

	}

	void World::StepVelocity(Double dt)
	{
		for (auto& body : bodyList)
		{
			body->stepVel(Gravity, dt);
		}
	}

	void World::StepPosition(Double dt)
	{
		for (auto& body : bodyList)
		{
			body->stepPos(dt);
		}
	}
	void World::RandomRect(double x, double y)
	{
		double bottomL = double(rand()) / RAND_MAX * 20 + 20;
		double heightL = double(rand()) / RAND_MAX * 20 + 20;
		addPolygon(1, { { -bottomL / 2.f, 0 }, {-bottomL / 2.f,heightL}, { bottomL / 2.f, heightL},{ bottomL / 2.f, 0} }, { x,y });
	}
	void World::RandomTriangle(double x, double y)
	{
		double bottomL = double(rand()) / RAND_MAX * 20 + 20;
		double heightL = double(rand()) / RAND_MAX * 20 + 20;
		addPolygon(1, { { -bottomL / 2.f, 0 }, {0,heightL}, { bottomL / 2.f, 0}, }, { x,y });
	}
	void World::Randompolygon(double x, double y)
	{

	}
	void World::RandomCircle(double x, double y)
	{

	}
	void World::addRandomBody(double x, double y)
	{
		VerticesListType verticesList;
		int idx = rand() % 2;
		switch (idx)
		{
		case 0:
			RandomRect(x, y);
			break;
		case 1:
			RandomTriangle(x, y);
			break;
		case 2:
			Randompolygon(x, y);
			break;
		default:
			break;
		}
	}

	void  World::selectBody(double x, double y)
	{
		selectBodyIns = bodyList.back();
	}
	void  World::dragBodyToPos(double x, double y)
	{
		if (selectBodyIns == nullptr) return;

		selectBodyIns->moveToPos(Vec2(x, y));
	}
	void  World::clearSelectBody()
	{
		selectBodyIns = nullptr;
	}

	void World::setBackGround(Double backgroundR_, Double backgroundG_, Double backgroundB_, Double backgroundA_)
	{
		backgroundR = backgroundR_;
		backgroundG = backgroundG_;
		backgroundB = backgroundB_;
		backgroundA = backgroundA_;
	}

	void World::setDispalyProperties(bool shoWBVH_, bool showContact_)
	{
		showBVH = shoWBVH_;
		showContact = showContact_;
	}

	void World::initScene(int sceneId_)
	{
		std::lock_guard<std::mutex> lock(mutex_);
		if (sceneIndex == sceneId_) return;
		sceneIndex = sceneId_;
		switch (sceneIndex)
		{
		case 0:
			RandomScene();
		case 1:
			boxScene();
		default:
			break;
		}

	}


	void World::clearScene()
	{
		//std::lock_guard<std::mutex> lock(mutex_);

		bodyList.clear();
		bvhTree->destory();
		bvhTree = new BVHTree();
	}

	void World::changeScene(size_t sceneId_)
	{
		std::lock_guard<std::mutex> lock(mutex_);

		if (sceneIndex == sceneId_) return;
		sceneIndex = sceneId_;
		clearScene();
		switch (sceneIndex)
		{
		case 0:
			RandomScene();
		case 1:
			boxScene();
		default:
			break;
		}
	}

	void  World::RandomScene()
	{

		addPolygon(1, { { -10, -10 }, {0,10}, {60, 0}, }, { 500,400 });

		addPolygon(1, { { 20, 0 }, {-20,0}, {0, 20}, }, { 600,600 });

		addPolygon(1, { { 20, 0 }, {-20,0}, {0, 20}, }, { 300,300 });

		addPolygon(1, { { 10, 0 }, {-10,0}, {-10,20}, {10, 30} }, { 400,600 });

		addPolygon(1, { { -22, -10 }, {-30, 15}, {-20, 30}, {10, 20}, {20, -20} }, { 500,600 });

		addPolygon(1, { { -22, -10 }, {-30, 15}, {-20, 30}, {10, 20}, {20, -20} }, { 350,400 });

		addPolygon(1, { {0, 0 }, {0,20}, {20, 20}, {20, 0} }, { 200,700 });

		addPolygon(1, { {0, 0 }, {0,20}, {40, 20}, {40, 0} }, { 500,280 });


		addPolygon(10, { {2,2 }, {2,22}, {1000, 22}, {1000, 2 } }, { 0,0 }, 0, true);

		addPolygon(10, { {0, 0 }, {0,20}, {400, 20}, {400, 0} }, { 400,150 }, std::acos(-1) / 7, true);

		addPolygon(10, { {0, 0 }, {0,20}, {300, 20}, {300, 0} }, { 100,300 }, -std::acos(-1) / 7, true);


		//addPolygon(10000000000, { {0, 0 }, {0,50}, {50, 50}, {50, 0} }, { 700,300 }, 0, true);

		GenerateTree();
	}
	void  World::boxScene()
	{

	}
}