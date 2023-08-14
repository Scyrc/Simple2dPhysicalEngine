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

	physicalEngine::Circle* World::addCircle(float density, Double R, const Vec2& position /*= { 0, 0 }*/, double rotateAngle /*= 0*/, bool isStatic /*= false*/)
	{
		auto body = new Circle(density, position, R);
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
		if (pauseWorld) return;
		std::lock_guard<std::mutex> lock(mutex_);
		
		//if (simulateSpeed == 0) return;

		/*bodyList[0]->setVel(bodyList[0]->getVel() + Vec2(-1, 0));
		bodyList[1]->setVel(bodyList[1]->getVel() + Vec2(-1, 0));
		bodyList[7]->setVel(bodyList[7]->getVel() + Vec2(-0.6, 0));*/
		dt = std::min(dt, Double(0.01f));
		for (int i = 0; i < simulateSpeed; ++i)
		{
			StepVelocity(dt);
			bvhTree->update(bodyList);

			CollisionManager::Detect(bvhTree);
			CollisionManager::clearInactiveCollisionAndContactpointPoint();
			CollisionManager::perpareContact();
			
			for (auto& joint : joinsList)
			{
				joint->prepare(dt);
			}
			for (int i=0;i<velocityInterationNumPreFrame;++i)
			{
				for (auto& joint : joinsList)
				{
					joint->solveVelocity(dt);
				}
				CollisionManager::solveCollisionNew();
			}

			StepPosition(dt);

			for (int i = 0; i < positionInterationNumPreFrame; ++i)
			{
				CollisionManager::solvePosNew(dt);
			}



			CollisionManager::deactivateCollisionAndContactpoint();
		}
	}

	void World::draw()
	{
		std::lock_guard<std::mutex> lock(mutex_);

		glClearColor(backgroundR, backgroundG, backgroundB, backgroundA); 
		glClear(GL_COLOR_BUFFER_BIT);        
	

		for (auto& body : bodyList)
		{
			body->Draw();
		}

		for (auto& joint : joinsList)
		{
			joint->Draw();
		}

		if(showBVH)
			bvhTree->draw();

		if (showContact)
			CollisionManager::drawContacts();

		glFlush();

		//double maxVeloctiy = 0;
		//double maxangluarVelocity = 0;

		/*
		for (auto& body : bodyList)
		{
			//std::cout << "velocity:" << body->getVel().length() << std::endl;
			//if (maxangluarVelocity > 1000.f)
			//std::cout << "angular velocity:" << body->getAnguleVel() << std::endl;

			if (body->getVel().length() > maxVeloctiy)
			{
				maxVeloctiy = body->getVel().length();
			}

			if (std::fabs(body->getAnguleVel()) > maxangluarVelocity)
			{
				maxangluarVelocity = std::fabs(body->getAnguleVel());
			}
		}
		
		//if(maxVeloctiy)
		//std::cout << "max velocity:" << maxVeloctiy << std::endl;
		//if (maxangluarVelocity > 1000.f)
		//std::cout << "max angular velocity:" << maxangluarVelocity << std::endl;
		*/
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
		x /= 10.f;
		y /= 10.f;
		double bottomL = double(rand()) / RAND_MAX * 2 + 2;
		double heightL = double(rand()) / RAND_MAX * 2 + 2;
		auto body = addPolygon(1, { { -bottomL / 2.f, 0 }, {-bottomL / 2.f,heightL}, { bottomL / 2.f, heightL},{ bottomL / 2.f, 0} }, { x,y });
		if (sceneIndex == 0)
		{
			body->fallDownMode = true;
			body->setRestitution(0.3f);
			body->setFriction(0.0f, 0.0f);
		}
		
	}
	void World::RandomTriangle(double x, double y)
	{
		x /= 10.f;
		y /= 10.f;
		double bottomL = double(rand()) / RAND_MAX * 2 + 2;
		double heightL = double(rand()) / RAND_MAX * 2 + 2;
		auto body =  addPolygon(1, { { -bottomL / 2.f, 0 }, {0,heightL}, { bottomL / 2.f, 0}, }, { x,y });
		if (sceneIndex == 0)
		{
			body->fallDownMode = true;
			body->setRestitution(0.3f);
			body->setFriction(0.0f, 0.0f);
		}
	}
	void World::Randompolygon(double x, double y)
	{

	}
	void World::RandomCircle(double x, double y)
	{
		x /= 10.f;
		y /= 10.f;
		double radius = double(rand()) / RAND_MAX * 1.5 + 1;
		auto body =  addCircle(0.5, radius, { x, y });
		if (sceneIndex == 0)
		{
			body->fallDownMode = true;
			body->setRestitution(0.2f);
			body->setFriction(0.1f, 0.1f);
		}
	}
	void World::addRandomBody(double x, double y)
	{
		VerticesListType verticesList;
		int idx = rand() % 3;
		switch (idx)
		{
		case 0:
			RandomRect(x, y);
			break;
		case 1:
			RandomTriangle(x, y);
			break;
		case 2:
			RandomCircle(x, y);
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

	void World::setGravity(Double x, Double y)
	{
		Gravity = Vec2(x, y);
	}

	void World::initScene(int sceneId_)
	{
		std::lock_guard<std::mutex> lock(mutex_);
		if (sceneIndex == sceneId_) return;
		sceneIndex = sceneId_;
		switch (sceneIndex)
		{
		case 0:
			fallDownScene();
			break;
		case 1:
			boxScene();
			break;
		default:
			break;
		}

	}


	void World::clearScene()
	{
		//std::lock_guard<std::mutex> lock(mutex_);

		bodyList.clear();
		joinsList.clear();
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
			fallDownScene();
			break;
		case 1:
			restitutionScene();
			break;
		case 2:
			frictionScene();
			break;
		case 3:
			boxScene();
			break;
		case 4:
			jointScene();
			break;
		case 5:
			softJointScene();
			break;
		case 6:
			wreckingBall();
			break;
		case 7:
			DominoScene();
			break;
		case 8:
			seesawScene();
			break;
		case 9:
			dowblePendulumScene();
		default:
			break;
		}
	}

	void World::resetScene(size_t sceneId)
	{
		//std::lock_guard<std::mutex> lock(mutex_);
		sceneIndex = -1;
		
		changeScene(sceneId);
	}

	void  World::RandomScene()
	{
		
		addPolygon(0.25, { { -1, -1 }, {0,1}, {6, 0}, }, { 50,40 });
	
		addPolygon(0.25, { { 2, 0 }, {-2,0}, {0, 2}, }, { 60,60 });

		addPolygon(0.25, { { 2, 0 }, {-2,0}, {0, 2}, }, { 30,30 });

		addPolygon(0.25, { { 1, 0 }, {-1,0}, {-1,2}, {1, 3} }, { 40,60 });

		addPolygon(0.25, { { -2.2, -1 }, {-3, 1.5}, {-2, 3}, {1, 2}, {2, -2} }, { 50,60 });

		addPolygon(0.25, { { -2.2, -1 }, {-3, 1.5}, {-2, 3}, {1, 2}, {2, -2} }, { 35,40});

		addPolygon(0.25, { {0, 0 }, {0,2}, {2, 2}, {2, 0} }, { 20,70});

		addPolygon(0.25, { {0, 0 }, {0,2}, {4, 2}, {4, 0} }, { 50,28});

		

		addPolygon(0.25, { {0,0 }, {0,2}, {20, 2}, {20, 0} }, {80, 30}, 0, true);

		
		//addPolygon(0.25, { {0,0 }, {0,2}, {100, 2}, {100, 0} }, { 0,0 }, 0, true);

		addPolygon(0.25, { {0, 0 }, {0,2}, {40, 2}, {40, 0} }, { 40,15 }, std::acos(-1) / 7, true);

		addPolygon(0.25, { {0, 0 }, {0,2}, {30, 2}, {30, 0} }, { 10,28 }, -std::acos(-1) / 7, true);
		
		addCircle(0.25, 2.0f, { 80,60 });



		addPolygon(0.25, { {0, 0 }, {0,5}, {5, 5}, {5, 0} }, { 70,30 }, 0, true);

		for (auto& body : bodyList)
		{
			body->fallDownMode = true;
		}
		GenerateTree();
	}
	void  World::boxScene()
	{
		auto ground = addPolygon(1, { {0, 0 }, {0,2}, {100, 2}, {100, 0} }, { 0,0}, 0, true);
		ground->setRestitution(0);
		ground->setFriction(1, 1);
		int sceneCenter = 50;
		float boxSize = pyramidBoxSize;
		int bottomNum = pyramidLayerNum;
		float posX;
		float posY = boxSize + 0.1f;
		float gapX = 0.2f;
		while (bottomNum > 0)
		{
			if (bottomNum % 2 == 0)
			{
				posX = sceneCenter - (bottomNum / 2 * boxSize + float(bottomNum - 1) / 2.0f * gapX);

			}
			else
			{
				posX = sceneCenter - ( float(bottomNum) / 2.0 * boxSize + (bottomNum - 1) / 2 * gapX);

			}
			for (size_t i = bottomNum; i > 0; --i)
			{
				auto box = addPolygon(0.1, { {0, 0 }, {0,boxSize}, {boxSize, boxSize}, {boxSize, 0} }, { posX, posY});
				box->setRestitution(pyramidRestitution);
				box->setFriction(1, 1);
				posX += boxSize;
				posX += gapX;
			}

			posY += boxSize;
			posY += 0.2;

			--bottomNum;
		}
	}

	void World::jointScene()
	{	
		{
			float startPosX = 30;
			float startPosY = 10;
			float BoxSize = 4.f;
			float lineLength = 28.f;
			float gapx = 0.f;
			auto body = addPolygon(0.1, { {-BoxSize / 2.f, -BoxSize / 2.f}, {-BoxSize / 2.f, BoxSize / 2.f}, {BoxSize / 2.f, BoxSize / 2.f}, {BoxSize / 2.f, -BoxSize / 2.f} }, { startPosX - lineLength,startPosY + lineLength });
			body->setRestitution(0.5);
			body->setFriction(0, 0);
			body->setVelocityDamping(0, 0);
			DistancePointPrimitive primitive;
			primitive.bodyA = body;
			primitive.localPointA = body->getCentroid();
			primitive.targetPoint = { startPosX, startPosY + lineLength };
			primitive.maxDistance = (primitive.targetPoint - primitive.bodyA->getCentroidWorldPos()).length();
			primitive.minDistance = primitive.maxDistance;
			auto joint = new DistanceJoint(primitive);
			this->joinsList.emplace_back(joint);

			for (int i = 0; i < 5; ++i)
			{
				startPosX += BoxSize;
				startPosX += gapx;
				auto body = addPolygon(0.1, { {-BoxSize / 2.f, -BoxSize / 2.f}, {-BoxSize / 2.f, BoxSize / 2.f}, {BoxSize / 2.f, BoxSize / 2.f}, {BoxSize / 2.f, -BoxSize / 2.f} }, { startPosX,startPosY });
				body->setRestitution(0.5);
				body->setFriction(0, 0);
				body->setVelocityDamping(0, 0);
				DistancePointPrimitive primitive;
				primitive.bodyA = body;
				primitive.localPointA = body->getCentroid();
				primitive.targetPoint = { startPosX, startPosY + lineLength };
				primitive.maxDistance = (primitive.targetPoint - primitive.bodyA->getCentroidWorldPos()).length();
				primitive.minDistance = primitive.maxDistance;

				auto joint = new DistanceJoint(primitive);
				this->joinsList.emplace_back(joint);
			}

		}

		{
			int circleNum = newtonPendunlumCircleNum;
			float CircleRadius = newtonPendunlumCircleRadius;
			float lineLength = newtonPendunlumLineLength;
			float CircleRestitution = newtonPendunlumRestitution;

			float gapx = 0;

			float startPosX = 30;
			float startPosY = 50.f;
			float density = 1;

			auto body = addCircle(density, CircleRadius, Vec2(startPosX - lineLength, startPosY + lineLength));
			body->setRestitution(CircleRestitution);
			body->setFriction(0, 0);
			body->setVelocityDamping(0, 0);
			DistancePointPrimitive primitive;
			primitive.bodyA = body;
			primitive.localPointA = body->getCentroid();
			primitive.targetPoint = { startPosX, startPosY + lineLength };
			primitive.maxDistance = (primitive.targetPoint - primitive.bodyA->getCentroidWorldPos()).length();
			primitive.minDistance = primitive.maxDistance;
			auto joint = new DistanceJoint(primitive);
			this->joinsList.emplace_back(joint);

			for (int i = 0; i < circleNum; ++i)
			{
				startPosX += CircleRadius * 2;
				startPosX += gapx;
				auto body = addCircle(density, CircleRadius, Vec2(startPosX, startPosY));
				body->setRestitution(CircleRestitution);
				body->setFriction(0, 0);
				body->setVelocityDamping(0, 0);
				DistancePointPrimitive primitive;
				primitive.bodyA = body;
				primitive.localPointA = body->getCentroid();
				primitive.targetPoint = { startPosX, startPosY + lineLength };
				primitive.maxDistance = (primitive.targetPoint - primitive.bodyA->getCentroidWorldPos()).length();
				primitive.minDistance = primitive.maxDistance;

				auto joint = new DistanceJoint(primitive);
				this->joinsList.emplace_back(joint);

			}

		}
	}

	void World::softJointScene()
	{
		float sceneCenter = 50.f;
		int num = BridgejointNum;
		float BoxWidth = BridgejointWidth;
		float BoxHeight= BoxWidth / 3.0f;
		float lineLength = 1;
		float gapx = BoxWidth / 5.0f;
		float startPosX = sceneCenter - (float(num) / 2 * BoxWidth + float(num -1 ) / 2 * gapx);
		float BridgestartPosY = 30;
		Body* body1 = addPolygon(1, { {-BoxWidth / 2.f, -BoxHeight / 2.f}, {-BoxWidth / 2.f, BoxHeight / 2.f}, {BoxWidth / 2.f, BoxHeight / 2.f}, {BoxWidth / 2.f, -BoxHeight / 2.f} }, { startPosX ,BridgestartPosY });
		Body* body2;
		/*
		{
			PointJointPrimitive ppm;

			ppm.bodyA = body1;
			ppm.localPointA = Vec2(-BoxWidth / 2.f, 0.f);
			ppm.targetPoint = body1->toWorldPoint(Vec2(-BoxWidth / 2.f, 0.f));
			ppm.dampingRatio = 0.1f;
			ppm.frequency = 1000;
			ppm.maxForce = 10000;
			auto pointJoint = new PointJoint(ppm);
			this->joinsList.emplace_back(pointJoint);
		}
		*/
		DistancePointPrimitive primitive;
		primitive.bodyA = body1;
		primitive.localPointA = Vec2{ - BoxWidth / 2.f, 0.f };
		primitive.targetPoint = { startPosX - lineLength, BridgestartPosY };
		primitive.maxDistance = (primitive.targetPoint - primitive.bodyA->getCentroidWorldPos()).length();
		primitive.minDistance = primitive.maxDistance;

		auto joint = new DistanceJoint(primitive);
		this->joinsList.emplace_back(joint);
		
		for (int i = 1; i < num; ++i)
		{
			startPosX += BoxWidth;
			startPosX += gapx;
			body2 = addPolygon(1, { {-BoxWidth / 2.f, -BoxHeight / 2.f}, {-BoxWidth / 2.f, BoxHeight / 2.f}, {BoxWidth / 2.f, BoxHeight / 2.f}, {BoxWidth / 2.f, -BoxHeight / 2.f} }, { startPosX ,BridgestartPosY });

			RevoluteConstraintPrimitive primitive;
			primitive.bodyA = body1;
			primitive.bodyB = body2;
			primitive.localPointA = Vec2{ BoxWidth / 2.f + gapx * 0.5f, 0.f };
			primitive.localPointB = Vec2{ -BoxWidth / 2.f - gapx * 0.5f , 0.f };
			primitive.dampingRatio = 0.8f;
			primitive.frequency = 10;
			primitive.maxForce = 10000;
			auto joint = new RevoluteConstraintJoint(primitive);
			this->joinsList.emplace_back(joint);

			body1 = body2;
		}

		{
			DistancePointPrimitive primitive;
			primitive.bodyA = body1;
			primitive.localPointA = Vec2{ BoxWidth / 2.f, 0.f };
			primitive.targetPoint = { startPosX + lineLength, BridgestartPosY };
			primitive.maxDistance = (primitive.targetPoint - primitive.bodyA->getCentroidWorldPos()).length();

			auto joint = new DistanceJoint(primitive);
			this->joinsList.emplace_back(joint);
		}


		int boxlayernum = BridgeboxLayerNum;
		int boxnumprelayer = BridgeboxNumPreLayer;
		float boxSize = BridgeboxSize;

		float boxPosX = sceneCenter - float(boxlayernum) / 2.0f * boxSize;
		float boxPosY = BridgestartPosY;


		for (int i=0;i<boxlayernum;++i)
		{
			boxPosX = sceneCenter - float(boxlayernum) / 2.0f * boxSize;
			boxPosY += boxSize;
			for (int j =0;j<boxnumprelayer;++j)
			{
				auto box = addPolygon(0.5, { {-boxSize / 2.f,-boxSize / 2.f}, {-boxSize / 2.f,boxSize / 2.f}, {boxSize / 2.f,boxSize / 2.f}, {boxSize / 2.f,-boxSize / 2.f } }, { boxPosX, boxPosY });
				box->setRestitution(0.1);
				box->setFriction(0.8, 0.3);
				boxPosX += boxSize;
			}
		}


		/*auto box = addPolygon(1, { {-boxSize / 2.f,-boxSize / 2.f}, {-boxSize / 2.f,boxSize / 2.f}, {boxSize / 2.f,boxSize / 2.f}, {boxSize / 2.f,-boxSize / 2.f } }, { 80, 60 });

		PointJointPrimitive ppm;

		ppm.bodyA = box;
		ppm.localPointA = Vec2(-boxSize / 2.f, boxSize / 2.f);
		ppm.targetPoint = box->toWorldPoint(Vec2(-boxSize / 2.f, boxSize / 2.f));
		ppm.dampingRatio = 0.1f;
		ppm.frequency = 10;
		ppm.maxForce = 10000;
		auto pointJoint = new PointJoint(ppm);
		this->joinsList.emplace_back(pointJoint);*/
	}

	void World::wreckingBall()
	{
		float groundH = 1.f;
		auto ground = addPolygon(100000.f, { {0,0 }, {0,groundH}, {100, groundH}, {100, 0} }, { 0,0 }, 0, true);
		ground->setRestitution(1.0f);
		ground->setFriction(1, 1);

		float sceneCenter = 50.f;
		int boxlayernum = 12;
		int boxnumprelayer = 6;
		float boxSize = 2.2;

		float boxPosX = sceneCenter - float(boxlayernum) / 2.0f * boxSize;
		float boxPosY = boxSize / 2.0f + groundH + 0.2f;
		float gapx = 0.1f;
		float gapy = 0.1f;

		for (int i = 0; i < boxlayernum; ++i)
		{
			boxPosX = sceneCenter - float(boxlayernum) / 2.0f * boxSize;
			for (int j = 0; j < boxnumprelayer; ++j)
			{
				auto box = addPolygon(0.01, { {-boxSize / 2.f,-boxSize / 2.f}, {-boxSize / 2.f,boxSize / 2.f}, {boxSize / 2.f,boxSize / 2.f}, {boxSize / 2.f,-boxSize / 2.f } }, { boxPosX, boxPosY });
				box->setRestitution(0.0);
				box->setFriction(1, 1);
				boxPosX += boxSize;
				boxPosX += gapx;
			}
			boxPosY += boxSize;
			boxPosY += gapy;
		}

		{
			float CircleRadius = 1.5f * boxSize;
			float lineLength = 20.f;

			float startPosX = boxPosX + CircleRadius;
			float startPosY = boxSize / 2.0f + groundH + 0.1f + float(boxlayernum) / 2.0f  * boxSize;
			
			float gapx = 0;
			float density = 2;
			auto body = addCircle(density, CircleRadius, Vec2(startPosX + lineLength, startPosY + lineLength));
			body->setRestitution(1);
			body->setFriction(0, 0);
			body->setVelocityDamping(0, 0);
			DistancePointPrimitive primitive;
			primitive.bodyA = body;
			primitive.localPointA = body->getCentroid();
			primitive.targetPoint = { startPosX, startPosY + lineLength };
			primitive.maxDistance = (primitive.targetPoint - primitive.bodyA->getCentroidWorldPos()).length();
			//primitive.minDistance = primitive.maxDistance;
			auto joint = new DistanceJoint(primitive);
			this->joinsList.emplace_back(joint);
		}
	}

	void World::frictionScene()
	{
		float pi = 3.1415926f;
		float rotateAngle = pi / 7;

		float flatFloorLength = 50.f;
		float rotateFloorLength = 50.f;

		float flatFloorPosY = 20.f;

		float flatFloorPosX = 80.f;

		int count = 3;
		while (count > 0)
		{
			-- count;
			float rotateFloorPosX = flatFloorPosX - flatFloorLength * 0.5 - rotateFloorLength * std::cos(rotateAngle) * 0.5;
			float rotateFloorPosY = flatFloorPosY + rotateFloorLength * std::sin(rotateAngle) * 0.5;

			auto floor = addPolygon(1, { {-flatFloorLength * 0.5, -0.1}, {-flatFloorLength * 0.5, 0.1}, {flatFloorLength * 0.5, 0.1}, {flatFloorLength * 0.5, -0.1} }, { flatFloorPosX,flatFloorPosY }, 0, true);
			floor->setFriction(0, 0);
			floor = addPolygon(1, { {-rotateFloorLength * 0.5, -0.1}, {-rotateFloorLength * 0.5, 0.1}, {rotateFloorLength * 0.5, 0.1}, {rotateFloorLength * 0.5, -0.1} }, { rotateFloorPosX,rotateFloorPosY }, -rotateAngle, true);
			floor->setFriction(0, 0);
			flatFloorPosY += 20.f;
		}
	}

	void World::DominoScene()
	{
		float rotateAngle = Constant::Pi / 8;

		float floorLength = 40.f;
		float groundH = 1.f;
		float posY = 58.f;
		auto ground1 = addPolygon(100000.f, { {0,0 }, {0,groundH}, {floorLength, groundH}, {floorLength, 0} }, {20, posY }, 0, true);
		ground1->setRestitution(0.1f);
		ground1->setFriction(0.1f, 0.1f);

		auto ground2 = addPolygon(100000.f, { {0,0 }, {0,groundH}, {floorLength, groundH}, {floorLength, 0} }, { 50, 52 }, rotateAngle, true);
		ground2->setRestitution(0.1f);
		ground2->setFriction(0.1f, 0.1f);

		auto ground3 = addPolygon(100000.f, { {0,0 }, {0,groundH}, {floorLength, groundH}, {floorLength, 0} }, { 20, 40 }, -rotateAngle, true);
		ground3->setRestitution(0.1f);
		ground3->setFriction(0.1f, 0.1f);


		float boxH = 8.f;
		float boxW =1.f;
		int boxNum = 9;
		float posX = 30;
		for (int i=0; i < boxNum; ++i)
		{
			auto box = addPolygon(1.f, { {0,0 }, {0,boxH}, {boxW, boxH}, {boxW, 0} }, { posX, posY + 1.5* groundH });
			box->setFriction(0.2f, 0.2f);
			box->setRestitution(0.0);
			box->setVelocityDamping(0, 0);
			posX += boxW * 4;
		}

		
		float BoxSize = boxW * 2.0f;
		float lineLength = boxH * 2.f;
		
		auto body = addPolygon(5, { {-BoxSize / 2.f, -BoxSize / 2.f}, {-BoxSize / 2.f, BoxSize / 2.f}, {BoxSize / 2.f, BoxSize / 2.f}, {BoxSize / 2.f, -BoxSize / 2.f} }, { 24 - lineLength ,posY +  boxH * 0.8f + lineLength });
		body->setRestitution(1);
		body->setFriction(0, 0);
		body->setVelocityDamping(0, 0);
		DistancePointPrimitive primitive;
		primitive.bodyA = body;
		primitive.localPointA = body->getCentroid();
		primitive.targetPoint = { 24, posY + boxH * 0.8f + lineLength };
		primitive.maxDistance = (primitive.targetPoint - primitive.bodyA->getCentroidWorldPos()).length();
		auto joint = new DistanceJoint(primitive);
		this->joinsList.emplace_back(joint);
		
	}

	void World::fallDownScene()
	{
		float rotateAngle = Constant::HalfPi / 7;

		float floorLength = 70.f;
		float groundH = 1.2f;

		float posX = 0;
		float posY = 65;

		auto ground1 = addPolygon(100000.f, { {0,0 }, {0,groundH}, {floorLength, groundH}, {floorLength, 0} }, { posX, posY }, -rotateAngle, true);
		ground1->setRestitution(1);
		ground1->setFriction(0.1f, 0.1f);

		posX = 36;
		posY = 40;
		auto ground2 = addPolygon(100000.f, { {0,0 }, {0,groundH}, {floorLength, groundH}, {floorLength, 0} }, { posX, posY }, rotateAngle, true);
		ground2->setRestitution(1);
		ground2->setFriction(0.1f, 0.1f);

		posX = 15;
		posY = 15;
		auto ground3 = addPolygon(100000.f, { {0,0 }, {0,groundH}, {floorLength, groundH}, {floorLength, 0} }, { posX, posY }, -rotateAngle, true);
		ground3->setRestitution(1);
		ground3->setFriction(0.1f, 0.1f);

		float bodyPosX = 10;
		float bodyPosY = 780;
		for (int i = 0; i < 15 ; ++i)
		{
			RandomCircle(bodyPosX, bodyPosY);
			auto circle = dynamic_cast<Circle*>(bodyList.back());
			circle->setRestitution(0.3);
			circle->setFriction(0.1f, 0.1f);
			bodyPosX += 50;
		}
		bodyPosX = 160;
		bodyPosY = 500;
		for (int i = 0; i < 15; ++i)
		{
			RandomCircle(bodyPosX, bodyPosY);
			auto circle = dynamic_cast<Circle*>(bodyList.back());
			circle->setRestitution(0.3);
			circle->setFriction(0.1f, 0.1f);
			bodyPosX += 50;
		}
	}

	void World::restitutionScene()
	{
		float floorLength = 100.f;
		float groundH = 1.f;
		auto ground1 = addPolygon(100000.f, { {0,0 }, {0,groundH}, {floorLength, groundH}, {floorLength, 0} }, { 0, 0 }, 0, true);
		ground1->setRestitution(1);
		ground1->setFriction(0.2f, 0.2f);

		float sceneCenter = 50.f;
		int circleNum = restitutionCircleNum;
		float CircleRadius = restitutionCircleRadius;
		float gap = 0.5 * CircleRadius;
		float posX = sceneCenter - (float(circleNum) / 2.0f * CircleRadius + float(circleNum - 1) / 2.0f * gap);
		float posY = CircleRadius * 6;
		for (int i = 0; i < circleNum; ++i)
		{
			auto circle = addCircle(1, CircleRadius, Vec2(posX, posY));
			posX += (2 * CircleRadius + gap);
			circle->setRestitution(float(i) / float(circleNum));
		}
	}

	void World::seesawScene()
	{
		float floorLength = 100.f;
		float groundH = 1.f;
		auto ground1 = addPolygon(100000.f, { {0,0 }, {0,groundH}, {floorLength, groundH}, {floorLength, 0} }, { 0, 0 }, 0, true);

		float planeLength = 36.f;
		float planeWidth = 1.5f;
		float planeX = 50.f;
		float planeY = 8.f;
		auto plane = addPolygon(1, { {-planeLength / 2.f,-planeWidth / 2.f},  {-planeLength / 2.f,planeWidth / 2.f}, {planeLength / 2.f,planeWidth / 2.f}, {planeLength / 2.f,-planeWidth / 2.f}, }, { planeX, planeY });
		//plane->setRestitution(0.5);
		plane->setFriction(0.1, 0.1);
		DistancePointPrimitive primitive;
		primitive.bodyA = plane;
		primitive.localPointA = plane->getCentroid();
		primitive.targetPoint = plane->getCentroidWorldPos();
		primitive.maxDistance = 0.01f;
		primitive.minDistance = 0.f;
		auto joint = new DistanceJoint(primitive);
		this->joinsList.emplace_back(joint);

		{
			float planeLength1 = 2.f;
			float planeWidth1 = planeY - planeWidth * 0.5;
			float planeX1 = planeX - planeLength * 0.5 + planeLength1 * 0.5;
			float planeY1 = planeY - planeWidth * 0.5 - planeWidth1 * 0.5 - 0.2f;
			auto plane = addPolygon(2.f, { {-planeLength1 / 2.f,-planeWidth1 / 2.f},  {-planeLength1 / 2.f,planeWidth1 / 2.f}, {planeLength1 / 2.f,planeWidth1 / 2.f}, {planeLength1 / 2.f,-planeWidth1 / 2.f}, }, { planeX1, planeY1 }, 0, true);
		}
		float boxNum = 5;
		float BoxSize = 3;
		float  boxPosX = planeX - planeLength * 0.5 + BoxSize * 1.5;
		float  boxPosY = planeY + planeWidth * 0.5 + BoxSize * 0.5 + 0.1f;
		for(int i=0; i< boxNum; i++)
		{
			auto body = addPolygon(0.8, { {-BoxSize / 2.f, -BoxSize / 2.f}, {-BoxSize / 2.f, BoxSize / 2.f}, {BoxSize / 2.f, BoxSize / 2.f}, {BoxSize / 2.f, -BoxSize / 2.f} }, { boxPosX, boxPosY });
			body->setRestitution(0.2f);
			body->setFriction(0.1, 0.1);
			boxPosY += BoxSize;
		}

		float circleR = 3.8f;
		float  circlePosX = planeX + planeLength * 0.5 - circleR;
		float  circlePosY = planeY + planeWidth * 0.5 + circleR + 36;

		auto Circle = addCircle(1, circleR, Vec2(circlePosX, circlePosY));

		

	
	}

	void World::dowblePendulumScene()
	{
		float planeLength = 1.5f;
		float planeWidth = 18;
		float planeX = 50.f;
		float planeY = 60.f;
		auto plane = addPolygon(1, { {-planeLength / 2.f,-planeWidth / 2.f},  {-planeLength / 2.f,planeWidth / 2.f}, {planeLength / 2.f,planeWidth / 2.f}, {planeLength / 2.f,-planeWidth / 2.f}, }, { planeX, planeY });
		//plane->setRestitution(0.5);
		
		DistancePointPrimitive primitive;
		primitive.bodyA = plane;
		primitive.localPointA = Vec2(0, planeWidth / 2.0f - 1.f );
		primitive.targetPoint = plane->toWorldPoint(primitive.localPointA) + Vec2(0, 0.f);
		primitive.maxDistance = 0.01f;
		primitive.minDistance = 0.f;
		auto joint = new DistanceJoint(primitive);
		this->joinsList.emplace_back(joint);
		

		/*PointJointPrimitive ppm;

		ppm.bodyA = plane;
		ppm.localPointA = Vec2(0, planeWidth / 2.0f);
		ppm.targetPoint = plane->toWorldPoint(ppm.localPointA) + Vec2(0, 10.f);;
		ppm.dampingRatio = 0.8f;
		ppm.frequency = 10;
		ppm.maxForce = 100;
		auto pointJoint = new PointJoint(ppm);
		this->joinsList.emplace_back(pointJoint); */
	}

}