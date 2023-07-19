#pragma once


#include "body.h"
#include "../collision/bvhtree.h"
#include <mutex>
namespace physicalEngine
{
	class World
	{
		
	public:
		World();
		Polygon* addPolygon(float mass, const VerticesListType& verticesList, const Vec2& position = { 0, 0 }, double rotateAngle = 0 , bool isStatic =false);
		void GenerateTree();
		void step(Double dt);
		void Lock() { mutex_.lock(); }
		void Unlock() { mutex_.unlock(); }
		void draw();

		void initScene(int idx);
		void changeScene(size_t sceneId);

		void addRandomBody(double x, double y);
		void selectBody(double x, double y);
		void dragBodyToPos(double x, double y);
		void clearSelectBody();

		void setBackGround(Double backgroundR_, Double backgroundG_, Double backgroundB_, Double backgroundA_);
		void setDispalyProperties(bool shoWBVH_, bool showContact_);


	protected:
		
		void clearScene();

		void RandomScene();
		void boxScene();


		void RandomRect(double x, double y);
		void RandomCircle(double x, double y);
		void RandomTriangle(double x, double y);
		void Randompolygon(double x, double y);
		void StepVelocity(Double dt);
		void StepPosition(Double dt);

	private:
		Vec2 Gravity{ 0, -9.8};

		BVHTree* bvhTree;

		std::mutex mutex_;

		std::vector<Body*> bodyList;
		Body* selectBodyIns;



		Double backgroundR, backgroundG, backgroundB, backgroundA;

		bool showBVH = false;
		bool showContact = false;
		

		int simulateSpeed = 2;

		size_t sceneIndex = 10000;

	public:
		void setSimluateSpeed(int speed)
		{
			simulateSpeed = speed;
		}

		size_t getBodyNums()
		{
			return bodyList.size();
		}


	};


}

