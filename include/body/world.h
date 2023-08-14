#pragma once


#include "body.h"
#include "joint.h"
#include "distance_joint.h"
#include "revolute_joint.h"
#include "point_joint.h"

#include "circle.h"

#include "../collision/bvhtree.h"
#include <mutex>
namespace physicalEngine
{
	class World
	{
		
	public:
		World();
		Polygon* addPolygon(float density, const VerticesListType& verticesList, const Vec2& position = { 0, 0 }, double rotateAngle = 0 , bool isStatic =false);
		Circle* addCircle(float density,  Double R, const Vec2& position = { 0, 0 }, double rotateAngle = 0, bool isStatic = false);

		void GenerateTree();
		void step(Double dt);
		void Lock() { mutex_.lock(); }
		void Unlock() { mutex_.unlock(); }
		void draw();

		void initScene(int idx);
		void changeScene(size_t sceneId);
		void resetScene(size_t sceneId);

		void addRandomBody(double x, double y);
		void selectBody(double x, double y);
		void dragBodyToPos(double x, double y);
		void clearSelectBody();

		void setBackGround(Double backgroundR_, Double backgroundG_, Double backgroundB_, Double backgroundA_);
		void setDispalyProperties(bool shoWBVH_, bool showContact_);
		void setGravity(Double x, Double y);


	protected:
		
		void clearScene();

		void RandomScene();
		void boxScene();
		void jointScene();
		void softJointScene();
		void wreckingBall();

		void frictionScene();
		void DominoScene();
		void fallDownScene();

		void restitutionScene();

		void seesawScene();
		void dowblePendulumScene();

		void RandomRect(double x, double y);
		void RandomCircle(double x, double y);
		void RandomTriangle(double x, double y);
		void Randompolygon(double x, double y);
		void StepVelocity(Double dt);
		void StepPosition(Double dt);


	private:
		Vec2 Gravity{ 0, -9.8};
		int velocityInterationNumPreFrame = 8;
		int positionInterationNumPreFrame = 6;

		BVHTree* bvhTree;

		std::mutex mutex_;

		std::vector<Body*> bodyList;
		std::vector<Joint*> joinsList;
		Body* selectBodyIns;



		Double backgroundR, backgroundG, backgroundB, backgroundA;

		bool showBVH = false;
		bool showContact = false;
		

		int simulateSpeed = 2;
		bool pauseWorld = false;

		size_t sceneIndex = 10000;

		//½ð×ÖËþ³¡¾°
		int pyramidLayerNum = 6;
		float pyramidRestitution = 0.f;
		float pyramidBoxSize = 2.5f;

		//Å£¶Ù°Ú³¡¾°
		int newtonPendunlumCircleNum = 5;
		float newtonPendunlumCircleRadius = 2.f;
		float newtonPendunlumLineLength = 20.f;
		float newtonPendunlumRestitution = 1.0f;

		//ËÙ¶È»Ö¸´³¡¾°
		int restitutionCircleNum = 10.f;
		float restitutionCircleRadius = 2.f;


		//ÇÅ³¡¾°
		int BridgejointNum = 21;
		float BridgejointWidth = 1.5f;

		int BridgeboxLayerNum = 3;
		int BridgeboxNumPreLayer = 3;
		float BridgeboxSize = 2.2f;





	public:
		void setSimluateSpeed(int speed)
		{
			simulateSpeed = speed;
		}

		void setPause(bool bPause)
		{
			pauseWorld = bPause;
		}

		void setInterationNum(int vel, int pos)
		{
			velocityInterationNumPreFrame = vel;
			positionInterationNumPreFrame = pos;
		}

		size_t getBodyNums()
		{
			return bodyList.size();
		}

		void setpyramidParam(int layerNum, float restitution, float boxSize)
		{
			pyramidLayerNum = layerNum;
			pyramidRestitution = restitution;
			pyramidBoxSize = boxSize;
		}

		void setnewtonPendunlumParam(int CircleNum, float CircleRadius, float LineLength, float Restitution)
		{
			newtonPendunlumCircleNum = CircleNum;
			newtonPendunlumCircleRadius = CircleRadius;
			newtonPendunlumLineLength = LineLength;
			newtonPendunlumRestitution = Restitution;
		}

		void setrestitutionParam(int CircleNum, float CircleRadius)
		{
			restitutionCircleNum = CircleNum;
			restitutionCircleRadius = CircleRadius;
		}

		void setBridgeParam(int jointNum, float jointWidth, int boxLayerNum, int boxNumPreLayer, float boxSize)
		{
			BridgejointNum = jointNum;
			BridgejointWidth = jointWidth;

			BridgeboxLayerNum = boxLayerNum;
			BridgeboxNumPreLayer = boxNumPreLayer;
			BridgeboxSize = boxSize;
		}

	};


}

