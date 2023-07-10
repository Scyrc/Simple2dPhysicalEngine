#include "world.h"


namespace physicalEngine
{

	World::World()
	{
		bvhTree = new BVHTree();
	}

	physicalEngine::Polygon* World::addPolygon(float mass, const VerticesListType& verticesList, const Vec2& position /*= { 0, 0 }*/)
	{
		auto body = new Polygon(mass, verticesList);
		body->setPos(position);
		bodyList.push_back(body);
		return body;
	}

	void World::GenerateTree()
	{
		bvhTree->generate(bodyList);
	}

	void World::step()
	{
		Double dt = 1.0 / 60.0;
		bodyList[0]->setVel(bodyList[0]->getVel() + Vec2(1, 0));
		StepVelocity(dt);
		StepPosition(dt);
		for (auto& body : bodyList)
		{
			body->Draw();
		}
		bvhTree->update(bodyList);
		bvhTree->draw();
	}

	void World::StepVelocity(Double dt)
	{
		for (auto& body : bodyList)
		{
			body->setVel(body->getVel() + body->getAcc(Gravity) * dt);
			body->setAnguleVel(body->getAnguleVel() + body->getTorques() * body->GetInvInertia() * dt);
		}
	}

	void World::StepPosition(Double dt)
	{
		for (auto& body : bodyList)
		{
			body->setPos(body->getPos() + body->getVel() * dt);
		}
	}

}