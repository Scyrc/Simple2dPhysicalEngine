#include "world.h"


namespace physicalEngine
{

	physicalEngine::Polygon* World::addPolygon(float mass, const VerticesListType& verticesList, const Vec2& position /*= { 0, 0 }*/)
	{
		auto body = new Polygon(mass, verticesList);
		body->setPos(position);
		bodyList.push_back(body);
		return body;
	}

	void World::step()
	{
		Double dt = 1.0 / 60.0;
		StepVelocity(dt);
		StepPosition(dt);
		for (auto& body : bodyList)
		{
			body->Draw();
		}
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