#include "../../include/body/body.h"
#include "iostream"

int mainTEST()
{
	physicalEngine::VerticesListType verticesList{ {-1, -0.5},  {-1, 0.5} , {1, 0.5} , {1, -0.5} };
	auto body = new physicalEngine::Polygon(1, verticesList);
	body->setPos({ 50, 50 });

	physicalEngine::Vec2 localP = body->toLocalPoint({ 51, 50.5});
	body->setAngle(physicalEngine::Constant::HalfPi);
	physicalEngine::Vec2 worldP = body->toWorldPoint(localP);

	std::cout << localP.x << " ," << localP.y << std::endl;
	std::cout << worldP.x << " ," << worldP.y << std::endl;

	return 0;
}