#include "body.h"
#include <gl/glut.h>
#include <iostream>
#include "aabb.h"
namespace physicalEngine
	
{

	void Polygon::calcArea()
	{
		// 求所有三角形的面积之和
		Double areaSum = 0.f;
		for (size_t i = 0; i < VerticesList.size(); ++i)
		{
			auto j = (i + 1) % VerticesList.size();
			areaSum += std::abs(VerticesList[i].cross(VerticesList[j]));
		}
		
		area = areaSum / 2.0;

		std::cout << "area: " << area << " ";
	}

	void Polygon::calcCentroid()
	{
		// 重心 = (各三角形重心 * 其面积) / 总面积
	   // 三角形重心 = 两向量之和 / 3
		Vec2 GravityCenter {0, 0};
		for (size_t i = 0; i < VerticesList.size(); ++i)
		{
			auto j = (i + 1) % VerticesList.size();
			GravityCenter += (VerticesList[i] + VerticesList[j]) * std::abs(VerticesList[i].cross(VerticesList[j]));
		}

		centroid = GravityCenter / 6.0 / area;
		std::cout << "centroid: " << centroid.x << centroid.y << " ";

	}

	void Polygon::calcInertia()  // 转动惯量 = m / 6 * (各三角形面积 * 其(a*a+a*b+b*b)) / (总面积)
	{
		Double acc0 = 0, acc1 = 0;
		for (size_t i = 0; i < VerticesList.size(); ++i) {
			auto a = VerticesList[i], b = VerticesList[(i + 1) % VerticesList.size()];
			auto cross = abs(a.cross(b));
			acc0 += cross * (a.dot(a) + b.dot(b) + a.dot(b));
			acc1 += cross;
		}
		inertia =  mass * acc0 / 6 / acc1;
		std::cout << "inertia: " << inertia << " ";

	}

	void Polygon::init()
	{
		calcArea();
		calcCentroid();
		calcInertia();
	}

	Polygon::Polygon(Double mass_, const VerticesListType& VerticesList_) : Body(mass_), VerticesList(VerticesList_)
	{
		init();
	}

	void Polygon::GetVerticesWorldPosition(VerticesListType& posList)
	{
		posList.resize(VerticesList.size());

		for (int i = 0; i < VerticesList.size(); ++i)
		{
			posList[i] = position + this->operator[](i);
		}
	}

	void Polygon::Draw()
	{
		
		//std::cout << "velocity: " << velocity.x << " " << velocity.y << std::endl;
		//std::cout << "position: " << position.x << " " << position.y << std::endl;

		glBegin(GL_LINE_LOOP);
		glBegin(GL_LINE_STRIP);
		glColor3f(1.0f, 0.0f, 0.0f); // Red
		
		//VerticesListType* list = GetVerticesWorldPosition();
		VerticesListType resList(VerticesList.size());

		for (int i = 0; i < VerticesList.size(); ++i)
		{
			resList[i] = position + this->operator[](i);
		}

		for (auto& v : resList)
		{
			glVertex2d(v.x, v.y);
		}

		glEnd();

		/*AABB box = AABB(this);
		box.draw();*/
	}

	Double Body::GetInvInertia() const
	{
		return inertia == 0 ? 0 : 1.0 / inertia;
	}

}
	