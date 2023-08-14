#include "../../include/body/circle.h"
#include <gl/glut.h>
#include <iostream>
namespace physicalEngine
{

	

	Circle::Circle(Double density_, const Vec2& position_, const Double radius_) : Body(density_),radius(radius_)
	{
		position = position_;
		positionShow = position * 10;
		radiusShow = radius * 10;
		area = pi * radius * radius;
		mass = area * density;
		std::cout << "mass: " << mass << " ";
		centroid = Vec2(0, 0);
		inertia = mass * radius * radius * 0.5f;

		topPoint = centroid + Vec2(0, radius);

		topPointForDraw = RotationMatrix * (topPoint - centroid) + centroid + position;
	}

	void Circle::Draw()
	{
		positionShow = position * 10;

		glBegin(GL_POLYGON);
		//glColor3f(bodyColor.r, bodyColor.g, bodyColor.b);
		glColor4f(bodyColor.r, bodyColor.g, bodyColor.b, 0.6f);

		if (isSleep)
			glColor3f(0.f, 0.f, 0.f);
		static int n = 300;
		for (int i = 0; i < n; i++)
		{
			glVertex2d(radiusShow * cos(2 * pi / n * i) + positionShow.x, radiusShow * sin(2 * pi / n * i) + positionShow.y);
		}
		glEnd();

		glBegin(GL_LINE_LOOP);
		glLineWidth(3);
		//glColor3f(1, 1, 1);
		glColor4f(bodyColor.r, bodyColor.g, bodyColor.b, 1.f);

		for (int i = 0; i < n; i++)
		{
			glVertex2d(radiusShow * cos(2 * pi / n * i) + positionShow.x, radiusShow * sin(2 * pi / n * i) + positionShow.y);
		}
		glEnd();

		glBegin(GL_LINE_LOOP);
		glLineWidth(6);
		//glColor3f(1, 1, 1);
		glColor4f(bodyColor.r, bodyColor.g, bodyColor.b, 1.f);
		glVertex2d(positionShow.x, positionShow.y);
		glVertex2d(topPointForDraw.x * 10, topPointForDraw.y * 10);
		glEnd();
	}
	BodyType Circle::getType() const
	{
		return BodyType::CircleType;
	}

	Vec2 Circle::edge(size_t idx) const
	{
		return Vec2();
	}

	size_t Circle::getEdgeNums() const
	{
		return 0;
	}

	Vec2 Circle::getCentroidWorldPos() const
	{
		
		return centroid + position;
	}
	
	void Circle::stepPos(Double dt)
	{
		Body::stepPos(dt);

		topPointForDraw = RotationMatrix * (topPoint - centroid) + centroid + position;

		
	}

	/*
	void Circle::stepVel(Vec2 GravityAcc, Double dt)
	{
		if (isStatic) return;
		

		if (isSleep && (!lastPosition.fuzzyEqual(position, 1e-4 / dt) || !twoDMath::nearlyEqual(lastAngle, angle, 1e-5 / dt)))
		{
			isSleep = false;
		}

		//if (isSleep) return;

		Double lvd = 1.0f / (1.0f + dt * linearVelocityDamping);
		Double avd = 1.0f / (1.0f + dt * angularVelocityDamping);

		velocity += getAcc(GravityAcc * mass) * dt;
		velocity = velocity * lvd;

		angularVelocity += torques * GetInvInertia() * dt;
		angularVelocity *= avd;
	}
	*/

	void Circle::move(Vec2 mtv)
	{
		if (isStatic) return;
		position += mtv;
	}

	void Circle::moveToPos(const Vec2& pos)
	{
		if (isStatic) return;
		position = pos;
	}

	void Circle::RotateTo(Double angle_)
	{
		//angle = angle_;
		//RotationMatrix.rotate(angle);
	}

	bool Circle::IsContianPoint(const Vec2& point)
	{
		return false;
	}
}