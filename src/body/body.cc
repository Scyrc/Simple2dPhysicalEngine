#include "../../include/body/body.h"
#include <gl/glut.h>
#include <iostream>
#include "../../include/body/aabb.h"
#include <ctime>
#include <cstdlib>
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
		mass = area * density;
		std::cout << "area: " << area << " ";
		std::cout << "mass: " << mass << " ";

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
		std::cout << "centroid: " << centroid.x << "," << centroid.y << " ";

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
		std::cout << "inertia: " << inertia << std::endl;

	}

	void Polygon::init()
	{
		calcArea();
		calcCentroid();
		calcInertia();
	}

	Polygon::Polygon(Double density_, const VerticesListType& VerticesList_) : Body(density_), VerticesList(VerticesList_)
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

	Vec2 Polygon::edge(size_t idx) const
	{
		return  this->operator[](idx + 1) - this->operator[](idx);;
	}

	void Polygon::adjacentEdge(size_t idx, Vec2& edge1, Vec2& edge2) const
	{
		edge2 = this->edge(idx);

		if (idx == 0)
		{
			edge1 = this->edge(VerticesList.size() - 1);
		}
		else
		{
			edge1 = this->edge(idx - 1);
		}
	}


	void Polygon::Draw()
	{
		
		//std::cout << "velocity: " << velocity.x << " " << velocity.y << std::endl;
		//std::cout << "position: " << position.x << " " << position.y << std::endl;

		//glBegin(GL_LINE_LOOP);
		//glBegin(GL_LINE_ST12  RIP);

		glBegin(GL_POLYGON);
		//glColor3f(bodyColor.r, bodyColor.g, bodyColor.b);
		glColor4f(bodyColor.r, bodyColor.g, bodyColor.b, 0.6f);
		if(isSleep)
			glColor3f(0.f, 0.f, 0.f);
		//VerticesListType* list = GetVerticesWorldPosition();
		VerticesListType resList(VerticesList.size());
		for (int i = 0; i < VerticesList.size(); ++i)
		{
			resList[i] = position + this->operator[](i);
			resList[i] = resList[i] * 10;
		}

		for (auto& v : resList)
		{

			glVertex2d(v.x, v.y);
		}
		glEnd();

		glBegin(GL_LINE_LOOP);
		glLineWidth(6.0f);
		glColor4f(bodyColor.r, bodyColor.g, bodyColor.b, 1.f);

		for (auto& v : resList)
		{
			glVertex2d(v.x, v.y);
		}
		glEnd();


		auto centroidWorldPos = centroid + position;
		
		centroidWorldPos = centroidWorldPos * 10;
		glColor3f(0.0f, 1.0f, 0.0f);    //设置绘图颜色
		glRectf(centroidWorldPos.x - 2.f, centroidWorldPos.y - 2.f, centroidWorldPos.x + 2.f, centroidWorldPos.y + 2.f);  //绘制矩形

	}

	size_t Polygon::getEdgeNums() const
	{
		return VerticesList.size();
	}

	physicalEngine::Vec2 Polygon::getCentroidWorldPos() const
	{
		return centroid + position;
	}

	void Body::stepPos(Double dt)
	{
		if (isStatic) return;

		lastPosition = position;
		lastAngle = angle;

		position += velocity * dt;
		angle += angularVelocity * dt;
		RotationMatrix.rotate(angle);

		if (fallDownMode )
		{
			if(position.y < 0.f)
				position.y += Constant::sceneTop;

			if (position.x < 0.f)
				position.x += Constant::sceneR;

			if (position.x > Constant::sceneR)
				position.x -= Constant::sceneR;
		}

		if (!enableSleep) return;
		if (isSleep) return;

		if (lastPosition.fuzzyEqual(position, 1e-4 / dt) && twoDMath::nearlyEqual(lastAngle, angle, 1e-5 / dt))
		{
			++sleepCount;
			//std::cout << "sleepCount:" << sleepCount << std::endl;
		}
		else
		{
			sleepCount = 0;
		}

		if (sleepCount >= sleepMax)
		{
			sleepCount = 0;
			isSleep = true;
			velocity = Vec2(0.f, 0.f);
			angularVelocity = 0.f;
		}

		

	}

	void Body::stepVel(Vec2 GravityAcc, Double dt)
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

	void Body::initColor()
	{
		int x = rand() % 5;

		switch (x)
		{
		case 0:
			bodyColor = BodyColor(245, 90, 60);
			break;
		case 1:
			bodyColor = BodyColor(236, 236, 209);
			break;
		case 2:
			bodyColor = BodyColor(6,  62,123);
			break;
		case 3:
			bodyColor = BodyColor(242, 150, 72);
			break;
		case 4:
			bodyColor = BodyColor(245, 210, 89);
			break;
		default:
			break;
		}
		/*
		switch (x)
		{
		case 0:
			bodyColor = BodyColor(238, 63, 77);
			break;
		case 1:
			bodyColor = BodyColor(129, 60, 133);
			break;
		case 2:
			bodyColor = BodyColor(18, 107, 174);
			break;
		case 3:
			bodyColor = BodyColor(27, 167, 132);
			break;
		case 4:
			bodyColor = BodyColor(254, 215, 26);
			break;
		case 5:
			bodyColor = BodyColor(254, 186, 7);
			break;
		case 6:
			bodyColor = BodyColor(250, 126, 35);
			break;
		case 7:
			bodyColor = BodyColor(243, 59, 31);
			break;
		case 8:
			bodyColor = BodyColor(250, 142, 22);
			break;
		case 9:
			bodyColor = BodyColor(91, 174, 35);
			break;
		case 10:
			bodyColor = BodyColor(26, 104, 64);
			break;
		case 11:
			bodyColor = BodyColor(16, 174, 194);
			break;
		case 12:
			bodyColor = BodyColor(15, 89, 164);
			break;
		case 13:
			bodyColor = BodyColor(129, 192, 148);
			break;
		default:
			break;
		}
		*/
	}

	Body::Body(Double density_) : density(density_)
	{
		initColor();
	}

	void Body::setVel(const Vec2& Vel)
	{
		if (isStatic) return;
		velocity = Vel;
	}

	void Body::setAnguleVel(Double anguleVel)
	{
		if (isStatic) return;
		angularVelocity = anguleVel;
	}

	Double Body::GetInvInertia() const
	{
		return inertia == 0 ? 0 : 1.0 / inertia;
	}


	void Body::applyImpulse(const Vec2& impulse, const Vec2& r)
	{
		if (isStatic) return;
		velocity += impulse * getMassInv();
		angularVelocity += GetInvInertia() * r.cross(impulse);
	}

	Vec2 Body::toLocalPoint(const Vec2& point) const
	{
		Mat22 RotateMTemp;
		RotateMTemp.rotate(-angle);

		return RotateMTemp * (point - getCentroidWorldPos());
	}

	physicalEngine::Vec2 Body::toWorldPoint(const Vec2& point) const
	{
		return RotationMatrix * (point) + centroid + position;
	}

	void  Polygon::move(Vec2 mtv)
	{
		if (isStatic) return;
		position += mtv;
	}

	void Polygon::moveToPos(const Vec2& pos)
	{
		if (isStatic) return;
		position = pos;
	}

	void Polygon::RotateTo(Double angle_)
	{
		angle = angle_;
		RotationMatrix.rotate(angle);
	}

	bool Polygon::IsContianPoint(const Vec2& point)
	{
		return false;
	}

}
	