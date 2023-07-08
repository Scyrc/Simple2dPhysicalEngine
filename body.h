#pragma once
#include "ctypes.h"
#include <vector>

namespace physicalEngine
{
	enum  BodyType
	{
		defaultType = 0,
		PolygonType = 1
	};
	using VerticesListType = std::vector<Vec2>;

	class Body
	{
		protected:
		Double mass{ 1 }; 
		Vec2 position{ 0, 0 };
		Vec2 velocity{ 0, 0 };
		Double angularVelocity{ 0 };
		Double angle { 0 };
		Double torques{ 0 };
		Double inertia;
		Vec2 force {0, 0};
		Double area;
		Double friction{ 0.5 };
		Vec2 centroid;

		Double ECC{ 0.1};

	public:
		Body(Double mass_) : mass(mass_) {}

	public:
		virtual BodyType getType() const { return defaultType; }
		void setPos(const Vec2& Pos) { position = Pos; }
		Vec2 getPos() const { return position; }

		Vec2 getVel() const { return velocity; }
		void setVel(const Vec2& Vel) { velocity = Vel; }

		Double getAnguleVel() const { return angularVelocity; }
		void setAnguleVel(Double anguleVel) { angularVelocity = anguleVel; }

		void setForce(const Vec2& f) { force = f; }
		Vec2 getAcc(Vec2 G) { return (force + G) / mass; }

		Double GetInvInertia() const;

		Double getTorques() const { return torques; }

	};


	class Polygon : public Body
	{
	protected:
		void calcArea();
		void calcCentroid();
		void calcInertia();
		void init();
	public:
		virtual BodyType getType()  const override { return PolygonType; }

		Polygon(Double mass_, const VerticesListType& VerticesList_);

		VerticesListType VerticesList;
		Mat22 RotationMatrix {Mat22::I};

		void GetVerticesWorldPosition(VerticesListType& posList);

		Vec2 operator[](size_t id) const
		{
			return RotationMatrix * (VerticesList[id] - centroid) + centroid;
		}


		void Draw();
	};


}