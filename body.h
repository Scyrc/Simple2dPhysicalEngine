#pragma once
#include "ctypes.h"
#include <vector>

namespace physicalEngine
{
	using VerticesListType = std::vector<Vec2>;

	class Body
	{
		protected:

		Double mass{ 1 }; 
		Vec2 position{ 0, 0 };
		Vec2 velocity{ 0, 0 };
		Double angularVelocity{ 0 };
		Double angle { 0 };

		Vec2 force {0, 0};

		Double friction{ 0.5 };

		Double ECC{ 0.1};

	public:
		Body(Double mass_) : mass(mass_) {}

	public:
		void setPos(const Vec2& Pos) { position = Pos; }
		Vec2 getPos() const { return position; }

		Vec2 getVel() const { return velocity; }
		void setVel(const Vec2& Vel) { velocity = Vel; }
		void setForce(const Vec2& f) { force = f; }
		Vec2 getAcc(Vec2 G) { return (force + G) / mass; }

	};


	class Polygon : public Body
	{
	
	public:
		Vec2 centroid;
		Polygon(Double mass_, const VerticesListType& VerticesList_);

		VerticesListType VerticesList;
		Mat22 RotationMatrix {Mat22::I};

		VerticesListType* GetVerticesWorldPosition();

		Vec2 operator[](size_t id) const
		{
			return RotationMatrix * (VerticesList[id] - centroid) + centroid;
		}


		void Draw();
	};


}