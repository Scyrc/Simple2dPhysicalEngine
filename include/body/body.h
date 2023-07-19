#pragma once
#include "../common/ctypes.h"
#include <vector>
namespace physicalEngine
{
	enum  BodyType
	{
		defaultType = 0,
		PolygonType = 1
	};
	using VerticesListType = std::vector<Vec2>;
	
	struct BodyColor
	{
		BodyColor()
		{

		}
		BodyColor(BodyColor& other)
		{
			r = other.r;
			g = other.g;
			b = other.b;
		}
		BodyColor(Double r_, Double g_, Double b_)
		{
			r = r_ / 255.0f;
			g = g_ / 255.0f;
			b = b_ / 255.0f;
		}
		Double r, g, b;

	};
	class Body
	{
		protected:
		Double mass{ 1 }; 
		Double density{ 1 };
		Vec2 position{ 0, 0 };
		Vec2 velocity{ 0, 0 };
		Double angularVelocity{ 0 };
		Double angle { 0 };
		Double torques{ 0 };
		Double inertia{0};
		Vec2 force {0, 0};
		Double area = 0.f;
		Double StaticFriction{ 0.8f };
		Double DynamicFriction{ 0.6f };

		Vec2 centroid{0.f, 0.f};

		Double ECC{ 0.1};
		Double restitution = 0.7;


	public:
		Body(Double density_) : mass(density) {}
		BodyColor bodyColor;
		virtual void Draw() = 0;
		int id = -1;
		bool isStatic = false;
	public:
		virtual BodyType getType() const { return defaultType; }
		void setPos(const Vec2& Pos) { position = Pos; }
		Vec2 getPos() const { return position; }
		void setAngle(const Double angle_) { angle = angle_; }

		Vec2 getVel() const { return velocity; }

		Double getStaticFriction() const { return StaticFriction; }


		Double getDynamicFriction() const { return DynamicFriction; }

		void setVel(const Vec2& Vel);

		Double getAnguleVel() const { return angularVelocity; }
		void setAnguleVel(Double anguleVel);

		void setForce(const Vec2& f) { force = f; }
		Vec2 getAcc(Vec2 G) { return (force + G) / mass; }

		Double GetInvInertia() const;

		Double getTorques() const { return torques; }

		Double getRestitution() const { return restitution; }

		virtual Vec2 edge(size_t idx) const = 0;

		virtual size_t getEdgeNums() const = 0;

		virtual Vec2 getCentroidWorldPos() const = 0;

		Double getMassInv() const { return 1.0f / mass; }

		virtual void stepPos(Double dt) = 0;

		virtual void stepVel(Vec2 Gravity, Double dt) = 0;

		virtual void move(Vec2 mtv)=0;

		virtual void moveToPos(const Vec2& pos) = 0;


		virtual void RotateTo(Double angle) = 0;


		virtual bool IsContianPoint(const Vec2& point) = 0;

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

		Polygon(Double density_, const VerticesListType& VerticesList_);

		VerticesListType VerticesList;
		Mat22 RotationMatrix {Mat22::I};

		void GetVerticesWorldPosition(VerticesListType& posList);

		Vec2 operator[](size_t id) const
		{
			id = id % VerticesList.size();
			return RotationMatrix * (VerticesList[id] - centroid) + centroid;
		}

		virtual Vec2 edge(size_t idx) const override;


		virtual void Draw() override;

		virtual size_t getEdgeNums() const override;


		virtual Vec2 getCentroidWorldPos() const override;


		virtual void stepPos(Double dt) override;


		virtual void stepVel(Vec2 Gravity, Double dt) override;

		virtual void move(Vec2 mtv) override;

		virtual void moveToPos(const Vec2& pos) override;

		virtual void RotateTo(Double angle) override;

		virtual bool IsContianPoint(const Vec2& point) override;

	};


}