#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include "Body.h"
#include "Matrix.h"

class Constraint {
	public:
		Body* bodyA;
		Body* bodyB;
		
		Vec2 pointA;//The anchor point in bodyA's local space
		Vec2 pointB;//The anchor point in bodyB's local space

		virtual ~Constraint() = default;

		Matrix GetInverseMassMatrix() const;
		VecN GetVelocities() const;

		virtual void PreSolve(const float deltaTime) {}
		virtual void Solve() {}
		virtual void PostSolve() {}
};

class JointConstraint : public Constraint {
	private:
		Matrix jacobian;
		VecN cachedLambda;
		float bias;

	public:
		JointConstraint();
		JointConstraint(Body* bodyA, Body* bodyB, const Vec2& anchorPoint);
		void PreSolve(const float deltaTime) override;
		void Solve() override;
		void PostSolve() override;
};

class PenetrationConstraint : public Constraint {
	private: 
		Matrix jacobian;
		VecN cachedLambda;
		float bias;
		Vec2 normal;
	
	public:
		PenetrationConstraint();
		PenetrationConstraint(Body* bodyA, Body* bodyB, const Vec2& collisionPointA, const Vec2& collisionPointB, const Vec2& normal);
		void PreSolve(const float deltaTime) override;
		void Solve() override;
		void PostSolve() override;
};

#endif