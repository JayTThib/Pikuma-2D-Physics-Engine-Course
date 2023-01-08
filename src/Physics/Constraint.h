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

		virtual void PreSolve() {}
		virtual void Solve() {}
		virtual void PostSolve{}
};

class JointConstraint : public Constraint {
	private:
		Matrix jacobian;

	public:
		JointConstraint();
		JointConstraint(Body* bodyA, Body* bodyB, const Vec2& anchorPoint);
		void PreSolve() override;
		void Solve() override;
		void PostSolve() override;
};

class PenetrationConstraint : public Constraint {
	Matrix jacobian;
	//Solve() override
};

#endif