#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include "Body.h"
#include "Matrix.h"

class Constraint {
	public:
		Body* bodyA;
		Body* bodyB;
		
		virtual ~Constraint() = default;

		Matrix GetInverseMatrix() const;
		VecN GetVelocities() const;

		virtual void Solve() {};
};

class DistanceConstraint : public Constraint {
	//Matrix jacobian;
};

class PenetrationConstraint : public Constraint {
	//Matrix jacobian;
};

#endif