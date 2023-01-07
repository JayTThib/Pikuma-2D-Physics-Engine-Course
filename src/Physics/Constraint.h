#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include "Body.h"

class Constraint {
	public:
		Body* bodyA;
		Body* bodyB;

		//MatMN GetInverseMass();
		//VecN vec;

		void Solve();
};

#endif