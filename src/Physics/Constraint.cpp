#include "Constraint.h"

Matrix Constraint::GetInverseMatrix() const {
	Matrix inverseMat(6, 6);

	inverseMat.Zero();

	inverseMat.rows[0][0] = bodyA->inverseMass;
	inverseMat.rows[1][1] = bodyA->inverseMass;
	inverseMat.rows[2][2] = bodyA->inverseMomentOfInertia;

	inverseMat.rows[3][3] = bodyB->inverseMass;
	inverseMat.rows[4][4] = bodyB->inverseMass;
	inverseMat.rows[5][5] = bodyB->inverseMomentOfInertia;

	return inverseMat;
}

VecN Constraint::GetVelocities() const {
	VecN vecN(6);
	vecN.Zero();

	vecN[0] = bodyA->velocity.x;
	vecN[1] = bodyA->velocity.y;
	vecN[2] = bodyA->angularVelocity;
	vecN[3] = bodyB->velocity.x;
	vecN[4] = bodyB->velocity.y;
	vecN[5] = bodyB->angularVelocity;

	return vecN;
}

JointConstraint::JointConstraint() : Constraint(), jacobian(1, 6) {}

JointConstraint::JointConstraint(Body* bodyA, Body* bodyB, const Vec2& anchorPoint) : Constraint(), jacobian(1, 6) {
	this->bodyA = bodyA;
	this->bodyB = bodyB;
	this->pointA = bodyA->WorldSpaceToLocalSpace(anchorPoint);
	this->pointB = bodyB->WorldSpaceToLocalSpace(anchorPoint);
}

void JointConstraint::Solve() {

}