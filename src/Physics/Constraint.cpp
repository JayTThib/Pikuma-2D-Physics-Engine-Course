#include "Constraint.h"

Matrix Constraint::GetInverseMassMatrix() const {
	Matrix mat(6, 6);

	mat.Zero();

	mat.rows[0][0] = bodyA->inverseMass;
	mat.rows[1][1] = bodyA->inverseMass;
	mat.rows[2][2] = bodyA->inverseMomentOfInertia;

	mat.rows[3][3] = bodyB->inverseMass;
	mat.rows[4][4] = bodyB->inverseMass;
	mat.rows[5][5] = bodyB->inverseMomentOfInertia;

	return mat;
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

void JointConstraint::PreSolve() {

}

void JointConstraint::Solve() {
	const Vec2 anchorPointWorldPosA = bodyA->LocalSpaceToWorldSpace(pointA);
	const Vec2 anchorPointWorldPosB = bodyB->LocalSpaceToWorldSpace(pointB);

	const Vec2 distBetweenCenterOfMassAndPointA = anchorPointWorldPosA - bodyA->position;
	const Vec2 distBetweenCenterOfMassAndPointB = anchorPointWorldPosB - bodyB->position;

	jacobian.Zero();

	Vec2 jacobianElement1 = (anchorPointWorldPosA - anchorPointWorldPosB) * 2.0f;
	jacobian.rows[0][0] = jacobianElement1.x;//Linear velocity.x of A
	jacobian.rows[0][1] = jacobianElement1.y;//Linear velocity.y of A

	float jacobianElement2 = 2.0f * distBetweenCenterOfMassAndPointA.Cross(anchorPointWorldPosA - anchorPointWorldPosB);
	jacobian.rows[0][2] = jacobianElement2;//Angular velocity of A

	Vec2 jacobianElement3 = (anchorPointWorldPosB - anchorPointWorldPosA) * 2.0f;
	jacobian.rows[0][3] = jacobianElement3.x;//Linear velocity.x of B
	jacobian.rows[0][4] = jacobianElement3.y;//Linear velocity.y of B

	float jacobianElement4 = distBetweenCenterOfMassAndPointB.Cross(anchorPointWorldPosB - anchorPointWorldPosA) * 2.0f;
	jacobian.rows[0][5] = jacobianElement4;//Angular velocity of B

	const VecN velocityVecN = GetVelocities();
	const Matrix inverseMassMatrix = GetInverseMassMatrix();

	const Matrix jacobianTransposed = jacobian.Transpose();

	VecN numerator = jacobian * velocityVecN * -1.0f;
	Matrix denominator = jacobian * inverseMassMatrix * jacobianTransposed;
	
	VecN lambda = Matrix::SolveGaussSeidel(denominator, numerator);

	VecN impulses = jacobianTransposed * lambda;

	bodyA->ApplyImpulseLinear(Vec2(impulses[0], impulses[1]));
	bodyA->ApplyImpulseAngular(impulses[2]);
	bodyB->ApplyImpulseLinear(Vec2(impulses[3], impulses[4]));
	bodyB->ApplyImpulseAngular(impulses[5]);
}

void JointConstraint::PostSolve() {

}