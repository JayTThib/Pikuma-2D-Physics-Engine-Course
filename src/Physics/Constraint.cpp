#include "Constraint.h"
#include <algorithm>

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

JointConstraint::JointConstraint() : Constraint(), jacobian(1, 6), cachedLambda(1), bias(0.0f) {
	cachedLambda.Zero();
}

JointConstraint::JointConstraint(Body* bodyA, Body* bodyB, const Vec2& anchorPoint) : Constraint(), jacobian(1, 6), cachedLambda(1), bias(0.0f) {
	this->bodyA = bodyA;
	this->bodyB = bodyB;
	this->pointA = bodyA->WorldSpaceToLocalSpace(anchorPoint);
	this->pointB = bodyB->WorldSpaceToLocalSpace(anchorPoint);
	cachedLambda.Zero();
}

void JointConstraint::PreSolve(const float deltaTime) {
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
	
	//Warm starting (apply cachedLambda)
	const Matrix jacobianTransposed = jacobian.Transpose();
	VecN impulses = jacobianTransposed * cachedLambda;

	bodyA->ApplyImpulseLinear(Vec2(impulses[0], impulses[1]));
	bodyA->ApplyImpulseAngular(impulses[2]);
	bodyB->ApplyImpulseLinear(Vec2(impulses[3], impulses[4]));
	bodyB->ApplyImpulseAngular(impulses[5]);

	//Compute the bias term (baumgarte stabilization)
	const float beta = 0.1f;
	float positionalError = (anchorPointWorldPosB - anchorPointWorldPosA).Dot(anchorPointWorldPosB - anchorPointWorldPosA);
	positionalError = std::max(0.0f, positionalError - 0.01f);
	bias = (beta / deltaTime) * positionalError;
}

void JointConstraint::Solve() {
	const VecN velocityVecN = GetVelocities();
	const Matrix inverseMassMatrix = GetInverseMassMatrix();

	const Matrix jacobianTransposed = jacobian.Transpose();

	//Compute lambda using Ax=b (Gauss-Seidel method)
	Matrix leftHandSide = jacobian * inverseMassMatrix * jacobianTransposed;//A
	VecN rightHandSide = jacobian * velocityVecN * -1.0f;//b
	rightHandSide[0] -= bias;
	VecN lambda = Matrix::SolveGaussSeidel(leftHandSide, rightHandSide);
	cachedLambda += lambda;

	VecN impulses = jacobianTransposed * lambda;

	bodyA->ApplyImpulseLinear(Vec2(impulses[0], impulses[1]));
	bodyA->ApplyImpulseAngular(impulses[2]);
	bodyB->ApplyImpulseLinear(Vec2(impulses[3], impulses[4]));
	bodyB->ApplyImpulseAngular(impulses[5]);
}

void JointConstraint::PostSolve() {

}

PenetrationConstraint::PenetrationConstraint(): Constraint(), jacobian(2, 6), cachedLambda(2), bias(0.0f) {
	cachedLambda.Zero();
	friction = 0.0f;
}

PenetrationConstraint::PenetrationConstraint(Body* bodyA, Body* bodyB, const Vec2& collisionPointA, const Vec2& collisionPointB, const Vec2& normal): Constraint(), jacobian(2, 6), cachedLambda(2), bias(0.0f) {
	this->bodyA = bodyA;
	this->bodyB = bodyB;
	this->pointA = bodyA->WorldSpaceToLocalSpace(collisionPointA);
	this->pointB = bodyB->WorldSpaceToLocalSpace(collisionPointB);
	this->normal = bodyA->WorldSpaceToLocalSpace(normal);
	cachedLambda.Zero();
}

void PenetrationConstraint::PreSolve(const float deltaTime) {
	const Vec2 worldPointA = bodyA->LocalSpaceToWorldSpace(pointA);
	const Vec2 worldPointB = bodyB->LocalSpaceToWorldSpace(pointB);
	Vec2 worldNormal = bodyA->LocalSpaceToWorldSpace(normal);

	const Vec2 distBetweenCenterOfMassAndPointA = worldPointA - bodyA->position;
	const Vec2 distBetweenCenterOfMassAndPointB = worldPointB - bodyB->position;

	jacobian.Zero();

	Vec2 jacobianElement1 = -worldNormal;
	jacobian.rows[0][0] = jacobianElement1.x;//Linear velocity.x of A
	jacobian.rows[0][1] = jacobianElement1.y;//Linear velocity.y of A

	float jacobianElement2 = -distBetweenCenterOfMassAndPointA.Cross(worldNormal);
	jacobian.rows[0][2] = jacobianElement2;//Angular velocity of A

	Vec2 jacobianElement3 = worldNormal;
	jacobian.rows[0][3] = jacobianElement3.x;//Linear velocity.x of B
	jacobian.rows[0][4] = jacobianElement3.y;//Linear velocity.y of B

	float jacobianElement4 = distBetweenCenterOfMassAndPointB.Cross(worldNormal);
	jacobian.rows[0][5] = jacobianElement4;//Angular velocity of B
	
	//Populate the second row of the jacobian (friction)
	friction = std::max(bodyA->friction, bodyB->friction);
	if (friction > 0.0) {
		Vec2 tangentVec = worldNormal.Normal();
		jacobian.rows[1][0] = -tangentVec.x;
		jacobian.rows[1][1] = -tangentVec.y;
		jacobian.rows[1][2] = -distBetweenCenterOfMassAndPointA.Cross(tangentVec);
		jacobian.rows[1][3] = tangentVec.x;
		jacobian.rows[1][4] = tangentVec.y;
		jacobian.rows[1][5] = distBetweenCenterOfMassAndPointB.Cross(tangentVec);
	}

	//Warm starting (apply cachedLambda)
	const Matrix jacoTransposed = jacobian.Transpose();
	VecN impulses = jacoTransposed * cachedLambda;

	bodyA->ApplyImpulseLinear(Vec2(impulses[0], impulses[1]));
	bodyA->ApplyImpulseAngular(impulses[2]);
	bodyB->ApplyImpulseLinear(Vec2(impulses[3], impulses[4]));
	bodyB->ApplyImpulseAngular(impulses[5]);

	//Compute the bias term (baumgarte stabilization)
	const float beta = 0.2f;
	float positionalError = (worldPointB - worldPointA).Dot(-worldNormal);
	positionalError = std::min(0.0f, positionalError + 0.01f);
	bias = (beta / deltaTime) * positionalError;
}

void PenetrationConstraint::Solve() {
	const VecN velocityVecN = GetVelocities();
	const Matrix inverseMassMatrix = GetInverseMassMatrix();
	const Matrix jacobianTransposed = jacobian.Transpose();

	//Compute lambda using Ax=b (Gauss-Seidel method)
	Matrix leftHandSide = jacobian * inverseMassMatrix * jacobianTransposed;//A
	VecN rightHandSide = jacobian * velocityVecN * -1.0f;//b
	rightHandSide[0] -= bias;

	VecN lambda = Matrix::SolveGaussSeidel(leftHandSide, rightHandSide);

	VecN oldLambda = cachedLambda;
	cachedLambda += lambda;
	cachedLambda[0] = (cachedLambda[0] < 0.0f) ? 0.0f : cachedLambda[0];

	if (friction > 0.0f) {
		const float maxFriction = cachedLambda[0] * friction;
		cachedLambda[1] = std::clamp(cachedLambda[1], -maxFriction, maxFriction);
	}

	lambda = cachedLambda - oldLambda;

	VecN impulses = jacobianTransposed * lambda;

	bodyA->ApplyImpulseLinear(Vec2(impulses[0], impulses[1]));
	bodyA->ApplyImpulseAngular(impulses[2]);
	bodyB->ApplyImpulseLinear(Vec2(impulses[3], impulses[4]));
	bodyB->ApplyImpulseAngular(impulses[5]);
}

void PenetrationConstraint::PostSolve() {

}