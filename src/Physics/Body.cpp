#include "Body.h"
#include <math.h>
#include <iostream>

Body::Body(const Shape& shape, float x, float y, float mass) {
	this->shape = shape.Clone();
	this->position = Vec2(x, y);
	this->velocity = Vec2(0.0f, 0.0f);
	this->acceleration = Vec2(0.0f, 0.0f);
	this->rotation = 0.0f;
	this->angularVelocity = 0.0f;
	this->angularAcceleration = 0.0f;
	this->sumForces = Vec2(0.0f, 0.0f);
	this->sumTorque = 0.0f;
	this->elasticity = 1.0f;

	this->mass = mass;
	if (mass != 0.0) {
		this->inverseMass = 1.0f / mass;
	}
	else {
		this->inverseMass = 0.0f;
	}

	rotationalInertia = shape.GetMomentOfInertia() * mass;
	if (rotationalInertia != 0.0f) {
		this->inverseRotationalInertia = 1.0f / rotationalInertia;
	}
	else {
		this->inverseRotationalInertia = 0.0f;
	}
}

Body::~Body() {
	delete shape;
}

bool Body::IsStatic() const {
	const float epsilon = 0.005f;
	return fabs(inverseMass) < epsilon;//could be improved - https://floating-point-gui.de/errors/comparison/
}

void Body::IntegrateLinear(float deltaTime) {
	if (IsStatic()) {
		return;
	}

	acceleration = sumForces * inverseMass;
	velocity += acceleration * deltaTime;
	position += velocity * deltaTime;
	ClearForces();
}

void Body::IntegrateAngular(float deltaTime) {
	if (IsStatic()) {
		return;
	}

	angularAcceleration = sumTorque * inverseRotationalInertia;
	angularVelocity += angularAcceleration * deltaTime;
	rotation += angularVelocity * deltaTime;
	ClearTorque();
}

void Body::ApplyImpulse(const Vec2& impulse) {
	if (IsStatic()) {
		return;
	}

	velocity += impulse * inverseMass;
}

void Body::ApplyImpulse(const Vec2& impulse, const Vec2& pointOfImpactDist) {
	if (IsStatic()) {
		return;
	}

	velocity += impulse * inverseMass;
	angularVelocity += pointOfImpactDist.Cross(impulse) * inverseRotationalInertia;
}

void Body::AddForce(const Vec2& force) {
	sumForces += force;
}

void Body::AddTorque(float torque) {
	sumTorque += torque;
}

void Body::ClearForces() {
	sumForces = Vec2(0.0f, 0.0f);
}

void Body::ClearTorque() {
	sumTorque = 0.0f;
}

void Body::Update(float deltaTime) {
	IntegrateLinear(deltaTime);
	IntegrateAngular(deltaTime);

	if (shape->GetType() == POLYGON || shape->GetType() == BOX) {
		PolygonShape* polygonShape = (PolygonShape*)shape;
		polygonShape->UpdateVertices(rotation, position);
	}
}