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
	this->friction = 0.7f;

	this->mass = mass;
	if (mass != 0.0) {
		this->inverseMass = 1.0f / mass;
	}
	else {
		this->inverseMass = 0.0f;
	}

	momentOfInertia = shape.GetMomentOfInertia() * mass;
	if (momentOfInertia != 0.0f) {
		this->inverseMomentOfInertia = 1.0f / momentOfInertia;
	}
	else {
		this->inverseMomentOfInertia = 0.0f;
	}

	this->shape->UpdateVertices(rotation, position);
}

Body::~Body() {
	delete shape;
}

bool Body::IsStatic() const {
	const float epsilon = 0.005f;
	return fabs(inverseMass) < epsilon;//could be improved - https://floating-point-gui.de/errors/comparison/
}

void Body::ApplyImpulseLinear(const Vec2& impulse) {
	if (IsStatic()) {
		return;
	}

	velocity += impulse * inverseMass;
}

void Body::ApplyImpulseAtPoint(const Vec2& impulse, const Vec2& point) {
	if (IsStatic()) {
		return;
	}

	velocity += impulse * inverseMass;
	angularVelocity += point.Cross(impulse) * inverseMomentOfInertia;
}

void Body::ApplyImpulseAngular(const float impulse) {
	if (IsStatic()) {
		return;
	}

	angularVelocity += impulse * inverseMomentOfInertia;
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

Vec2 Body::LocalSpaceToWorldSpace(const Vec2& point) const {
	Vec2 rotated = point.Rotate(rotation);
	return rotated + position;
}

Vec2 Body::WorldSpaceToLocalSpace(const Vec2& point) const {
	float translatedX = point.x - position.x;
	float translatedY = point.y - position.y;
	float rotatedX = cos(-rotation) * translatedX - sin(-rotation) * translatedY;
	float rotatedY = cos(-rotation) * translatedY + sin(-rotation) * translatedX;
	return Vec2(rotatedX, rotatedY);
}

void Body::IntegrateForces(const float deltaTime) {
	if (IsStatic()) {
		return;
	}

	acceleration = sumForces * inverseMass;
	velocity += acceleration * deltaTime;
	angularAcceleration = sumTorque * inverseMomentOfInertia;
	angularVelocity += angularAcceleration * deltaTime;

	ClearForces();
	ClearTorque();
}

void Body::IntegrateVelocities(const float deltaTime) {
	if (IsStatic()) {
		return;
	}

	position += velocity * deltaTime;
	rotation += angularVelocity * deltaTime;
	shape->UpdateVertices(rotation, position);
}