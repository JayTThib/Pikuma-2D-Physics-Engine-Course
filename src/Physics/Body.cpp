#include "Body.h"
#include <iostream>

Body::Body(float x, float y, float mass) {
	this->position = Vec2(x, y);
	this->mass = mass;
	if (mass != 0.0) {
		this->invMass = 1.0f / mass;
	}
	else {
		this->invMass = 0.0f;
	}
	
}

Body::~Body() {
	
}

void Body::Integrate(float deltaTime) {
	acceleration = sumForces * invMass;
	velocity += acceleration * deltaTime;
	position += velocity * deltaTime;
	ClearForces();
}

void Body::AddForce(const Vec2& force) {
	sumForces += force;
}

void Body::ClearForces() {
	sumForces = Vec2(0.0f, 0.0f);
}