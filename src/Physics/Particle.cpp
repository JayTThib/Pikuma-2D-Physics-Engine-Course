#include "Particle.h"
#include <iostream>

Particle::Particle(float x, float y, float mass) {
	this->position = Vec2(x, y);
	this->mass = mass;
	if (mass != 0.0) {
		this->invMass = 1.0f / mass;
	}
	else {
		this->invMass = 0.0f;
	}
	
}

Particle::~Particle() {
	
}

void Particle::Integrate(float deltaTime) {
	//Integrate using the Euler method
	acceleration = sumForces * invMass;
	velocity += acceleration * deltaTime;
	position += velocity * deltaTime;
	ClearForces();
}

void Particle::AddForce(const Vec2& force) {
	sumForces += force;
}

void Particle::ClearForces() {
	sumForces = Vec2(0.0f, 0.0f);
}