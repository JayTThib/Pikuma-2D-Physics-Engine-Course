#ifndef BODY_H
#define BODY_H

#include "Vec2.h"
#include "Shape.h"

struct Body {
	bool isColliding = false;

	//Linear motion
	Vec2 position;
	Vec2 velocity;
	Vec2 acceleration;

	//Angular motion
	float rotation;//Radians
	float angularVelocity;
	float angularAcceleration;

	//Forces and torque
	Vec2 sumForces;
	float sumTorque;

	//Mass and Moment of Inertia
	float mass;
	float inverseMass;
	float rotationalInertia;
	float inverseRotationalInertia;

	float elasticity;//Coefficient of restitution (from 0 to 1. Perfect collision elasticity is 1, perfect inelastic collision is 0).

	Shape* shape = nullptr;

	Body(const Shape& shape, float x, float y, float mass);
	~Body();

	bool IsStatic() const;

	void AddForce(const Vec2& force);
	void AddTorque(float torque);
	void ClearForces();
	void ClearTorque();

	void ApplyImpulse(const Vec2& impulse);

	void IntegrateLinear(float deltaTime);
	void IntegrateAngular(float deltaTime);

	void Update(float deltaTime);
};

#endif