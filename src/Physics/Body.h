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

	float rotation;//Radians
	float angularVelocity;
	float angularAcceleration;

	Vec2 sumForces;
	float sumTorque;

	float mass;
	float inverseMass;
	float momentOfInertia;
	float inverseMomentOfInertia;

	float elasticity;//Coefficient of restitution (from 0 to 1. Perfect collision elasticity is 1, perfect inelastic collision is 0).
	float friction;//Coefficient of friction

	Shape* shape = nullptr;

	Body(const Shape& shape, float x, float y, float mass);
	~Body();

	bool IsStatic() const;

	void AddForce(const Vec2& force);
	void AddTorque(float torque);
	void ClearForces();
	void ClearTorque();

	Vec2 LocalSpaceToWorldSpace(const Vec2& point) const;
	Vec2 WorldSpaceToLocalSpace(const Vec2& point) const;

	void ApplyImpulse(const Vec2& impulse);
	void ApplyImpulse(const Vec2& impulse, const Vec2& pointOfImpactDist);

	void IntegrateLinear(float deltaTime);
	void IntegrateAngular(float deltaTime);

	void Update(float deltaTime);
};

#endif