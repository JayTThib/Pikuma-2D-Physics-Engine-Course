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

	//Pointer to the shape/geometry of this rigid body
	Shape* shape = nullptr;

	Body(const Shape& shape, float x, float y, float mass);
	~Body();

	void AddForce(const Vec2& force);
	void AddTorque(float torque);
	void ClearForces();
	void ClearTorque();

	void IntegrateLinear(float deltaTime);
	void IntegrateAngular(float deltaTime);

	void Update(float deltaTime);
};

#endif