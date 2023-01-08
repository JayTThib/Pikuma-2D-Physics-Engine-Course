#include "World.h"
#include "Constants.h"
#include "Contact.h"
#include "CollisionDetection.h"

World::World(float gravity) {
	this->gravity = -gravity;
}

World::~World() {
	for (Body* body : bodies) {
		delete body;
	}
}

void World::AddBody(Body* body) {
	bodies.push_back(body);
}

std::vector<Body*>& World::GetBodies() {
	return bodies;
}

void World::AddForce(const Vec2& force) {
	forces.push_back(force);
}

void World::AddTorque(float torque) {
	torques.push_back(torque);
}

void World::Update(float deltaTime) {
	for (Body* body : bodies) {
		Vec2 weight = Vec2(0.0f, body->mass * gravity * PIXELS_PER_METER);
		body->AddForce(weight);

		for (Vec2 force : forces) {
			body->AddForce(force);
		}

		for (float torque : torques) {
			body->AddTorque(torque);
		}
	}

	for (auto body : bodies) {
		body->Update(deltaTime);
	}

	CheckCollisions();
}

void World::CheckCollisions() {
	for (Body*& body : bodies) {
		body->isColliding = false;
	}

	for (int i = 0; i <= bodies.size() - 1; i++) {
		for (int j = i + 1; j < bodies.size(); j++) {
			Body* body1 = bodies[i];
			Body* body2 = bodies[j];

			Contact contact;
			if (CollisionDetection::IsColliding(body1, body2, contact)) {
				body1->isColliding = true;
				body2->isColliding = true;
				contact.ResolveCollision();
			}
		}
	}
}