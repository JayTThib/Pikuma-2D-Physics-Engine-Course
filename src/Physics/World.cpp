#include "World.h"
#include "Constants.h"

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
	}
}

void World::CheckCollisions() {

}