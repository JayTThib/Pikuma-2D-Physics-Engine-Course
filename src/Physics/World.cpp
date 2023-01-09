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

	for (Constraint* constraint : constraints) {
		delete constraint;
	}
}

void World::AddBody(Body* body) {
	bodies.push_back(body);
}

std::vector<Body*>& World::GetBodies() {
	return bodies;
}

void World::AddConstraint(Constraint* constraint) {
	constraints.push_back(constraint);
}

std::vector<Constraint*>& World::GetConstraints() {
	return constraints;
}

void World::AddForce(const Vec2& force) {
	forces.push_back(force);
}

void World::AddTorque(float torque) {
	torques.push_back(torque);
}

void World::Update(float deltaTime) {
	std::vector<PenetrationConstraint> penetrations;

	for (Body*& body : bodies) {
		Vec2 weight = Vec2(0.0f, body->mass * gravity * PIXELS_PER_METER);
		body->AddForce(weight);

		for (Vec2 force : forces) {
			body->AddForce(force);
		}

		for (float torque : torques) {
			body->AddTorque(torque);
		}
	}

	for (auto& body : bodies) {
		body->IntegrateForces(deltaTime);
	}

	for (Body*& body : bodies) {
		body->isColliding = false;
	}

	//Check collisions
	for (int i = 0; i <= bodies.size() - 1; i++) {
		for (int j = i + 1; j < bodies.size(); j++) {
			Body* body1 = bodies[i];
			Body* body2 = bodies[j];
			std::vector<Contact> contacts;

			if (CollisionDetection::IsColliding(body1, body2, contacts)) {
				body1->isColliding = true;
				body2->isColliding = true;

				for (auto contact : contacts) {
					PenetrationConstraint penetration(contact.body1, contact.body2, contact.start, contact.end, contact.normal);
					penetrations.push_back(penetration);
				}
			}
		}
	}

	for (auto& constraint : constraints) {
		constraint->PreSolve(deltaTime);
	}

	for (auto& constraint : penetrations) {
		constraint.PreSolve(deltaTime);
	}

	for (auto& constraint : constraints) {
		constraint->Solve();
	}
	
	for (auto& constraint : penetrations) {
		constraint.Solve();
	}

	for (auto& constraint : constraints) {
		constraint->PostSolve();
	}

	for (auto& constraint : penetrations) {
		constraint.PostSolve();
	}

	for (auto& body : bodies) {
		body->IntegrateVelocities(deltaTime);
	}
}