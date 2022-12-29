#ifndef FORCE_H
#define FORCE_H

#include "./Vec2.h"
#include "./Particle.h"

struct Force {
	static Vec2 GenerateDragForce(const Particle& particle, float magnitude);
	static Vec2 GenerateFrictionForce(const Particle& particle, float magnitude);
	static Vec2 GenerateGravitationalForce(const Particle& particleA, const Particle& particleB, float grav);
};

#endif
