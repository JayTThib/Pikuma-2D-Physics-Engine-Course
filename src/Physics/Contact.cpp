#include "Contact.h"

//Apply positional correction using the projection method
void Contact::ResolvePenetration() {
	if (body1->IsStatic() && body2->IsStatic()) {
		return;
	}

	float d1 = depth / (body1->inverseMass + body2->inverseMass) * body1->inverseMass;
	float d2 = depth / (body1->inverseMass + body2->inverseMass) * body2->inverseMass;

	body1->position -= normal * d1;
	body2->position += normal * d2;
}

void Contact::ResolveCollision() {
	ResolvePenetration();
	
	float elasticity = std::min(body1->elasticity, body2->elasticity);
	float fric = std::min(body1->friction, body2->friction);

	//Calc the relative velocity between the two objects
	Vec2 body1point = end - body1->position;
	Vec2 body2point = start - body2->position;
	Vec2 body1Vel = body1->velocity + Vec2(-body1->angularVelocity * body1point.y, body1->angularVelocity * body1point.x);
	Vec2 body2Vel = body2->velocity + Vec2(-body2->angularVelocity * body2point.y, body2->angularVelocity * body2point.x);
	const Vec2 relativeVelocity = body1Vel - body2Vel;

	//Calc collision impulse along the normal
	float relativeVelocityDotNormal = relativeVelocity.Dot(normal);
	const Vec2 impulseDirectionNormal = normal;
	const float impulseMagnitudeNormal = -(1 + elasticity) * relativeVelocityDotNormal / ((body1->inverseMass + body2->inverseMass) + body1point.Cross(normal) * body1point.Cross(normal) * body1->inverseMomentOfInertia + body2point.Cross(normal) * body2point.Cross(normal) * body2->inverseMomentOfInertia);
	Vec2 impulseNormal = impulseDirectionNormal * impulseMagnitudeNormal;
	
	//Calc collision impulse along the tangent
	Vec2 tangent = normal.Normal();
	float relativeVelocityDotTangent = relativeVelocity.Dot(tangent);
	const Vec2 impulseDirectionTangent = tangent;
	const float impulseMagnitudeTangent = fric * -(1 + elasticity) * relativeVelocityDotTangent / ((body1->inverseMass + body2->inverseMass) + body1point.Cross(tangent) * body1point.Cross(tangent) * body1->inverseMomentOfInertia + body2point.Cross(tangent) * body2point.Cross(tangent) * body2->inverseMomentOfInertia);
	Vec2 impulseTangent = impulseDirectionTangent * impulseMagnitudeTangent;

	Vec2 finalImpulse = impulseNormal + impulseTangent;

	body1->ApplyImpulseAtPoint(finalImpulse, body1point);
	body2->ApplyImpulseAtPoint(-finalImpulse, body2point);
}