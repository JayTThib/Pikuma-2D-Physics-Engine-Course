#include "CollisionDetection.h"

bool CollisionDetection::IsColliding(Body* body1, Body* body2, Contact& contact) {
	if (body1->shape->GetType() == CIRCLE) {
		if (body2->shape->GetType() == CIRCLE) {
			return IsCollidingCircleCircle(body1, body2, contact);
		}
		else {
			return IsCollidingCirclePolygon(body1, body2, contact);
		}
	}
	else if (body2->shape->GetType() == CIRCLE) {
		return IsCollidingCirclePolygon(body2, body1, contact);
	}
	else {
		return IsCollidingPolygonPolygon(body1, body2, contact);
	}
}

bool CollisionDetection::IsCollidingCircleCircle(Body* body1, Body* body2, Contact& contact) {
	CircleShape* circle1 = (CircleShape*)body1->shape;
	CircleShape* circle2 = (CircleShape*)body2->shape;

	const Vec2 dist = body2->position - body1->position;
	const float radiusSum = circle1->radius + circle2->radius;

	bool isColliding = dist.MagnitudeSquared() <= (radiusSum * radiusSum);
	if (!isColliding) {
		return false;
	}
	else {
		contact.body1 = body1;
		contact.body2 = body2;

		contact.normal = dist;
		contact.normal.Normalize();

		contact.start = body2->position - contact.normal * circle2->radius;
		contact.end = body1->position + contact.normal * circle1->radius;
		contact.depth = (contact.end - contact.start).Magnitude();
		return true;
	}
}

bool CollisionDetection::IsCollidingCirclePolygon(Body* circleBody, Body* polyBody, Contact& contact) {

}

bool CollisionDetection::IsCollidingPolygonPolygon(Body* body1, Body* body2, Contact& contact) {
	//Uses the Separating Axis Theorem (doesn't work with concave polygons)
	const PolygonShape* poly1 = (PolygonShape*)body1->shape;
	const PolygonShape* poly2 = (PolygonShape*)body2->shape;
	Vec2 poly1BestAxisOfPenetration, poly2BestAxisOfPenetration, vertexInPoly2WithMinProjection, vertexInPoly1WithMinProjection;
	float separationBetweenPoly1And2 = poly1->FindMinSeparation(poly2, poly1BestAxisOfPenetration, vertexInPoly2WithMinProjection);

	if (separationBetweenPoly1And2 >= 0) {
		return false;
	}

	float separationBetweenPoly2And1 = poly2->FindMinSeparation(poly1, poly2BestAxisOfPenetration, vertexInPoly1WithMinProjection);
	if (separationBetweenPoly2And1 >= 0) {
		return false;
	}
	else{
		contact.body1 = body1;
		contact.body2 = body2;

		if (separationBetweenPoly1And2 > separationBetweenPoly2And1) {
			
			contact.depth = -separationBetweenPoly1And2;
			contact.normal = poly1BestAxisOfPenetration.Normal();
			contact.start = vertexInPoly2WithMinProjection;
			contact.end = vertexInPoly2WithMinProjection + contact.normal * contact.depth;
		}
		else {
			contact.depth = -separationBetweenPoly2And1;
			contact.normal = -poly2BestAxisOfPenetration.Normal();
			contact.start = vertexInPoly1WithMinProjection - contact.normal * contact.depth;
			contact.end = vertexInPoly1WithMinProjection;
		}

		return true;
	}
}