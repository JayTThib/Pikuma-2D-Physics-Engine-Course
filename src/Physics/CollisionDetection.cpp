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

bool CollisionDetection::IsCollidingCirclePolygon(Body* circBody, Body* polyBody, Contact& contact) {
	const PolygonShape* polyShape = (PolygonShape*)polyBody->shape;
	const CircleShape* circShape = (CircleShape*)circBody->shape;
	const std::vector<Vec2>& polyVertices = polyShape->worldVertices;
	
	bool isOutside = false;
	float distBetweenCircleAndEdge = std::numeric_limits<float>::lowest();
	Vec2 minCurrVertex, minNextVertex;

	//Find the closest edge on the polygon, and see if the circle is inside the polygon
	for (int i = 0; i < polyVertices.size(); i++) {
		int currVertex = i;
		int nextVertex = (i + 1) % polyVertices.size();//Avoids going out of bounds
		Vec2 edge = polyShape->EdgeAt(currVertex);
		Vec2 normal = edge.Normal();

		Vec2 circCenter = circBody->position - polyVertices[currVertex];
		float projection = circCenter.Dot(normal);

		if (projection > 0) {
			distBetweenCircleAndEdge = projection;
			minCurrVertex = polyShape->worldVertices[currVertex];
			minNextVertex = polyShape->worldVertices[nextVertex];
			isOutside = true;
			break;
		}
		else if (projection > distBetweenCircleAndEdge) {
			distBetweenCircleAndEdge = projection;
			minCurrVertex = polyVertices[currVertex];
			minNextVertex = polyVertices[nextVertex];
		}
	}

	/* Calc collision info based on the region that the circle touched on the polygon.
	* Let's say that we have a flat horizontal edge. 
	* If the center of the circle is to the left of the left edge point, then the circle is in region A.
	* If the center of the circle is to the right of the right edge point, then the cicle is in region B.
	* If the center of the circle is between the edge's end points, then the circle is in region C.
	*/
	if (isOutside) {
		//Check if we are in region A
		Vec2 v1 = circBody->position - minCurrVertex;//Vec from the nearest vertex to the circle center
		Vec2 v2 = minNextVertex - minCurrVertex;//The nearest edge (from curr vertex to next vertex)

		if (v1.Dot(v2) < 0) {
			if (v1.Magnitude() > circShape->radius) {
				return false;
			}
			else{//Detected collision in region A
				contact.body1 = polyBody;
				contact.body2 = circBody;
				contact.depth = circShape->radius - v1.Magnitude();
				contact.normal = v1.Normalize();
				contact.start = circBody->position + (contact.normal * -circShape->radius);
				contact.end = contact.start + (contact.normal * contact.depth);
			}
		}
		else {//Check if we are inside region B
			v1 = circBody->position - minNextVertex;//Vec from the next nearest vertex to the circle center
			v2 = minCurrVertex - minNextVertex;//The nearest edge
			if (v1.Dot(v2) < 0) {
				if (v1.Magnitude() > circShape->radius) {
					return false;
				}
				else {//Detected collision in region B
					contact.body1 = polyBody;
					contact.body2 = circBody;
					contact.depth = circShape->radius - v1.Magnitude();
					contact.normal = v1.Normalize();
					contact.start = circBody->position + (contact.normal * -circShape->radius);
					contact.end = contact.start + (contact.normal * contact.depth);
				}
			}
			else {//We are inside region C
				if (distBetweenCircleAndEdge > circShape->radius) {
					return false;
				}
				else {//Detected collision in region C
					contact.body1 = polyBody;
					contact.body2 = circBody;
					contact.depth = circShape->radius - distBetweenCircleAndEdge;
					contact.normal = (minNextVertex - minCurrVertex).Normal();
					contact.start = circBody->position - (contact.normal * circShape->radius);
					contact.end = contact.start + (contact.normal * contact.depth);
				}
			}
		}
	}
	else {
		contact.body1 = polyBody;
		contact.body2 = circBody;
		contact.depth = circShape->radius - distBetweenCircleAndEdge;
		contact.normal = (minNextVertex - minCurrVertex).Normal();
		contact.start = circBody->position - (contact.normal * circShape->radius);
		contact.end = contact.start + (contact.normal * contact.depth);
	}

	return true;
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