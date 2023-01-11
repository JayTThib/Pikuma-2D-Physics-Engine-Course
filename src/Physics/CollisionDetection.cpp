#include "CollisionDetection.h"

bool CollisionDetection::IsColliding(Body* body1, Body* body2, std::vector<Contact>& contacts) {
	if (body1->shape->GetType() == CIRCLE) {
		if (body2->shape->GetType() == CIRCLE) {
			return IsCollidingCircleCircle(body1, body2, contacts);
		}
		else {
			return IsCollidingCirclePolygon(body1, body2, contacts);
		}
	}
	else if (body2->shape->GetType() == CIRCLE) {
		return IsCollidingCirclePolygon(body2, body1, contacts);
	}
	else {
		return IsCollidingPolygonPolygon(body1, body2, contacts);
	}
}

bool CollisionDetection::IsCollidingCircleCircle(Body* body1, Body* body2, std::vector<Contact>& contacts) {
	CircleShape* circle1 = (CircleShape*)body1->shape;
	CircleShape* circle2 = (CircleShape*)body2->shape;

	const Vec2 dist = body2->position - body1->position;
	const float radiusSum = circle1->radius + circle2->radius;

	bool isColliding = dist.MagnitudeSquared() <= (radiusSum * radiusSum);
	if (!isColliding) {
		return false;
	}
	else {
		Contact contact;

		contact.body1 = body1;
		contact.body2 = body2;

		contact.normal = dist;
		contact.normal.Normalize();

		contact.start = body2->position - contact.normal * circle2->radius;
		contact.end = body1->position + contact.normal * circle1->radius;
		contact.depth = (contact.end - contact.start).Magnitude();

		contacts.push_back(contact);

		return true;
	}
}

bool CollisionDetection::IsCollidingCirclePolygon(Body* circBody, Body* polyBody, std::vector<Contact>& contacts) {
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

	Contact contact;

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

	contacts.push_back(contact);

	return true;
}

bool CollisionDetection::IsCollidingPolygonPolygon(Body* body1, Body* body2, std::vector<Contact>& contacts) {
	//Reference edge is the original axis where the contact of best penetration was found.
	//Incident edge is a face of the "other" rigidbody that is opposing the reference edge.

	PolygonShape* poly1 = (PolygonShape*)body1->shape;
	PolygonShape* poly2 = (PolygonShape*)body2->shape;

	int indexReferenceEdge1, indexReferenceEdge2;
	Vec2 supportPoint1, supportPoint2;
	float separationBetweenPoly1And2 = poly1->FindMinSeparation(poly2, indexReferenceEdge1, supportPoint1);

	if (separationBetweenPoly1And2 >= 0) {
		return false;
	}

	float separationBetweenPoly2And1 = poly2->FindMinSeparation(poly1, indexReferenceEdge2, supportPoint2);
	if (separationBetweenPoly2And1 >= 0) {
		return false;
	}
	else{
		PolygonShape* referenceShape;
		PolygonShape* incidentShape;
		int indexReferenceEdge;

		if (separationBetweenPoly1And2 > separationBetweenPoly2And1) {
			referenceShape = poly1;
			incidentShape = poly2;
			indexReferenceEdge = indexReferenceEdge1;
		}
		else {
			referenceShape = poly2;
			incidentShape = poly1;
			indexReferenceEdge = indexReferenceEdge2;
		}

		Vec2 referenceEdge = referenceShape->EdgeAt(indexReferenceEdge);

		//The index of the point that's penetrating into the other object.
		int incidentIndex = incidentShape->FindIncidentEdge(referenceEdge.Normal());
		//The index of the point that isn't penetrating the other object, but is on the same edge as the point that is.
		int incidentNextIndex = (incidentIndex + 1) % incidentShape->worldVertices.size();

		Vec2 penetratingPoint = incidentShape->worldVertices[incidentIndex];
		Vec2 nonpenetratingPoint = incidentShape->worldVertices[incidentNextIndex];

		std::vector<Vec2> contactPoints = { penetratingPoint, nonpenetratingPoint };
		std::vector<Vec2> clippedPoints = contactPoints;
				
		for (int i = 0; i < referenceShape->worldVertices.size(); i++) {
			if (i == indexReferenceEdge) {
				continue;
			}

			Vec2 clip1 = referenceShape->worldVertices[i];
			Vec2 clip2 = referenceShape->worldVertices[(i + 1) % referenceShape->worldVertices.size()];
			int numClipped = referenceShape->ClipSegmentToLine(contactPoints, clippedPoints, clip1, clip2);

			if (numClipped > 2) {
				break;
			}

			contactPoints = clippedPoints;
		}

		Vec2 vref = referenceShape->worldVertices[indexReferenceEdge];

		//Loop all clipped points, but only consider those where separation is negative (objects are penetrating each other) 
		for (Vec2& clippedVertex : clippedPoints) {
			float separation = (clippedVertex - vref).Dot(referenceEdge.Normal());

			if (separation <= 0) {
				Contact contact;
				contact.body1 = body1;
				contact.body2 = body2;
				contact.normal = referenceEdge.Normal();
				contact.start = clippedVertex;
				contact.end = clippedVertex + contact.normal * -separation;

				if (separationBetweenPoly2And1 >= separationBetweenPoly1And2) {
					std::swap(contact.start, contact.end);
					contact.normal *= -1.0f;
				}

				contacts.push_back(contact);
			}
		}

		return true;
	}
}