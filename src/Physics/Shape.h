#ifndef SHAPE_H
#define SHAPE_H

#include "Vec2.h"
#include <vector>

enum ShapeType {
	CIRCLE,
	POLYGON,
	BOX
};

struct Shape {
	virtual ~Shape() = default;
	virtual ShapeType GetType() const = 0;
	virtual Shape* Clone() const = 0;
	virtual void UpdateVertices(float angle, const Vec2& position) = 0;//Rotate and translate the polygon vertices from local space to world space.
	virtual float GetMomentOfInertia() const = 0;
};

struct CircleShape : public Shape {
	float radius;

	CircleShape(const float radius);
	virtual ~CircleShape();
	ShapeType GetType() const override;
	Shape* Clone() const override;
	void UpdateVertices(float angle, const Vec2& position) override;
	float GetMomentOfInertia() const override;
};

struct PolygonShape : public Shape {
	std::vector<Vec2> localVertices;
	std::vector<Vec2> worldVertices;

	PolygonShape() = default;
	PolygonShape(const std::vector<Vec2> vertices);
	virtual ~PolygonShape();
	ShapeType GetType() const override;
	Shape* Clone() const override;
	Vec2 EdgeAt(int index) const;
	float FindMinSeparation(const PolygonShape* other, int& indexReferenceEdge, Vec2& supportPoint) const;
	void UpdateVertices(float angle, const Vec2& position);
	float PolygonArea() const;
	Vec2 PolygonCentroid() const;
	float GetMomentOfInertia() const override;
	int FindIncidentEdge(const Vec2& referenceEdgeNormal) const;
	int ClipSegmentToLine(const std::vector<Vec2>& contactsIn, std::vector<Vec2>& contactsOut, const Vec2& clip1, const Vec2& clip2) const;
};

struct BoxShape : public PolygonShape {
	float width;
	float height;

	BoxShape(float width, float height);
	virtual ~BoxShape();
	ShapeType GetType() const override;
	Shape* Clone() const override;
	float GetMomentOfInertia() const override;
};

#endif