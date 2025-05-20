#pragma once

#include <SFML/Graphics.hpp>
#include <memory>
#include <vector>
#include <optional>

#include "Point.h"

class Triangulation
{
public:
	void InitPoint(double x, double y);
	void GetTriangulation();

	void Draw();

private:
	struct Triangle;
	struct Edge;

	using PointPtr = std::shared_ptr<Point>;
	using EdgePtr = std::shared_ptr<Edge>;
	using TrianglePtr = std::shared_ptr<Triangle>;
	using EdgeVertecies = std::pair<PointPtr, PointPtr>;

	struct Edge
	{
		std::vector<PointPtr> nodes;
		std::vector<TrianglePtr> triangles;
		
		bool operator==(const Edge& other) const
		{
			return (*nodes[0] == *other.nodes[0] && *nodes[1] == *other.nodes[1]) || 
				(*nodes[0] == *other.nodes[1] && *nodes[1] == *other.nodes[0]);
		}

		bool operator==(const EdgeVertecies& points) const
		{
			return (*nodes[0] == *points.first && *nodes[1] == *points.second) || 
				(*nodes[1] == *points.first && *nodes[0] == *points.second);
		}
	};

	struct Triangle
	{
		std::vector<EdgePtr> edges;

		std::vector<TrianglePtr> GetNeighbors();
		EdgePtr GetCommonEdge(const TrianglePtr& neighbor);

		bool operator==(const Triangle& other) const
		{
			return (*edges[0] == *other.edges[0] || *edges[0] == *other.edges[1] || *edges[0] == *other.edges[2])
				&& (*edges[1] == *other.edges[0] || *edges[1] == *other.edges[1] || *edges[1] == *other.edges[2])
				&& (*edges[2] == *other.edges[0] || *edges[2] == *other.edges[1] || *edges[2] == *other.edges[2]);
		}

		bool operator!=(const Triangle& other) const
		{
			return !(*this == other);
		}
	};

	struct VerteciesTriangle
	{
		PointPtr v1;
		PointPtr v2;
		PointPtr v3;
	};

	std::vector<PointPtr> GetConvexHull();
	PointPtr GetStartPont();
	bool ComparePointsByAngle(const Point& point1, const Point& point2, const Point& point3);
	void SortPointsByPolarAngle();
	bool CheckLeftTurn(const Point& point1, const Point& point2, const Point& point3);
	void RemoveDuplicates();
	void CreateTrianglesInConvexHull(std::vector<PointPtr>& convexHull);
	void AddPoint(const PointPtr& point);
	TrianglePtr CreateTriangle(const PointPtr& point1, const PointPtr& point2, const PointPtr& point3);
	EdgePtr GetEdge(const PointPtr& point1, const PointPtr& point2);
	EdgePtr CreateEdge(const PointPtr& point1, const PointPtr& point2);
	std::optional<EdgePtr> IsPointOnEdge(const PointPtr& point, const TrianglePtr& triangle);
	bool IsPointInsideTriangle(const PointPtr& point, const TrianglePtr& triangle);
	void InsertPointToTriangle(const PointPtr& point, const TrianglePtr& triangle);
	void InsertPointIntoEdge(const PointPtr& point, const EdgePtr& edge);
	std::vector<PointPtr> GetVertecies(const TrianglePtr& triangle);
	void RemoveTriangle(const TrianglePtr& triangle);
	void CheckTriangles();
	void CheckAndFlip(const TrianglePtr& triangle);
	bool CheckDelaunayCondition(const PointPtr& point0, const PointPtr& point1, const PointPtr& point2, const PointPtr& point3);
	PointPtr GetOppositeVertex(const EdgePtr& edge, const TrianglePtr& triangle);
	PointPtr GetTriangleCenter(const TrianglePtr& triangle);
	std::optional<TrianglePtr> FindTriangle(const PointPtr& point);

	void GetCirclce(sf::CircleShape& circle, double x, double y);
	
	std::vector<PointPtr> m_points;
	std::vector<EdgePtr> m_edges;
	std::vector<TrianglePtr> m_triangles;
	std::vector<TrianglePtr> m_processTriangle;
};
