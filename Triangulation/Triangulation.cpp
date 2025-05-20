#include <SFML/Graphics.hpp>
#include <algorithm>
#include <cmath>
#include <memory>
#include <stack>
#include <optional>
#include <array>

#include <iostream>

#include "Point.h"
#include "Triangulation.h"

// Init

void Triangulation::InitPoint(double x, double y)
{
	m_points.push_back(std::make_shared<Point>(x, y));
}

// Get convex hull

Triangulation::PointPtr Triangulation::GetStartPont()
{
	auto startPoint = m_points[0];
	auto startPointPos = startPoint->GetPoint();

	for (const auto& p : m_points)
	{
		const auto point = p->GetPoint();

		if (point.y > startPointPos.y || (point.y == startPointPos.y && point.x < startPointPos.x))
		{
			startPoint = p;
			startPointPos = startPoint->GetPoint();
		}
	}

	return startPoint;
}

bool Triangulation::ComparePointsByAngle(const Point& point1, const Point& point2, const Point& point3)
{
	const auto p1 = point1.GetPoint();
	const auto p2 = point2.GetPoint();
	const auto p3 = point3.GetPoint();

	const double firstAngle = std::atan2(p2.y - p1.y, p2.x - p1.x);
	const double secondAngle = std::atan2(p3.y - p1.y, p3.x - p1.x);

	if (firstAngle > secondAngle)
	{
		return true;
	}
	if (firstAngle == secondAngle)
	{
		return p2.x < p3.x;
	}

	return false;
}

void Triangulation::SortPointsByPolarAngle()
{
	const auto startPoint = GetStartPont();

	std::sort(m_points.begin(), m_points.end(), [&](PointPtr& point2, PointPtr& point3) {
		return ComparePointsByAngle(*startPoint, *point2, *point3);
	});
}

bool Triangulation::CheckLeftTurn(const Point& point1, const Point& point2, const Point& point3)
{
	const auto p1 = point1.GetPoint();
	const auto p2 = point2.GetPoint();
	const auto p3 = point3.GetPoint();

	double turn = (p2.x - p1.x) * (p3.y - p1.y) - (p3.x - p1.x) * (p2.y - p1.y);

	return turn >= 0;
}

std::vector<Triangulation::PointPtr> Triangulation::GetConvexHull()
{
	SortPointsByPolarAngle();

	std::vector<PointPtr> convexHull;

	for (auto& point : m_points)
	{
		while (convexHull.size() >= 2)
		{
			Point p1 = *convexHull[convexHull.size() - 2];
			Point p2 = *convexHull[convexHull.size() - 1];

			if (CheckLeftTurn(p1, p2, *point))
			{
				convexHull.back()->SetState(false);
				convexHull.pop_back();
			}
			else
			{
				break;
			}
		}

		point->SetState(true);
		convexHull.push_back(point);
	}

	return convexHull;
}

// Get triangulation

void Triangulation::RemoveDuplicates()
{
	std::sort(m_points.begin(), m_points.end());

	const auto last = std::unique(m_points.begin(), m_points.end());
	m_points.erase(last, m_points.end());
}

Triangulation::EdgePtr Triangulation::CreateEdge(const PointPtr& p1, const PointPtr& p2)
{
	const auto edge = std::make_shared<Edge>();
	edge->nodes.push_back(p1);
	edge->nodes.push_back(p2);

	m_edges.push_back(edge);

	return edge;
}

Triangulation::EdgePtr Triangulation::GetEdge(const PointPtr& p1, const PointPtr& p2)
{
	const auto it = std::find_if(m_edges.begin(), m_edges.end(), [&](const EdgePtr& edge) {
		return *edge == std::make_pair(p1, p2);
	});

	if (it != m_edges.end())
	{
		return *it;
	}

	return CreateEdge(p1, p2);
}

Triangulation::TrianglePtr Triangulation::CreateTriangle(const PointPtr& p1, const PointPtr& p2, const PointPtr& p3)
{
	const auto edge1 = GetEdge(p1, p2);
	const auto edge2 = GetEdge(p2, p3);
	const auto edge3 = GetEdge(p3, p1);

	const auto triangle = std::make_shared<Triangle>();
	triangle->edges.push_back(edge1);
	triangle->edges.push_back(edge2);
	triangle->edges.push_back(edge3);

	edge1->triangles.push_back(triangle);
	edge2->triangles.push_back(triangle);
	edge3->triangles.push_back(triangle);

	m_triangles.push_back(triangle);

	return triangle;
}

std::optional<Triangulation::EdgePtr> Triangulation::IsPointOnEdge(const PointPtr& point, const TrianglePtr& triangle)
{
	const auto p1 = point->GetPoint();

	for (const auto& edge : triangle->edges)
	{
		const auto p2 = edge->nodes[0]->GetPoint();
		const auto p3 = edge->nodes[1]->GetPoint();
		const double length1 = std::sqrt(std::pow((p1.x - p2.x), 2) + std::pow((p1.y - p2.y), 2));
		const double length2 = std::sqrt(std::pow((p1.x - p3.x), 2) + std::pow((p1.y - p3.y), 2));
		const double totalLength = std::sqrt(std::pow((p2.x - p3.x), 2) + std::pow((p2.y - p3.y), 2));

		if (std::fabs((length1 + length2) - totalLength) < 1e-7)
		{
			return edge;
		}
	}

	return std::nullopt;
}

bool Triangulation::IsPointInsideTriangle(const PointPtr& point, const TrianglePtr& triangle)
{
	const auto vertecies = GetVertecies(triangle);

	const auto p0 = point->GetPoint();
	const auto p1 = vertecies[0]->GetPoint();
	const auto p2 = vertecies[1]->GetPoint();
	const auto p3 = vertecies[2]->GetPoint();

	const double turn1 = (p1.x - p0.x) * (p2.y - p1.y) - (p2.x - p1.x) * (p1.y - p0.y);
	const double turn2 = (p2.x - p0.x) * (p3.y - p2.y) - (p3.x - p2.x) * (p2.y - p0.y);
	const double turn3 = (p3.x - p0.x) * (p1.y - p3.y) - (p1.x - p3.x) * (p3.y - p0.y);

	if ((turn1 <= 0 && turn2 <= 0 && turn3 <= 0) || (turn1 >= 0 && turn2 >= 0 && turn3 >= 0))
	{
		return true;
	}

	return false;
}

void Triangulation::InsertPointIntoEdge(const PointPtr& point, const EdgePtr& edge)
{
	std::cout << "InsertPointIntoEdge" << std::endl;
	point->SetState(true);

	const auto p1 = edge->nodes[0];
	const auto p2 = edge->nodes[1];
	std::vector<PointPtr> oppositeVertecies;

	for (const auto& triangle : edge->triangles)
	{
		const auto vertex = GetOppositeVertex(edge, triangle);
		oppositeVertecies.push_back(vertex);		
	}

	for (const auto& vertex : oppositeVertecies)
	{
		const auto t1 = CreateTriangle(p1, vertex, point);
		const auto t2 = CreateTriangle(p2, vertex, point);

		m_processTriangle.push_back(t1);
		m_processTriangle.push_back(t2);
	}
}

void Triangulation::InsertPointToTriangle(const PointPtr& point, const TrianglePtr& triangle)
{
	point->SetState(true);

	const auto vertecies = GetVertecies(triangle);

	const auto t1 = CreateTriangle(vertecies[0], vertecies[1], point);
	const auto t2 = CreateTriangle(vertecies[1], vertecies[2], point);
	const auto t3 = CreateTriangle(vertecies[2], vertecies[0], point);

	m_processTriangle.push_back(t1);
	m_processTriangle.push_back(t2);
	m_processTriangle.push_back(t3);
}

void Triangulation::RemoveTriangle(const TrianglePtr& triangle)
{
	for (const auto& edge : triangle->edges)
	{
		const auto it = std::find(edge->triangles.begin(), edge->triangles.end(), triangle);

		if (it != edge->triangles.end())
		{
			edge->triangles.erase(it);
		}
	}

	const auto it = std::find(m_triangles.begin(), m_triangles.end(), triangle);

	if (it != m_triangles.end())
	{
		m_triangles.erase(it);
	}
}

Triangulation::PointPtr Triangulation::GetTriangleCenter(const TrianglePtr& triangle)
{
	const auto vertecies = GetVertecies(triangle);

	const auto p1 = vertecies[0]->GetPoint();
	const auto p2 = vertecies[1]->GetPoint();
	const auto p3 = vertecies[2]->GetPoint();

	const double centerX = (p1.x + p2.x + p3.x) / 3;
	const double centerY = (p1.y + p2.y + p3.y) / 3;

	return std::make_shared<Point>(centerX, centerY);
}

std::optional<Triangulation::TrianglePtr> Triangulation::FindTriangle(const PointPtr& point)
{
	bool isFound = false;
	auto current = m_triangles[0];
	const auto p2 = point->GetPoint();

	while (!isFound)
	{
		auto center = GetTriangleCenter(current);
		const auto p1 = center->GetPoint();		

		const double x1 = p1.x;
		const double y1 = p1.y;
		const double x2 = p2.x;
		const double y2 = p2.y;

		bool isFoundNeighbor = false;

		for (const auto& edge : current->edges)
		{
			const auto p3 = edge->nodes[0]->GetPoint();
			const auto p4 = edge->nodes[1]->GetPoint();

			const double x3 = p3.x;
			const double y3 = p3.y;
			const double x4 = p4.x;
			const double y4 = p4.y;

			const double D = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);
			const double intersection1 = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / D;
			const double intersection2 = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / D;

			if (intersection1 >= 0 && intersection1 <= 1 && intersection2 >= 0 && intersection2 <= 1)
			{
				for (const auto triangle : edge->triangles)
				{
					if (triangle != current)
					{
						current = triangle;
						isFoundNeighbor = true;
						break;
					}
				}

				if (isFoundNeighbor)
				{
					break;
				}
				else
				{
					continue;
				}
			}
		}

		if (IsPointInsideTriangle(point, current) || IsPointOnEdge(point, current))
		{
			isFound = true;
		}
	}
	if (isFound)
	{
		return current;
	}

	return std::nullopt;
}

void Triangulation::AddPoint(const PointPtr& p)
{
	std::vector<TrianglePtr> removeTriangles;

	const auto triangle = FindTriangle(p);

	if (triangle)
	{
		if (IsPointInsideTriangle(p, *triangle))
		{
			removeTriangles.push_back(*triangle);
			InsertPointToTriangle(p, *triangle);			
		}
		else if (const auto& edge = IsPointOnEdge(p, *triangle))
		{
			for (const auto t: (*edge)->triangles)
			{
				removeTriangles.push_back(t);
			}
			
			InsertPointIntoEdge(p, *edge);
			const auto it = std::find(m_edges.begin(), m_edges.end(), *edge);
			if (it != m_edges.end())
			{
				m_edges.erase(it);
			}
		}
	}	

	for (const auto& triangle : removeTriangles)
	{
		RemoveTriangle(triangle);
	}
}

std::vector<Triangulation::PointPtr> Triangulation::GetVertecies(const TrianglePtr& triangle)
{
	std::vector<PointPtr> vertecies;

	for (const auto& edge : triangle->edges)
	{
		for (const auto& point : edge->nodes)
		{
			if (std::find(vertecies.begin(), vertecies.end(), point) == vertecies.end())
			{
				vertecies.push_back(point);
			}
		}
	}

	return vertecies;
}

bool Triangulation::CheckDelaunayCondition(const PointPtr& point0, const PointPtr& point1, const PointPtr& point2, const PointPtr& point3)
{
	const auto p0 = point0->GetPoint();
	const auto p1 = point1->GetPoint();
	const auto p2 = point2->GetPoint();
	const auto p3 = point3->GetPoint();

	const double x0 = p0.x;
	const double y0 = p0.y;
	double x1 = p1.x;
	double y1 = p1.y;
	const double x2 = p2.x;
	const double y2 = p2.y;
	double x3 = p3.x;
	double y3 = p3.y;

	double orientation = (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);

	if (orientation < 0)
	{
		std::swap(x1, x3);
		std::swap(y1, y3);
	}

	const double sA = (x0 - x1) * (x0 - x3) + (y0 - y1) * (y0 - y3);
	const double sB = (x2 - x1) * (x2 - x3) + (y2 - y1) * (y2 - y3);

	const double sC = ((x0 - x1) * (y0 - y3) - (x0 - x3) * (y0 - y1)) * ((x2 - x3) * (x2 - x1) + (y2 - y3) * (y2 - y1)) + ((x0 - x1) * (x0 - x3) + (y0 - y1) * (y0 - y3)) * ((x2 - x3) * (y2 - y1) - (x2 - x1) * (y2 - y3));

	if (sA >= -(1e-7) && sB >= -(1e-7))
	{
		return true;
	}
	if (sA < 0 && sB < 0)
	{
		return false;
	}
	if (sC >= -(1e-7))
	{
		return true;
	}

	return false;
}

std::vector<Triangulation::TrianglePtr> Triangulation::Triangle::GetNeighbors()
{
	std::vector<TrianglePtr> neighbors;

	for (const auto& edge : edges)
	{
		for (const auto& triangle : edge->triangles)
		{
			if (*this != *triangle)
			{
				neighbors.push_back(triangle);
			}
		}
	}
	return neighbors;
}

Triangulation::PointPtr Triangulation::GetOppositeVertex(const EdgePtr& edge, const TrianglePtr& triangle)
{
	for (const auto& triangleEdge : triangle->edges)
	{		
		if (triangleEdge == edge)
		{
			continue;
		}
		
		for (const auto& point : triangleEdge->nodes)
		{
			if (point != edge->nodes[0] && point != edge->nodes[1])
			{
				return point;
			}
		}
	}
}

Triangulation::EdgePtr Triangulation::Triangle::GetCommonEdge(const TrianglePtr& neighbor)
{
	for (const auto& edge : edges)
	{
		for (const auto& neighborEdge : neighbor->edges)
		{
			if (edge == neighborEdge)
			{
				return edge;
			}
		}
	}
}

void Triangulation::CheckAndFlip(const TrianglePtr& triangle)
{
	std::stack<TrianglePtr> stack;

	stack.push(triangle);
	
	while (!stack.empty())
	{
		const auto current = stack.top();
		stack.pop();
		const auto neighbors = current->GetNeighbors();

		const auto it = std::find_if(m_processTriangle.begin(), m_processTriangle.end(), [&](const TrianglePtr& t) {
			return *t == *current;
		});

		if (it != m_processTriangle.end())
		{
			m_processTriangle.erase(it);
		}

		for (const auto& neighbor : neighbors)
		{
			auto commonEdge = current->GetCommonEdge(neighbor);

			auto v1 = commonEdge->nodes[0];
			auto v3 = commonEdge->nodes[1];
			auto v2 = GetOppositeVertex(commonEdge, current);
			auto v0 = GetOppositeVertex(commonEdge, neighbor);

			if (!CheckDelaunayCondition(v0, v1, v2, v3))
			{
				const auto it = std::find(m_edges.begin(), m_edges.end(), commonEdge);

				if (it != m_edges.end())
				{
					m_edges.erase(it);
				}

				RemoveTriangle(current);
				RemoveTriangle(neighbor);

				const auto t1 = CreateTriangle(v1, v2, v0);
				const auto t2 = CreateTriangle(v0, v2, v3);

				stack.push(t1);
				stack.push(t2);

				break;
			}
		}
	}
}

void Triangulation::CheckTriangles()
{
	while (!m_processTriangle.empty())
	{
		const auto triangle = m_processTriangle.back();
		m_processTriangle.pop_back();
		CheckAndFlip(triangle);
	}
}

void Triangulation::CreateTrianglesInConvexHull(std::vector<PointPtr>& convexHull)
{
	const PointPtr p0 = m_points[0];

	for (size_t i = 2; i < convexHull.size(); i++)
	{
		const auto p1 = convexHull[i - 1];
		const auto p2 = convexHull[i];

		auto triangle = CreateTriangle(p0, p1, p2);

		CheckAndFlip(triangle);
	}
}

void Triangulation::GetTriangulation()
{
	RemoveDuplicates();

	if (m_points.size() < 3)
	{
		return;
	}

	std::vector<PointPtr> convexHull = GetConvexHull();
	CreateTrianglesInConvexHull(convexHull);

	for (const auto& point : m_points)
	{
		if (point->IsAdd())
		{
			continue;
		}

		AddPoint(point);
		CheckTriangles();
	}
}

// Draw

void Triangulation::GetCirclce(sf::CircleShape& circle, double x, double y)
{
	const auto point = std::make_shared<Point>(x, y);
	for (const auto& triangle : m_triangles)
	{
		if (IsPointInsideTriangle(point, triangle))
		{
			const auto vertecies = GetVertecies(triangle);

			const auto p1 = vertecies[0]->GetPoint();
			const auto p2 = vertecies[1]->GetPoint();
			const auto p3 = vertecies[2]->GetPoint();

			const double D = 2 * (p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y));
			const double centerX = ((std::pow(p1.x, 2) + std::pow(p1.y, 2)) * (p2.y - p3.y) + 
					(std::pow(p2.x, 2) + std::pow(p2.y, 2)) * (p3.y - p1.y) + 
					(std::pow(p3.x, 2) + std::pow(p3.y, 2)) * (p1.y - p2.y)) / D;
			const double centerY = ((std::pow(p1.x, 2) + std::pow(p1.y, 2)) * (p3.x - p2.x) + 
					(std::pow(p2.x, 2) + std::pow(p2.y, 2)) * (p1.x - p3.x) + 
					(std::pow(p3.x, 2) + std::pow(p3.y, 2)) * (p2.x - p1.x)) / D;
			const double radius = std::sqrt(std::pow((centerX - p1.x), 2) + std::pow((centerY - p1.y), 2));

			circle.setRadius(static_cast<float>(radius));
			circle.setOrigin(static_cast<float>(radius), static_cast<float>(radius));
			circle.setPosition(static_cast<float>(centerX), static_cast<float>(centerY));

			circle.setPointCount(100);

			break;
		}
	}
}

void Triangulation::Draw()
{
	sf::ContextSettings settings;
	settings.antialiasingLevel = 4;

	sf::RenderWindow window(sf::VideoMode(800, 600), "Triangulation", sf::Style::Default, settings);

	sf::CircleShape p(2);
	p.setFillColor(sf::Color::Green);
	p.setOrigin(1, 1);

	sf::CircleShape circle;
	circle.setFillColor(sf::Color::Blue);

	while (window.isOpen())
	{
		sf::Event event;
		while (window.pollEvent(event))
		{
			if (event.type == sf::Event::Closed)
			{
				window.close();
			}
			if (event.type == sf::Event::MouseButtonPressed)
			{
				if (event.mouseButton.button == sf::Mouse::Left)
				{
					GetCirclce(circle, event.mouseButton.x, event.mouseButton.y);
				}
				else if (event.mouseButton.button == sf::Mouse::Right)
				{
					const auto point = std::make_shared<Point>(event.mouseButton.x, event.mouseButton.y);

					const size_t trianglesCount = m_triangles.size();

					AddPoint(point);
					CheckTriangles();
				}
			}
		}

		window.clear();

		window.draw(circle);

		for (const auto& point : m_points)
		{
			p.setPosition(point->GetPoint().x, point->GetPoint().y);
			window.draw(p);
		}

		for (const auto& edge : m_edges)
		{
			double x1 = edge->nodes[0]->GetPoint().x;
			double y1 = edge->nodes[0]->GetPoint().y;
			double x2 = edge->nodes[1]->GetPoint().x;
			double y2 = edge->nodes[1]->GetPoint().y;

			sf::Vertex line[] = {
				sf::Vertex(sf::Vector2f(x1, y1)),
				sf::Vertex(sf::Vector2f(x2, y2))
			};

			window.draw(line, 2, sf::Lines);
		}

		window.display();
	}
}
