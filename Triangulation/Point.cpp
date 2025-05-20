#include "Point.h"

Point::Point(const double x, const double y)
	: m_x(x)
	, m_y(y)
{
}

Point::Coordinates Point::GetPoint() const
{
	return { m_x, m_y };
}

bool Point::IsAdd() const
{
	return m_isAdd;
}

void Point::SetState(const bool state)
{
	m_isAdd = state;
}

bool Point::operator==(const Point& other) const
{
	return (m_x == other.m_x && m_y == other.m_y);
}

bool Point::operator!=(const Point& other) const
{
	return !(*this == other);
}

bool Point::operator<(const Point& other) const
{
	if (m_x != other.m_x)
	{
		return m_x < other.m_x;
	}

	return m_y < other.m_y;
}

// bool Point::operator>(const Point& other) const
//{
//	return *this != other && !(*this < other);
// }
