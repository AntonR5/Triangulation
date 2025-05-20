#pragma once

class Point
{
public:
	Point(const double x, const double y);

	struct Coordinates
	{
		double x;
		double y;
	};

	Coordinates GetPoint() const;
	bool IsAdd() const;
	void SetState(const bool state);

	bool operator==(const Point& other) const;
	bool operator!=(const Point& other) const;
	bool operator<(const Point& other) const;
	//bool operator>(const Point& other) const;
	
private:
	const double m_x;
	const double m_y;
	bool m_isAdd = false;
};
