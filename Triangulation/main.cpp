#include <fstream>
#include <iostream>
#include <sstream>

#include "Triangulation.h"

void GetPoints(std::istream& input, Triangulation& triangulation)
{
	std::string str;
	std::istringstream iss;
	double x, y;

	while (getline(input, str))
	{
		iss.str(str);
		if (!(iss >> x >> y))
		{
			throw std::runtime_error("Error");
		}

		triangulation.InitPoint(x, y);

		iss.clear();
	}
}

int main()
{
	std::ifstream inFile;
	inFile.open("graph-1.txt");
	if (!inFile.is_open())
	{
		std::cout << "Error opening file" << std::endl;
		return 0;
	}

	Triangulation triangulation;

	try
	{
		GetPoints(inFile, triangulation);
		triangulation.GetTriangulation();
	}
	catch (...)
	{
		std::cout << "Error" << std::endl;
	}
	
	triangulation.Draw();

	return 0;
}
