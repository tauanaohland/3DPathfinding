#pragma once
#include <string>
#include "Descriptors.h"

class ISearcher
{
public:
	void virtual LoadMap(std::string filemap) = 0;
	PathDescriptor virtual Search(const Point& start, const Point& goal, const std::string& expansionPolicy, float TimeLimit, bool debug = false) = 0;
	std::string virtual GetName() = 0;
};