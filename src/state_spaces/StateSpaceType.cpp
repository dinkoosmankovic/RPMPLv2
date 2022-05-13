//
// Created by dinko on 21.5.21..
//

#include "StateSpaceType.h"
#include <iostream>
#include <string>

std::ostream &operator<<(std::ostream &os, const StateSpaceType &v) 
{
	std::string strss;
	switch(v) 
	{
		case StateSpaceType::Abstract:
			strss = "Abstract Type"; break;
		case StateSpaceType::RealVectorSpace:
			strss = "RealVectorSpace Type"; break;
		case StateSpaceType::RealVectorSpaceFCL:
			strss = "RealVectorSpaceFCL Type"; break;
		case StateSpaceType::SO2:
			strss = "SO2 Type"; break;
		case StateSpaceType::SO3:
			strss = "SO3 Type"; break;
	}
	return os << strss;
}