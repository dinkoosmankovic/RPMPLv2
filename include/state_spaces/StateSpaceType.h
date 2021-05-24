//
// Created by dinko on 21.5.21..
//

#ifndef RPMPL_STATESPACETYPE_H
#define RPMPL_STATESPACETYPE_H

#include <ostream>
#include <string>

enum class StateSpaceType
{
	Abstract, // only here for placeholding
	RealVectorSpace,
	SO2,
	SO3
};

std::ostream& operator<<(std::ostream &os, const StateSpaceType& v);


#endif //RPMPL_STATESPACETYPE_H
