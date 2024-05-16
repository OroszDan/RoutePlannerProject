#include "Segment.h"

Segment::Segment(const int64_t id, const float_t length, Junction* from, Junction* to, uint8_t maxSpeed) :
	m_Id(id), m_LengthInMetres(length), m_From(from), m_To(to), m_MaxSpeedInKmh(maxSpeed)
{
	m_InnerNodes = std::make_shared<std::vector<Junction*>>();
}

Segment::~Segment()
{
	m_From = nullptr;
	m_To = nullptr;
}

Junction* Segment::GetEndJunction(const Junction * startJunction)
{
	if (startJunction == m_From)
	{
		return m_To;
	}
	else if (startJunction == m_To)
	{
		return m_From;
	}
	else
	{
		throw new std::exception("No endjunction can be found!");
	}
}
