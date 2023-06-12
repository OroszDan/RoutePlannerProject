#include "Segment.h"

Segment::Segment(const int64_t id, const float_t length, Junction* from, Junction* to, uint8_t maxSpeed) :
	m_Id(id), m_Length(length), m_From(from), m_To(to), m_MaxSpeed(maxSpeed)
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
	return startJunction == m_From ? m_To : m_From;
}
