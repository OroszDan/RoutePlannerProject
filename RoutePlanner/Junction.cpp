#include "Junction.h"
#include "Segment.h"

Junction::Junction(int64_t id, float_t lon, float_t lat):
	m_Id(id), m_Lon(lon), m_Lat(lat), m_FastestRouteInMinutes(FLT_MAX), m_FastestRouteNeighbor(nullptr)
{
	m_Segments = new std::vector<Segment*>();
}

void Junction::AddSegment(Segment* segment)
{
	m_Segments->push_back(segment);
}

Junction::~Junction()
{
	delete m_Segments;
}
