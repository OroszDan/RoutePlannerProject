#pragma once

#include <cmath>
#include <cstdint>
#include <vector>
#include <memory>

class Segment;  //forward declaration

class Junction
{
public:

	Junction(int64_t id, float_t lon, float_t lat, float_t elevation);

	void AddSegment(Segment* segment);

	virtual ~Junction();

	int64_t m_Id;

	float_t m_Lon;

	float_t m_Lat;

	float_t m_Elevation;

	Segment* m_FastestRouteNeighbor;

	float_t m_FastestRouteInMinutes;

	float_t m_FastestRouteInMinutesHeuristic;

	float_t m_BatteryChargeInPercent;

	std::shared_ptr<std::vector<Segment*>> m_StartingSegments;
};

struct Node
{
	int64_t m_Id;

	float_t m_Lon;

	float_t m_Lat;

	float_t m_Elevation;
};

