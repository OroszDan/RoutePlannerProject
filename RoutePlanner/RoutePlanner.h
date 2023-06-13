#pragma once

#include "Converter.h"

class RoutePlanner
{
public:
	RoutePlanner();
	void Initialize();
	void FindFastestRoute(const float_t startLat, const float_t startLon, const float_t targetLat, const float_t targetLon, std::shared_ptr<std::vector<const Junction*>> resultJunctions);
	Segment* FindContainingSegment(const Junction* junction);
	Junction* GetMin(std::shared_ptr<std::unordered_map<int64_t, Junction*>> LE);
	float_t GetHeuristicTravelTime(const Junction* start, const Junction* target);
	float_t GetTravelTimeInMinutes(float_t distance, uint8_t maxSpeed);
	Junction* GetClosestJunction(const float_t lat, const float_t lon);
	void SaveInnerJunctions(const Junction* referenceJunction, const Segment* segment, std::shared_ptr<std::vector<const Junction*>> junctions);
	void SaveInnerJunctionsAtTarget(const Junction* referenceJunction, const Segment* segment, std::shared_ptr<std::vector<const Junction*>> junctions, const ptrdiff_t startIndex);
	void SaveInnerJunctionsAtStart(const Junction* referenceJunction, const Segment* segment, std::shared_ptr<std::vector<const Junction*>> junctions, const ptrdiff_t index);

private:

	std::shared_ptr<std::unordered_map<int64_t, Junction*>> m_Junctions;
	std::shared_ptr<std::vector<Segment*>> m_Segments;
};

