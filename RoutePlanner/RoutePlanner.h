#pragma once

#include "Converter.h"

class RoutePlanner
{

public:
	RoutePlanner();
	void Initialize();
	void FindFastestRoute(const float_t startLat, const float_t startLon, const float_t targetLat, const float_t targetLon, std::shared_ptr<std::vector<const Junction*>> resultJunctions);
	Segment* FindContainingSegment(const Junction* junction);
	//bool SameJunction(const float_t lat, const float_t lon, const Junction* currentJunction);
	Junction* GetMin(std::shared_ptr<std::unordered_map<int64_t, int64_t>> S, std::shared_ptr<std::unordered_map<int64_t, Junction*>> LE);
	float_t GetHeuristicTravelTime(const Junction* start, const Junction* target);
	float_t GetTravelTimeInMinutes(float_t distance, uint8_t maxSpeed);
	void SaveInnerJunctions(const Junction* referenceJunction, const Segment* segment, std::shared_ptr<std::vector<const Junction*>> junctions);
	void GetStartAndTargetJunction(const float_t startLat, const float_t startLon, const float_t targetLat, const float_t targetLon, Junction*& startJunction, Junction*& targetJunction);
	void SaveInnerJunctionsAtRoadEnds(const Junction* referenceJunction, const Segment* segment, std::shared_ptr<std::vector<const Junction*>> junctions, const ptrdiff_t startIndex);

private:

	std::shared_ptr<std::unordered_map<int64_t, Junction*>> m_Junctions;
	std::shared_ptr<std::vector<Segment*>> m_Segments;
};

