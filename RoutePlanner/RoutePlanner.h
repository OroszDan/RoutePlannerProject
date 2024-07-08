#pragma once

#include "Converter.h"
#include "Car.h"

class RoutePlanner
{
public:
	RoutePlanner();
	void Initialize(std::string preprocessedFileName);
	void FindFastestRoute(const float_t startLat, const float_t startLon, const float_t targetLat, const float_t targetLon, const Car& car, std::shared_ptr<std::vector<const Junction*>> resultJunctions);

	
private:
	float_t CalculateConsumptionInPercent(const Junction* start, const Junction* end, const Segment* route, const Car& car) const;
	float_t RechargeBattery(float_t& batteryChargeInPercent, const Car& car, const uint16_t maxChargeSpeedInKW);
	Segment* FindContainingSegment(const Junction* junction);
	ChargingJunction* SelectCharger(std::shared_ptr<std::vector<ChargingJunction*>> foundChargers, const Car& car, float_t currentLat, float_t currentLon);
	Junction* PopMin(std::shared_ptr<std::unordered_map<int64_t, Junction*>> LE);
	float_t GetHeuristicTravelTime(const Junction* start, const Junction* target);
	float_t GetTravelTimeInMinutes(float_t distance, uint8_t maxSpeed);
	Junction* GetClosestJunction(const float_t lat, const float_t lon);
	Junction* PopFirst(std::shared_ptr<std::unordered_map<int64_t, Junction*>> LE);
	void FindChargersWithinProximity(std::shared_ptr<std::vector<ChargingJunction*>> foundChargers, Junction* startJunction, const Junction* targetJunction, const Car& car);
	void SaveInnerJunctions(const Junction* referenceJunction, const Segment* segment, std::shared_ptr<std::vector<const Junction*>> junctions);
	void SaveInnerJunctionsAtTarget(const Junction* referenceJunction, const Segment* segment, std::shared_ptr<std::vector<const Junction*>> junctions, const ptrdiff_t startIndex);
	void SaveInnerJunctionsAtStart(const Junction* referenceJunction, const Segment* segment, std::shared_ptr<std::vector<const Junction*>> junctions, const ptrdiff_t index);

private:

	std::shared_ptr<std::unordered_map<int64_t, Junction*>> m_Junctions;
	std::shared_ptr<std::vector<Segment*>> m_Segments;
};

