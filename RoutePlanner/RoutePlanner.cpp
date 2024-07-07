#include "RoutePlanner.h"
#include "Util.h"

RoutePlanner::RoutePlanner()
{
    m_Junctions = std::make_shared<std::unordered_map<int64_t, Junction*>>();
    m_Segments = std::make_shared<std::vector<Segment*>>();
}

void RoutePlanner::Initialize(std::string preprocessedFileName)
{
    std::unique_ptr<Converter> converter = std::make_unique<Converter>();

    converter->ReadPreprocessedDataFromJson(preprocessedFileName, m_Junctions, m_Segments);
}

void RoutePlanner::FindFastestRoute(const float_t startLat, const float_t startLon, const float_t targetLat, const float_t targetLon, 
                                    const Car& car, std::shared_ptr<std::vector<const Junction*>> resultJunctions)
{
    //A* algorithm 

    std::shared_ptr<std::unordered_map<int64_t, int64_t>> S = std::make_shared<std::unordered_map<int64_t, int64_t>>();
    std::shared_ptr<std::unordered_map<int64_t, Junction*>> LE = std::make_shared<std::unordered_map<int64_t, Junction*>>();
    bool routeFound = false;
    bool chargerNotFound = false;
    float_t batteryChargingTimeInMinutes = 0;
    float_t batteryChargeInPercent = car.m_ChargeInPercent;

    Junction* start = GetClosestJunction(startLat, startLon);
    Junction* innerStart = nullptr;
    Junction* target = GetClosestJunction(targetLat, targetLon);    
    Junction* innerTarget = nullptr;

    Segment* startSegment = nullptr;
    Segment* targetSegment = nullptr;

    if (start->m_StartingSegments->size() == 0)
    {
        //inner node
        innerStart = start;
        startSegment = FindContainingSegment(innerStart);
        start = startSegment->m_From;
    }

    if (target->m_StartingSegments->size() == 0)
    {
        //inner node
        targetSegment = FindContainingSegment(target);
    }

    start->m_FastestRouteInMinutes = 0;
    start->m_FastestRouteInMinutesHeuristic = 0;
    start->m_BatteryChargeInPercent = car.m_ChargeInPercent;

    Junction* u = nullptr;
    Junction* endJunction = nullptr;

    auto foundChargers = std::make_shared<std::vector<ChargingJunction*>>();
 
    LE->insert(std::make_pair(start->m_Id, start));
    S->insert(std::make_pair(start->m_Id, start->m_Id));

    while (S->size() < m_Junctions->size() && !routeFound && !chargerNotFound)
    {
        bool mustCharge = false;
        ChargingJunction* selectedCharger = nullptr;
        const Junction* mustChargeJunction = nullptr;
        while (S->size() < m_Junctions->size() && !routeFound && !mustCharge)
        {
            //getmin(and delete)
            u = PopMin(LE);

            if (ChargingJunction* chargerPtr = dynamic_cast<ChargingJunction*>(u))  //type check
            {
                if (u != start)
                {
                    foundChargers->emplace_back(chargerPtr);
                }
            }

            if (u->m_BatteryChargeInPercent < car.m_MinChargeInPercent)
            {
                mustCharge = true;
                mustChargeJunction = u;
            }
            else
            {
                for (auto it = u->m_StartingSegments->begin(); it != u->m_StartingSegments->end() && routeFound == false && !mustCharge; ++it)
                {
                    endJunction = (*it)->GetEndJunction(u);
                    //insert neighbor 

                    if (u->m_FastestRouteInMinutes + GetTravelTimeInMinutes((*it)->m_LengthInMetres, (*it)->m_MaxSpeedInKmh) < endJunction->m_FastestRouteInMinutes)
                    {
                        endJunction->m_FastestRouteInMinutes =
                            u->m_FastestRouteInMinutes + GetTravelTimeInMinutes((*it)->m_LengthInMetres, (*it)->m_MaxSpeedInKmh);
                        endJunction->m_FastestRouteNeighbor = *it;

                        //need to calculate consumption
                        endJunction->m_BatteryChargeInPercent = u->m_BatteryChargeInPercent - CalculateConsumptionInPercent(u, endJunction, *it, car);

                    }

                    if (!S->contains(endJunction->m_Id) && !LE->contains(endJunction->m_Id))
                    {
                        endJunction->m_FastestRouteInMinutesHeuristic = GetHeuristicTravelTime(endJunction, target);
                        LE->insert(std::make_pair(endJunction->m_Id, endJunction));
                        S->insert(std::make_pair(endJunction->m_Id, endJunction->m_Id));
                   
                    }

                    if (endJunction->m_Id == target->m_Id || (*it) == targetSegment)
                    {
                        routeFound = true;
                    }

                }
            }
        }

        if (!routeFound)
        {
            //find suitable charger
            if (foundChargers->size() > 0)
            {
                selectedCharger = SelectCharger(foundChargers, car, mustChargeJunction->m_Lat, mustChargeJunction->m_Lon);

                auto maxIt = std::max_element(selectedCharger->m_ChargingInfo.cbegin(), selectedCharger->m_ChargingInfo.cend(), 
                            [](const ChargingData data1, const ChargingData data2) {
                                    return data1.m_Output > data2.m_Output;
                            });
                batteryChargeInPercent = selectedCharger->m_BatteryChargeInPercent;
                
                //charge the battery here
                batteryChargingTimeInMinutes += RechargeBattery(batteryChargeInPercent, car, maxIt->m_Output);

                const Junction* currentJunction = selectedCharger;  //charger junction might be saved 2 times if route is found

                Segment* currentSegment = nullptr;
                std::shared_ptr<std::vector<const Junction*>> localResultJunctions = std::make_shared<std::vector<const Junction*>>();

                while (currentJunction != start)
                {
                    if (startSegment && currentJunction == startSegment->m_To)
                    {
                        break;
                    }
                    //resultjunctions sequence will be wrong probably
                    localResultJunctions->push_back(currentJunction);
                    currentSegment = currentJunction->m_FastestRouteNeighbor;

                    SaveInnerJunctions(currentJunction, currentSegment, localResultJunctions);

                    //resultSegments->push_back(currentSegment);
                    currentJunction = currentSegment->GetEndJunction(currentJunction);
                }

                if (startSegment)
                {
                    localResultJunctions->push_back(currentJunction);

                    auto startIt = std::find(startSegment->m_InnerNodes->begin(),
                        startSegment->m_InnerNodes->end(), innerStart);
                    ptrdiff_t index = startIt - startSegment->m_InnerNodes->begin();

                    SaveInnerJunctionsAtStart(localResultJunctions->back(), startSegment, localResultJunctions, index);
                }
                else
                {
                    localResultJunctions->push_back(currentJunction);
                }

                resultJunctions->insert(resultJunctions->begin(), localResultJunctions->begin(), localResultJunctions->end());

                //adjust new starting point which is the selected charger

                startSegment = nullptr;
                start = selectedCharger;
                start->m_FastestRouteInMinutes = 0;
                start->m_FastestRouteInMinutesHeuristic = 0;
                start->m_BatteryChargeInPercent = batteryChargeInPercent;

                S = std::make_shared<std::unordered_map<int64_t, int64_t>>();
                LE = std::make_shared<std::unordered_map<int64_t, Junction*>>();
                foundChargers = std::make_shared<std::vector<ChargingJunction*>>();

                LE->insert(std::make_pair(start->m_Id, start));
                S->insert(std::make_pair(start->m_Id, start->m_Id));
            }
            else
            {
                chargerNotFound = true;
            }
        }
    }

    if (S->size() < m_Junctions->size() && !chargerNotFound) //route found
    {
        //collect result
        //std::shared_ptr<std::vector<const Segment*>> resultSegments = std::make_shared<std::vector<const Segment*>>();
        std::shared_ptr<std::vector<const Junction*>> localResultJunctions = std::make_shared<std::vector<const Junction*>>();

        const Junction* currentJunction;

        if (targetSegment)
        {
            auto targetIt = std::find(targetSegment->m_InnerNodes->begin(), 
                                                        targetSegment->m_InnerNodes->end(), target);
            ptrdiff_t startIndex = targetIt - targetSegment->m_InnerNodes->begin();

            SaveInnerJunctionsAtTarget(u, targetSegment, localResultJunctions, startIndex);

            currentJunction = u;
        }
        else
        {
            currentJunction = target;
        }

        Segment* currentSegment = nullptr;

        while (currentJunction != start)
        {
            if (startSegment && currentJunction == startSegment->m_To)
            {
                break;
            }

            localResultJunctions->push_back(currentJunction);
            currentSegment = currentJunction->m_FastestRouteNeighbor;

            SaveInnerJunctions(currentJunction, currentSegment, localResultJunctions);

            //resultSegments->push_back(currentSegment);     
            currentJunction = currentSegment->GetEndJunction(currentJunction);
        }

        if (startSegment)
        {
            localResultJunctions->push_back(currentJunction);

            auto startIt = std::find(startSegment->m_InnerNodes->begin(),
                startSegment->m_InnerNodes->end(), innerStart);
            ptrdiff_t index = startIt - startSegment->m_InnerNodes->begin();

            SaveInnerJunctionsAtStart(localResultJunctions->back(), startSegment, localResultJunctions, index);
        }
        else
        {
            localResultJunctions->push_back(currentJunction);
        }

        resultJunctions->insert(resultJunctions->begin(), localResultJunctions->begin(), localResultJunctions->end());
    }
}

float_t RoutePlanner::CalculateConsumptionInPercent(const Junction* start, const Junction* end, const Segment* route, const Car& car) const
{
    int32_t balanceConst1 = 13720000;
    //int32_t balanceConst2 = 13720000;

    //peugeot e 208 range
    // NEDC: 410km
    // Real: 290km  //kb 70kmh constant speed

    //consumption
    // NEDC: 11 Kwh / 100 km --> 24%  1 meter --> 0.00024%
    // Real: 16 KWh / 100 km --> 34%  1 meter --> 0.00034%

    //calculate consumption on 1 meter

     float_t result = route->m_LengthInMetres * (car.m_NEDCConsumptionOnOneMetreInPercent + 
           (car.m_DragCoefficient * route->m_MaxSpeedInKmh * route->m_MaxSpeedInKmh) / balanceConst1 /*+ (car.m_WeightInKg * route->m_Slope) / balanceConst2*/);

     return result;

}

float_t RoutePlanner::RechargeBattery(float_t& batteryChargeInPercent, const Car& car, const uint16_t maxChargeSpeedInKW)
{
    const float_t onePercentBatteryCapacityInKWh = car.m_BatteryCapacityInKWh / 100.0f;
    float_t chargingTimeInMinutes = 0;
  
    for (auto it = car.m_ChargeSpeedDataInKW->cbegin() + car.m_MinChargeInPercent; it != car.m_ChargeSpeedDataInKW->cbegin() + car.m_MaxChargeInPercent; ++it)
    {
        if (*it < maxChargeSpeedInKW)
           chargingTimeInMinutes += onePercentBatteryCapacityInKWh / *it * 60;
        else
           chargingTimeInMinutes += onePercentBatteryCapacityInKWh / maxChargeSpeedInKW * 60;
    }

    batteryChargeInPercent = car.m_MaxChargeInPercent;

    return chargingTimeInMinutes;
}

Segment* RoutePlanner::FindContainingSegment(const Junction* junction)
{
    bool dataFound = false;

    Segment* containingSegment = nullptr;

    for (auto it = m_Segments->begin(); it != m_Segments->end() && !dataFound; ++it)
    {
        for (auto nodeIt = (*it)->m_InnerNodes->begin(); nodeIt != (*it)->m_InnerNodes->end() && !dataFound; ++nodeIt)
        {
            if (junction == *nodeIt)
            {
                dataFound = true;
            }
        }

        if (dataFound)
        {
            containingSegment = *it;
        }
    }

    return containingSegment;
}

ChargingJunction* RoutePlanner::SelectCharger(std::shared_ptr<std::vector<ChargingJunction*>> foundChargers, const Car& car, float_t currentLat, float_t currentLon)
{
    auto maxValue = *std::max_element(car.m_ChargeSpeedDataInKW->cbegin(), car.m_ChargeSpeedDataInKW->cend());
    
    std::erase_if(*foundChargers, [car](const ChargingJunction* current)
    {
        for (auto it = current->m_ChargingInfo.cbegin(); it != current->m_ChargingInfo.cend(); it++)
        {
            if (it->m_Type == car.m_ChargerStandard)
            {
                return false;
            }

        }

        return true;
    });

    if (foundChargers->size() > 0)
    {

        for (auto it = foundChargers->begin(); it != foundChargers->end(); ++it)
        {
            std::erase_if((*it)->m_ChargingInfo, [car](const ChargingData current)
            {
                if (current.m_Type != car.m_ChargerStandard)
                {
                    return true;
                }
                else
                {
                    return false;
                }
            });
        }

        //find closest one
        auto minIt =
        std::min_element(foundChargers->begin(), foundChargers->end(), [currentLat, currentLon](const ChargingJunction* elem1, const ChargingJunction* elem2)
        {
            return Util::CalculateDistanceBetweenTwoLatLonsInMetres(currentLat, elem1->m_Lat, currentLon, elem1->m_Lon)
                < Util::CalculateDistanceBetweenTwoLatLonsInMetres(currentLat, elem2->m_Lat, currentLon, elem2->m_Lon);
        });

        return *minIt;
        
    }
    else 
    {
        return nullptr;
    }
}

Junction* RoutePlanner::PopMin(std::shared_ptr<std::unordered_map<int64_t, Junction*>> LE)
{
    //min queue may be faster
    auto minIt = 
    std::min_element(LE->begin(), LE->end(), [](const std::pair<int64_t, Junction*> elem1, const std::pair<int64_t, Junction*> elem2)
    {
        return elem1.second->m_FastestRouteInMinutes + elem1.second->m_FastestRouteInMinutesHeuristic 
             < elem2.second->m_FastestRouteInMinutes + elem2.second->m_FastestRouteInMinutesHeuristic; 
    });

    Junction* minElem = (*minIt).second;

    LE->erase(minIt);

    return minElem;
}

float_t RoutePlanner::GetHeuristicTravelTime(const Junction* start, const Junction* target)
{
    float_t distance = Util::CalculateDistanceBetweenTwoLatLonsInMetres(start->m_Lat, target->m_Lat,
                                                                        start->m_Lon, target->m_Lon);
    float_t travelTime = GetTravelTimeInMinutes(distance, 50);

    return travelTime;
}

float_t RoutePlanner::GetTravelTimeInMinutes(float_t distance, uint8_t maxSpeed)
{
    return distance / 1000 / maxSpeed * 60;
}

Junction* RoutePlanner::GetClosestJunction(const float_t lat, const float_t lon)
{
    auto minIt =
    std::min_element(m_Junctions->begin(), m_Junctions->end(), [lat, lon](const std::pair<int64_t, Junction*> elem1, const std::pair<int64_t, Junction*> elem2)
    {
        return Util::CalculateDistanceBetweenTwoLatLonsInMetres(lat, elem1.second->m_Lat, lon, elem1.second->m_Lon)
             < Util::CalculateDistanceBetweenTwoLatLonsInMetres(lat, elem2.second->m_Lat, lon, elem2.second->m_Lon);
    });

    return (*minIt).second;
}

void RoutePlanner::SaveInnerJunctions(const Junction* referenceJunction, const Segment* segment, std::shared_ptr<std::vector<const Junction*>> junctions)
{
    if (referenceJunction == segment->m_From)
    {
        for (auto it = segment->m_InnerNodes->begin(); it != segment->m_InnerNodes->end(); ++it)
        {
            junctions->push_back(*it);
        }
        //simple
    }
    else
    {
        for (auto it = segment->m_InnerNodes->rbegin(); it != segment->m_InnerNodes->rend(); ++it)
        {
            junctions->push_back(*it);
        }
        //reverse
    }
}

void RoutePlanner::SaveInnerJunctionsAtTarget(const Junction* referenceJunction, const Segment* segment, std::shared_ptr<std::vector<const Junction*>> junctions, const ptrdiff_t startIndex)
{
    if (referenceJunction == segment->m_From)
    {
        for (int i = startIndex; i >= 0; --i)
        {
            junctions->push_back(segment->m_InnerNodes->at(i));
        }
    }
    else
    {
        for (size_t i = startIndex; i < segment->m_InnerNodes->size(); i++)
        {
            junctions->push_back(segment->m_InnerNodes->at(i));
        }
    }
}

void RoutePlanner::SaveInnerJunctionsAtStart(const Junction* referenceJunction, const Segment* segment, std::shared_ptr<std::vector<const Junction*>> junctions, const ptrdiff_t index)
{
    if (referenceJunction == segment->m_From)
    {
        for (int i = 0; i < index; i++)
        {
            junctions->push_back(segment->m_InnerNodes->at(i));
        }
    }
    else
    {
        for (int i = index; i < segment->m_InnerNodes->size(); ++i)
        {
            junctions->push_back(segment->m_InnerNodes->at(i));
        }
    }
}
