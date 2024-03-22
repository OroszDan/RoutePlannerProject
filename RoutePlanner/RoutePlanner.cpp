#include "RoutePlanner.h"
#include "Util.h"

RoutePlanner::RoutePlanner()
{
    m_Junctions = std::make_shared<std::unordered_map<int64_t, Junction*>>();
    m_Segments = std::make_shared<std::vector<Segment*>>();
}

void RoutePlanner::Initialize()
{
    std::unique_ptr<Converter> converter = std::make_unique<Converter>();

    converter->ReadPreprocessedDataFromJson("highwaydata.json", m_Junctions, m_Segments);
}

void RoutePlanner::FindFastestRoute(const float_t startLat, const float_t startLon, const float_t targetLat, const float_t targetLon, 
                                    const uint8_t minCharge, float_t batteryCharge, const int16_t batteryCapacity, 
                                    std::shared_ptr<std::vector<const int16_t>> chargeSpeedData,
                                    std::shared_ptr<std::vector<const Junction*>> resultJunctions)
{
    //A* algorithm 

    std::shared_ptr<std::unordered_map<int64_t, int64_t>> S = std::make_shared<std::unordered_map<int64_t, int64_t>>();
    std::shared_ptr<std::unordered_map<int64_t, Junction*>> LE = std::make_shared<std::unordered_map<int64_t, Junction*>>();
    bool routeFound = false;
    bool chargerNotFound = false;

    Junction* start = GetClosestJunction(startLat, startLon);
    Junction* innerStart = nullptr;
    Junction* target = GetClosestJunction(targetLat, targetLon);    
    Junction* innerTarget = nullptr;

    const ChargingJunction* selectedCharger = nullptr;

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

    Junction* u = nullptr;
    Junction* endJunction = nullptr;

    auto foundChargers = std::make_shared<std::vector<ChargingJunction*>>();
 
    LE->insert(std::make_pair(start->m_Id, start));
    S->insert(std::make_pair(start->m_Id, start->m_Id));

    while (S->size() < m_Junctions->size() && !routeFound)
    {
        while (S->size() < m_Junctions->size() && !routeFound && batteryCharge > minCharge)
        {
            //getmin(and delete)
            u = GetMin(LE);

            if (dynamic_cast<ChargingJunction*>(u))  //type check
            {
                foundChargers->emplace_back(dynamic_cast<ChargingJunction*>(u));
            }

            for (auto it = u->m_StartingSegments->begin(); it != u->m_StartingSegments->end() && routeFound == false; ++it)
            {
                endJunction = (*it)->GetEndJunction(u);
                //insert neighbor 

                if (u->m_FastestRouteInMinutes + GetTravelTimeInMinutes((*it)->m_Length, (*it)->m_MaxSpeed) < endJunction->m_FastestRouteInMinutes)
                {
                    endJunction->m_FastestRouteInMinutes =
                        u->m_FastestRouteInMinutes + GetTravelTimeInMinutes((*it)->m_Length, (*it)->m_MaxSpeed);
                    endJunction->m_FastestRouteNeighbor = *it;
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

        //find suitable charger
        if (foundChargers->size() > 0)
        {
            selectedCharger = SelectCharger(foundChargers, chargeSpeedData, ChargerType::undefined);

            //charge the battery here

            batteryCharge = RechargeBattery(batteryCharge, batteryCapacity, chargeSpeedData, 100);

            //adjust new starting point which is the selected charger
        }
        else
        {
            chargerNotFound = true;
        }

        

    }

    if (S->size() < m_Junctions->size() && batteryCharge > minCharge) //route found
    {
        //collect result
        std::shared_ptr<std::vector<const Segment*>> resultSegments = std::make_shared<std::vector<const Segment*>>();

        const Junction* currentJunction;

        if (targetSegment)
        {
            auto targetIt = std::find(targetSegment->m_InnerNodes->begin(), 
                                                        targetSegment->m_InnerNodes->end(), target);
            ptrdiff_t startIndex = targetIt - targetSegment->m_InnerNodes->begin();

            SaveInnerJunctionsAtTarget(u, targetSegment, resultJunctions, startIndex);

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

            resultJunctions->push_back(currentJunction);
            currentSegment = currentJunction->m_FastestRouteNeighbor;

            SaveInnerJunctions(currentJunction, currentSegment, resultJunctions);

            resultSegments->push_back(currentSegment);     
            currentJunction = currentSegment->GetEndJunction(currentJunction);
        }

        if (startSegment)
        {
            resultJunctions->push_back(currentJunction);

            auto startIt = std::find(startSegment->m_InnerNodes->begin(),
                startSegment->m_InnerNodes->end(), innerStart);
            ptrdiff_t index = startIt - startSegment->m_InnerNodes->begin();

            SaveInnerJunctionsAtStart(resultJunctions->back(), startSegment, resultJunctions, index);
        }
        else
        {
            resultJunctions->push_back(currentJunction);
        }
    }
}

float_t RoutePlanner::RechargeBattery(float_t& batteryChargeInPercent, const int16_t batteryCapacityInKWh, 
                                      std::shared_ptr<std::vector<const int16_t>> chargeSpeedDataKW, const int16_t maxChargingSpeedInKW)
{
    //fix charge to 80 percent capacity

    const float_t onePercentBatteryCapacityInKWh = batteryCapacityInKWh / static_cast<float_t>(100); //in kWh
    float_t chargingTime = 0; //in minutes
  
    for (auto it = chargeSpeedDataKW->cbegin() + (int)batteryChargeInPercent; it != chargeSpeedDataKW->cbegin() + 80; it++)
    {
        if (*it < maxChargingSpeedInKW)
           chargingTime += onePercentBatteryCapacityInKWh / *it * 60;
        else
           chargingTime += onePercentBatteryCapacityInKWh / maxChargingSpeedInKW * 60;
    }

    return chargingTime;
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

ChargingJunction* RoutePlanner::SelectCharger(std::shared_ptr<std::vector<ChargingJunction*>> foundChargers, 
                                                    std::shared_ptr<std::vector<const int16_t>> chargeSpeedDataKW, ChargerType carChargerType)
{
    auto maxValue = *std::max_element(chargeSpeedDataKW->cbegin(), chargeSpeedDataKW->cend());
    std::remove_if(foundChargers->cbegin(), foundChargers->cend(), [carChargerType](const ChargingJunction* current)
    {
        bool found = false;
        for (auto it = current->m_ChargingInfo.cbegin(); it != current->m_ChargingInfo.cend(); it++)
        {
            if (it->m_Type == carChargerType)
            {
                return false;
            }
           
        }

        return true;
    });

    if (foundChargers->size() > 0)
    {
        //find closest one
       /* auto minIt =
            std::min_element(foundChargers->begin(), foundChargers->end(), [lat, lon](const std::pair<int64_t, Junction*> elem1, const std::pair<int64_t, Junction*> elem2)
        {
            return Util::CalculateDistanceBetweenTwoLatLonsInMetres(lat, elem1.second->m_Lat, lon, elem1.second->m_Lon)
                < Util::CalculateDistanceBetweenTwoLatLonsInMetres(lat, elem2.second->m_Lat, lon, elem2.second->m_Lon);
        });*/
        
    }
    else 
    {
        return nullptr;
    }
}

Junction* RoutePlanner::GetMin(std::shared_ptr<std::unordered_map<int64_t, Junction*>> LE)
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
