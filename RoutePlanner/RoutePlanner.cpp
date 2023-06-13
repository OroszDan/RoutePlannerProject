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
    //converter->ConvertOsmDataToJson("liechtenstein-latest.osm", "highwaydata.json");

}

void RoutePlanner::FindFastestRoute(const float_t startLat, const float_t startLon, const float_t targetLat, const float_t targetLon, std::shared_ptr<std::vector<const Junction*>> resultJunctions)
{
    //A* algorithm

    std::shared_ptr<std::unordered_map<int64_t, int64_t>> S = std::make_shared<std::unordered_map<int64_t, int64_t>>();
    std::shared_ptr<std::unordered_map<int64_t, Junction*>> LE = std::make_shared<std::unordered_map<int64_t, Junction*>>();
    bool routeFound = false;

    for (auto it = m_Junctions->cbegin(); it != m_Junctions->cend(); ++it) 
    {
        S->insert(std::make_pair(it->first, it->first));
    }

    //start

    Junction* start;
    Junction* innerStart = nullptr;
    Junction* target;
    Junction* innerTarget = nullptr;

    GetStartAndTargetJunction(startLat, startLon, targetLat, targetLon, start, target);

    Segment* startSegment = nullptr;
    Segment* targetSegment = nullptr;


    if (start->m_Segments->size() == 0)
    {
        innerStart = start;
        //inner node
        startSegment = FindContainingSegment(innerStart);

        start = startSegment->m_From;
    }

    if (target->m_Segments->size() == 0)
    {
        //innerTarget = target;

        targetSegment = FindContainingSegment(target);
    }

    start->m_FastestRouteInMinutes = 0;

    /*if (startSegment)
    {
        start = startSegment->m_From;
    }
    else
    {
        start = m_Junctions->at(idFrom);
    }*/

    Junction* u = nullptr;
    Junction* endJunction = nullptr;

    //const Junction* target = m_Junctions->at(idTo);
  
 
    LE->insert(std::make_pair(start->m_Id, start));

    while (S->size() > 0 && !routeFound)
    {
        //minkivesz
        u = GetMin(S, LE);

        for(auto it = u->m_Segments->begin(); it != u->m_Segments->end() && routeFound == false; ++it)
        {
            endJunction = (*it)->GetEndJunction(u);
            //insert neighbor 

            if (u->m_FastestRouteInMinutes + GetTravelTimeInMinutes((*it)->m_Length, (*it)->m_MaxSpeed) < endJunction->m_FastestRouteInMinutes)
            {
                endJunction->m_FastestRouteInMinutes = 
                    u->m_FastestRouteInMinutes + GetTravelTimeInMinutes((*it)->m_Length, (*it)->m_MaxSpeed);
                endJunction->m_FastestRouteNeighbor = *it;   
            }

            if (S->contains(endJunction->m_Id) && !LE->contains(endJunction->m_Id))
            {
                endJunction->m_FastestRouteInMinutesHeuristic = GetHeuristicTravelTime(endJunction, target);
                LE->insert(std::make_pair(endJunction->m_Id,endJunction));
            }

            if (endJunction->m_Id == target->m_Id || (*it) == targetSegment)
            {
                routeFound = true;
            }
        }
    }

    if (S->size() > 0)
    {
        //collect result
        std::shared_ptr<std::vector<const Segment*>> resultSegments = std::make_shared<std::vector<const Segment*>>();

        const Junction* currentJunction;

        if (targetSegment)
        {
            auto targetIt = std::find(targetSegment->m_InnerNodes->begin(), 
                                                        targetSegment->m_InnerNodes->end(), target);
            ptrdiff_t startIndex = targetIt - targetSegment->m_InnerNodes->begin();
            /*if (u == targetSegment->m_From)
            {
                for (size_t i = index; i >= 0; i--)
                {
                    resultJunctions->push_back(targetSegment->m_InnerNodes->at(i));
                }
            }
            else
            {
                for (size_t i = index; i < targetSegment->m_InnerNodes->size(); i++)
                {
                    resultJunctions->push_back(targetSegment->m_InnerNodes->at(i));
                }
            }*/

            SaveInnerJunctionsAtRoadEnds(u, targetSegment, resultJunctions, startIndex);

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

            if (currentJunction == currentSegment->m_From)
            {
                for (auto it = currentSegment->m_InnerNodes->begin(); it != currentSegment->m_InnerNodes->end(); ++it)
                {
                    resultJunctions->push_back(*it);
                }
                //simple
            }
            else
            {
                for (auto it = currentSegment->m_InnerNodes->rbegin(); it != currentSegment->m_InnerNodes->rend(); ++it)
                {
                    resultJunctions->push_back(*it);
                }
                //reverse
            }

            resultSegments->push_back(currentSegment);     
            currentJunction = currentSegment->GetEndJunction(currentJunction);
        }

        if (startSegment)
        {/*
            if (resultJunctions->back() != startSegment->m_From && resultJunctions->back() != startSegment->m_To)
            {*/
                resultJunctions->push_back(currentJunction);
            //}

            auto startIt = std::find(startSegment->m_InnerNodes->begin(),
                startSegment->m_InnerNodes->end(), innerStart);
            ptrdiff_t index = startIt - startSegment->m_InnerNodes->begin();

            //SaveInnerJunctionsAtRoadEnds(resultJunctions->back(), startSegment, resultJunctions, index);

            if (resultJunctions->back() == startSegment->m_From)
            {
                for (int i = 0; i < index; i++)
                {
                    resultJunctions->push_back(startSegment->m_InnerNodes->at(i));
                }
            }
            else
            {
                for (int i = index; i < startSegment->m_InnerNodes->size(); ++i)
                {
                    resultJunctions->push_back(startSegment->m_InnerNodes->at(i));
                }

            }
        }
        else
        {
            resultJunctions->push_back(currentJunction);
        }
    
    }

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
            
            //dataFound = SameJunction(lat, lon, *nodeIt);
        }

        if (dataFound)
        {
            containingSegment = *it;
        }
    }

    return containingSegment;
}

//bool RoutePlanner::SameJunction(const float_t lat, const float_t lon, const Junction* currentJunction)
//{
//    return currentJunction->m_Lat == lat && currentJunction->m_Lon == lon;
//}

Junction* RoutePlanner::GetMin(std::shared_ptr<std::unordered_map<int64_t, int64_t>> S, std::shared_ptr<std::unordered_map<int64_t, Junction*>> LE)
{
    auto minIt = 
    std::min_element(LE->begin(), LE->end(), [](const std::pair<int64_t, Junction*> elem1, const std::pair<int64_t, Junction*> elem2)
    {
        return elem1.second->m_FastestRouteInMinutes + elem1.second->m_FastestRouteInMinutesHeuristic 
             < elem2.second->m_FastestRouteInMinutes + elem2.second->m_FastestRouteInMinutesHeuristic; 
    });

    Junction* minElem = (*minIt).second;

    S->erase((*minIt).first);
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

void RoutePlanner::GetStartAndTargetJunction(const float_t startLat, const float_t startLon, const float_t targetLat, const float_t targetLon, Junction*& startJunction, Junction*& targetJunction)
{
    auto startMinIt =
    std::min_element(m_Junctions->begin(), m_Junctions->end(), [startLat, startLon](const std::pair<int64_t, Junction*> elem1, const std::pair<int64_t, Junction*> elem2)
    {
        return Util::CalculateDistanceBetweenTwoLatLonsInMetres(startLat, elem1.second->m_Lat, startLon, elem1.second->m_Lon)
             < Util::CalculateDistanceBetweenTwoLatLonsInMetres(startLat, elem2.second->m_Lat, startLon, elem2.second->m_Lon);
    });

    startJunction = (*startMinIt).second;

    auto targetMinIt =
    std::min_element(m_Junctions->begin(), m_Junctions->end(), [targetLat, targetLon](const std::pair<int64_t, Junction*> elem1, const std::pair<int64_t, Junction*> elem2)
    {
        return Util::CalculateDistanceBetweenTwoLatLonsInMetres(targetLat, elem1.second->m_Lat, targetLon, elem1.second->m_Lon)
            < Util::CalculateDistanceBetweenTwoLatLonsInMetres(targetLat, elem2.second->m_Lat, targetLon, elem2.second->m_Lon);
    });

    targetJunction = (*targetMinIt).second;
}

void RoutePlanner::SaveInnerJunctionsAtRoadEnds(const Junction* referenceJunction, const Segment* segment, std::shared_ptr<std::vector<const Junction*>> junctions, const ptrdiff_t startIndex)
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
