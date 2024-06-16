#pragma once

#include "Segment.h"
#include "TinyXML2/tinyxml2.h"
#include "include/json/value.h"
#include "include/json/writer.h"
#include "ChargingJunction.h"


#include <set>
#include <map>
#include <vector>
#include <unordered_map>
#include <iostream>
#include <fstream>
#include <string>

class Converter
{
public:

	Converter();

	~Converter();

	void GetElevationData();

	void ConvertOsmDataToJson(std::string osmFileName, std::string jsonFileName);

	static void ReadPreprocessedDataFromJson(std::string fileName, std::shared_ptr<std::unordered_map<int64_t, Junction*>> junctions, std::shared_ptr<std::vector<Segment*>> segments);

	static void SetSegmentSlopeData(std::shared_ptr<std::vector<Segment*>> segments);
	
	static void SaveResultToGeoJson(std::shared_ptr<std::vector<const Junction*>> resultJunctions, std::string fileName);

	static std::shared_ptr<std::vector<int16_t>> LoadChargingSpeedData(std::string fileName);

private:

	static void LoadJsonFile(std::string fileName, Json::Value& root);

	void LoadOsmFile(std::string name);

	static void GetPreprocessedData(const Json::Value& root, std::shared_ptr<std::unordered_map<int64_t, Junction*>> Junctions, std::shared_ptr<std::vector<Segment*>> Segments);

	void SelectHighwayNodesNeeded();

	void SelectChargingNodes();

	void LoadHighwayNodes();

	void LoadHighways();

	void ConnectChargingNodes();

	static float_t CalculateSlope(float_t distanceInMetres, float_t startElevationInMetres, float_t endElevationInMetres);

	std::string PrepareForElevationData();

	void GetElevationData(const std::string& url, const std::string& requestJson);

	void SaveToJson(std::string fileName);

	void CalculateAndSetLength(Way* way);

	bool IsRoad(const char* roadType, const tinyxml2::XMLElement* tag);

private:

	std::unique_ptr<std::unordered_map<int64_t, uint8_t>> m_Node_Ids;
	std::unique_ptr<std::unordered_map<int64_t, Node>> m_Nodes;
	std::unique_ptr<std::vector<Way>> m_Ways;
	std::unique_ptr<std::vector<ChargingNode>> m_ChargingNodes;

	std::string m_ElevationDataJson;

	tinyxml2::XMLDocument m_xDoc;

	tinyxml2::XMLElement* m_Osm; 
	tinyxml2::XMLElement* m_Way;
	tinyxml2::XMLElement* m_Nd;

	
};

