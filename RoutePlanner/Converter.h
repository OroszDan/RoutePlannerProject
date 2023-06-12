#pragma once

#include "Segment.h"
#include "tinyxml2.h"

#include <set>
#include <map>
#include <vector>
#include <unordered_map>
#include <iostream>
#include <fstream>
#include <json/value.h>
#include <json/writer.h>

class Converter
{
public:

	Converter();

	~Converter();

	void ConvertOsmDataToJson(const char* osmFileName, const char* jsonFileName);

	void ReadPreprocessedDataFromJson(const char* fileName, std::shared_ptr<std::unordered_map<int64_t, Junction*>> junctions, std::shared_ptr<std::vector<Segment*>> segments);

	static void SaveResultToGeoJson(std::shared_ptr<std::vector<const Junction*>> resultJunctions, const char* filename);

private:

	void LoadJsonFile(const char* fileName, Json::Value& root);

	void LoadOsmFile(const char* fileName);

	void GetPreprocessedData(const Json::Value& root, std::shared_ptr<std::unordered_map<int64_t, Junction*>> Junctions, std::shared_ptr<std::vector<Segment*>> Segments);

	void SelectNodesNeeded();

	void LoadHighwayNodes();

	void LoadHighways();

	void SaveToJson(const char* fileName);	

private:

	std::unique_ptr<std::unordered_map<int64_t, uint8_t>> m_Node_Ids;
	std::unique_ptr<std::unordered_map<int64_t, Node>> m_Nodes;
	std::unique_ptr<std::vector<Way>> m_Ways;

	tinyxml2::XMLDocument m_xDoc;

	tinyxml2::XMLElement* m_Osm; 
	tinyxml2::XMLElement* m_Way;
	tinyxml2::XMLElement* m_Nd;

	void CalculateAndSetLength(Way* way);
	bool IsRoad(const char* roadType, const tinyxml2::XMLElement* tag);
};

