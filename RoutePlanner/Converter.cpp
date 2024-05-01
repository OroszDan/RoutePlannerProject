#define CURL_STATICLIB

#include "Converter.h"
#include "Util.h"
#include "TinyXML2/tinyxml2.h"
#include "include/json/value.h"
#include "include/json/writer.h"
#include "include/json/reader.h"
#include "curl/curl.h"

#include <string>
#include <cmath>
#include <numbers>
#include "ChargingJunction.h"


Converter::Converter()
{
	//m_Node_Ids = new std::set<int64_t>();
	/*m_Nodes = new std::unordered_map<int64_t, Node*>();*/
	/*Ways = new std::vector<Way*>();*/

	m_Osm = nullptr;
	m_Way = nullptr;
	m_Nd = nullptr;

}

Converter::~Converter()
{
	m_Osm = nullptr;
	m_Way = nullptr;
	m_Nd = nullptr;
}

void Converter::GetElevationData()
{

}

void Converter::LoadJsonFile(const char* fileName, Json::Value& root)
{
	std::ifstream inputStream(fileName);
	Json::Reader reader;

	bool success = reader.parse(inputStream, root);

	if (!success)
	{
		throw std::exception("Json file not loaded correctly");
	}
}

void Converter::LoadOsmFile(const char* name)
{
	auto err = m_xDoc.LoadFile(name);

	if (err != tinyxml2::XML_SUCCESS)
	{
		throw std::exception("Osm file not loaded correctly");
	}

	m_Osm = m_xDoc.FirstChildElement("osm");

}

void Converter::GetPreprocessedData(const Json::Value& root, std::shared_ptr<std::unordered_map<int64_t, Junction*>> Junctions, std::shared_ptr<std::vector<Segment*>> Segments)
{	
	for (auto it = root["chargingNodes"].begin(); it != root["chargingNodes"].end(); it++)
	{
		Json::Value chargingNode = it->get("chargingNode", Json::nullValue);

		if (chargingNode != Json::nullValue)
		{
			int64_t id = chargingNode["id"].asInt64();
			float_t lat = chargingNode["lat"].asFloat();
			float_t lon = chargingNode["lon"].asFloat();

			std::vector<ChargingData> chDatas = std::vector<ChargingData>();

			for (auto it = chargingNode["chargingInfos"].begin(); it != chargingNode["chargingInfos"].end(); it++)
			{
				Json::Value chargingData = it->get("chargingData", Json::nullValue);

				if (chargingData != Json::nullValue)
				{
					ChargingData data = ChargingData();
					data.m_Output = chargingData["output"].asInt();	
					data.m_Type = static_cast<ChargerType>(chargingData["type"].asInt());

					chDatas.emplace_back(data);
				}
			}

			ChargingJunction* chJunction = new ChargingJunction(id, lon, lat, chDatas);
			Junctions->insert(std::make_pair(chJunction->m_Id, chJunction));
		}
	}

	for (auto it = root["nodes"].begin(); it != root["nodes"].end(); ++it) 
	{
		Json::Value node = it->get("node", Json::nullValue);
		
		if (node != Json::nullValue)
		{
			int64_t id = node["id"].asInt64();
			float_t lat = node["lat"].asFloat();
			float_t lon = node["lon"].asFloat();
			
			Junction* junction = new Junction(id, lon, lat);
			Junctions->insert(std::make_pair(junction->m_Id, junction));
		}	
	}

	for (auto it = root["ways"].begin(); it != root["ways"].end(); ++it)
	{
		Json::Value node = it->get("way", Json::nullValue);

		if (node != Json::nullValue)
		{
			int64_t id = node["id"].asInt64();
			int64_t idFrom = node["idfrom"].asInt64();
			int64_t idTo = node["idto"].asInt64();
			float_t length = node["length"].asFloat();
			bool oneWay = node["oneway"].asBool();
			uint8_t maxspeed = node["maxspeed"].asInt();

			Segment* segment = new Segment(id, length, Junctions->at(idFrom), Junctions->at(idTo), maxspeed);

			for (auto nodeIt = node["innernodes"].begin(); nodeIt != node["innernodes"].end(); nodeIt++)
			{
				segment->m_InnerNodes->push_back(Junctions->at(nodeIt->asInt64()));
			}		

			if (oneWay)
			{
				Junctions->at(idFrom)->AddSegment(segment);
			}
			else
			{
				Junctions->at(idFrom)->AddSegment(segment);
				Junctions->at(idTo)->AddSegment(segment);
			}
			
			Segments->push_back(segment);
			
		}
	}
}

void Converter::SelectHighwayNodesNeeded()
{
	m_Node_Ids = std::make_unique<std::unordered_map<int64_t, uint8_t>>();

	m_Way = m_Osm->FirstChildElement("way");

	while (m_Way != nullptr)
	{
		const tinyxml2::XMLElement* startTag = m_Way->FirstChildElement("tag");
		const tinyxml2::XMLElement* tag = startTag;

		while (tag != nullptr && strcmp(tag->FindAttribute("k")->Value(), "highway"))
		{
			tag = tag->NextSiblingElement("tag");
		}

		if (tag != nullptr)
		{
			auto roadType = tag->FindAttribute("v")->Value();
			if (IsRoad(roadType, startTag))
			{
				m_Nd = m_Way->FirstChildElement("nd");
				int i = 0;
				while (m_Nd != nullptr)
				{
					int64_t ref = m_Nd->FindAttribute("ref")->Int64Value();

					auto it = m_Node_Ids->find(ref);

					if ((i == 0) || (m_Nd->NextSiblingElement("nd") == nullptr)) //first or last chargingNode
					{
						if (it != m_Node_Ids->end())
						{
							it->second++;
						}
						else
						{
							m_Node_Ids->insert(std::make_pair(ref, (uint8_t)1));
						}
					}
					else
					{
						m_Node_Ids->insert(std::make_pair(ref, (uint8_t)0));
					}

					m_Nd = m_Nd->NextSiblingElement("nd");

					i++;
				}

			}

		}

		m_Way = m_Way->NextSiblingElement("way");
	}
}

void Converter::SelectChargingNodes()
{
	m_ChargingNodes = std::make_unique<std::vector<ChargingNode>>();

	const tinyxml2::XMLAttribute* lat;
	const tinyxml2::XMLAttribute* lon;
	const tinyxml2::XMLAttribute* id;

	tinyxml2::XMLElement* node;

	node = m_Osm->FirstChildElement("node");

	while (node != nullptr)
	{

		tinyxml2::XMLElement* tag = node->FirstChildElement("tag");

		//if (tag != nullptr && !strcmp(tag->FindAttribute("k")->Value(), "amenity")) //strcmp kell vvagy 
		//{
		//	std::cout << tag->FindAttribute("v")->Value() << std::endl;
		//}
		
		while (tag != nullptr && 
			!( !strcmp(tag->FindAttribute("k")->Value(), "amenity") && !strcmp(tag->FindAttribute("v")->Value(), "charging_station")))
		{
			tag = tag->NextSiblingElement("tag");
		}
		
		if (tag != nullptr)
		{
			std::string value;
			tinyxml2::XMLElement* innerTag = node->FirstChildElement("tag");
			bool chargerFound = false;
			
			while (innerTag != nullptr)
			{
				std::string value = innerTag->FindAttribute("k")->Value();
				auto result = value.find("output");

				if (result != std::string::npos)
				{
					ChargingNode chargingNode;
					ChargerType chargerType = ChargerType::undefined;

					if (value.find("ccs") != std::string::npos)
					{
						chargerType = ChargerType::ccs;
					}
					else
					{
						if (value.find("type2") != std::string::npos)
						{
							chargerType = ChargerType::type2;
						}
						else
						{
							if (value.find("chademo") != std::string::npos)
							{
								chargerType = ChargerType::chademo;
							}
							else
							{
								if (value.find("tesla_supercharger") != std::string::npos)
								{
									chargerType = ChargerType::type2;
								}
							}
						}
					}

					if (chargerType != ChargerType::undefined)
					{
						if (!chargerFound)
						{
							chargerFound = true;
							chargingNode = ChargingNode();

							chargingNode.m_Id = node->FindAttribute("id")->Int64Value();
							chargingNode.m_Lat = node->FindAttribute("lat")->float_tValue();
							chargingNode.m_Lon = node->FindAttribute("lon")->float_tValue();

							m_ChargingNodes->emplace_back(chargingNode);
						}
						
						ChargingData data = ChargingData();

						data.m_Type = chargerType;

						std::stringstream ss(innerTag->FindAttribute("v")->Value());
						std::string number;
						std::getline(ss, number, ':');
						data.m_Output = std::stoi(number);

						m_ChargingNodes->back().m_ChargingInfo.emplace_back(data);
					}
				}

				innerTag = innerTag->NextSiblingElement("tag");
			}
			
			
			//get further data
		}

		node = node->NextSiblingElement("node");
		
	}
}

void Converter::LoadHighwayNodes()
{
	const tinyxml2::XMLAttribute* lat;
	const tinyxml2::XMLAttribute* lon;
	const tinyxml2::XMLAttribute* id;

	tinyxml2::XMLElement* node;

	m_Nodes = std::make_unique<std::unordered_map<int64_t, Node>>();

	node = m_Osm->FirstChildElement("node");

	int size = 0;

	while (node != nullptr)
	{
		id = node->FindAttribute("id");

		if (m_Node_Ids->find(id->Int64Value()) != m_Node_Ids->end())
		{
			lat = node->FindAttribute("lat");
			lon = node->FindAttribute("lon");

			Node highway_node = Node();
			highway_node.m_Id = id->Int64Value();
			highway_node.m_Lat = lat->float_tValue();
			highway_node.m_Lon = lon->float_tValue();

			m_Nodes->insert(std::make_pair(highway_node.m_Id, highway_node));
		}

		node = node->NextSiblingElement("node");
	}
}

void Converter::LoadHighways()
{
	m_Ways = std::make_unique<std::vector<Way>>();

	const tinyxml2::XMLAttribute* id;

	m_Way = m_Osm->FirstChildElement("way");

	while (m_Way != nullptr)
	{
		const tinyxml2::XMLElement* startTag = m_Way->FirstChildElement("tag");
		const tinyxml2::XMLElement* tag = startTag;
		auto Iid = m_Way->FindAttribute("id")->Int64Value();

		while (tag != nullptr && strcmp(tag->FindAttribute("k")->Value(), "highway"))
		{
			tag = tag->NextSiblingElement("tag");
		}

		if (tag != nullptr)
		{
			id = m_Way->FindAttribute("id");
			m_Nd = m_Way->FirstChildElement("nd");
			auto roadType = tag->FindAttribute("v")->Value();

			if (IsRoad(roadType, startTag))
			{
				Way way_temp = Way();

				way_temp.m_Id = m_Way->FindAttribute("id")->Int64Value();

				tag = startTag;

				while (tag != nullptr && strcmp(tag->FindAttribute("k")->Value(), "oneway"))
				{
					tag = tag->NextSiblingElement("tag");
				}

				if (tag != nullptr)
				{
					if (strcmp(tag->FindAttribute("v")->Value(), "yes"))
					{
						way_temp.m_OneWay = false;
					}
					else
					{
						way_temp.m_OneWay = true;
					}
				}
				else
				{
					way_temp.m_OneWay = false;
				}

				tag = startTag;

				while (tag != nullptr && strcmp(tag->FindAttribute("k")->Value(), "maxspeed"))
				{
					tag = tag->NextSiblingElement("tag");
				}

				if (tag != nullptr)
				{
					way_temp.m_Maxspeed = tag->FindAttribute("v")->IntValue();
				}
				else
				{
					way_temp.m_Maxspeed = 50;
				}

				m_Nd = m_Way->FirstChildElement("nd");

				int i = 0;
				while (m_Nd != nullptr)
				{
					auto nodeRef = m_Nd->FindAttribute("ref")->Int64Value();

					if (i == 0 || m_Nd->NextSiblingElement("nd") == nullptr)
					{
						way_temp.m_InnerNodes->push_back(nodeRef);
					}
					else
					{
						auto numberOfRoadsCrossing = m_Node_Ids->at(nodeRef);

						if (numberOfRoadsCrossing > 0)
						{
							way_temp.m_InnerNodes->push_back(nodeRef);

							CalculateAndSetLength(&way_temp);

							m_Ways->push_back(way_temp);

							way_temp.m_InnerNodes = std::make_shared<std::vector<int64_t>>();

							way_temp.m_InnerNodes->push_back(nodeRef);
							
						}
						else
						{
							way_temp.m_InnerNodes->push_back(nodeRef);
						}
					}

					m_Nd = m_Nd->NextSiblingElement("nd");
					i++;
				}		

				CalculateAndSetLength(&way_temp);

				m_Ways->push_back(way_temp);
			}
		}

		m_Way = m_Way->NextSiblingElement("way");
	}
}

void Converter::ConnectChargingNodes()
{
	uint32_t idCounter = 0;

	for (auto& charger : *m_ChargingNodes)
	{
		auto it = std::min_element(m_Nodes->cbegin(), m_Nodes->cend(), [charger](std::pair<int64_t, Node> node1, std::pair<int64_t, Node> node2) {

			return Util::CalculateDistanceBetweenTwoLatLonsInMetres(charger.m_Lat, node1.second.m_Lat, charger.m_Lon, node1.second.m_Lon)
				< Util::CalculateDistanceBetweenTwoLatLonsInMetres(charger.m_Lat, node2.second.m_Lat, charger.m_Lon, node2.second.m_Lon);
		});

		Way connectingWay = Way();

		connectingWay.m_Id = idCounter;
		connectingWay.m_InnerNodes->emplace_back(it->second.m_Id);
		connectingWay.m_InnerNodes->emplace_back(charger.m_Id);
		connectingWay.m_Length = Util::CalculateDistanceBetweenTwoLatLonsInMetres(charger.m_Lat, it->second.m_Lat, charger.m_Lon, it->second.m_Lon);
		connectingWay.m_Maxspeed = 50;
		connectingWay.m_OneWay = false;

		m_Ways->emplace_back(connectingWay);
		auto je = m_Ways->back();

		++idCounter;
	}

	//insert extra nodes
	/*for (auto& charger : *m_ChargingNodes)
	{
		m_Nodes->insert(std::make_pair(charger.m_Id, charger));
	}*/
}

std::string Converter::PrepareForElevationData()
{
	Json::Value nodedatas(Json::arrayValue);

	for (auto it = m_Nodes->cbegin(); it != m_Nodes->cend(); it++)
	{
		Json::Value nodedata;
		nodedata["latitude"] = it->second.m_Lat;
		nodedata["longitude"] = it->second.m_Lon;

		nodedatas.append(nodedata);
	}

	Json::Value doc;
	doc["locations"] = nodedatas;
	Json::StreamWriterBuilder builder;
	std::string result = Json::writeString(builder, doc);
	return result;
}

static size_t writeCallback(char* buffer, size_t size, size_t nmemb, void* param)
{
	std::string resultJson(buffer, nmemb);
	size_t totalsize = size * nmemb;
	return totalsize;
}

void Converter::GetElevationData(const std::string& url, const std::string& requestJson)
{
	CURL* curl;
	CURLcode res;
	std::string result;

	curl_global_init(CURL_GLOBAL_DEFAULT);

	curl = curl_easy_init();

	if (curl)
	{
		struct curl_slist* chunk = NULL;

		chunk = curl_slist_append(chunk, "Accept: application/json");
		chunk = curl_slist_append(chunk, "Content-Type: application/json");

		curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
		curl_easy_setopt(curl, CURLOPT_HTTPHEADER, chunk);
		curl_easy_setopt(curl, CURLOPT_POST, 1);
		curl_easy_setopt(curl, CURLOPT_POSTFIELDS, requestJson.c_str());
		curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
		curl_easy_setopt(curl, CURLOPT_WRITEDATA);

		res = curl_easy_perform(curl);

		curl_slist_free_all(chunk);
		curl_easy_cleanup(curl);
	}

	curl_global_cleanup();
}

void Converter::SaveToJson(const char* fileName)
{
	//save nodes
	Json::Value nodes(Json::arrayValue);

	for (auto it = m_Nodes->cbegin(); it != m_Nodes->cend(); it++)
	{
		Json::Value node;
		node["node"]["id"] = it->second.m_Id;
		node["node"]["lat"] = it->second.m_Lat;
		node["node"]["lon"] = it->second.m_Lon;

		nodes.append(node);
	}

	//save ways
	Json::Value ways(Json::arrayValue);

	for (auto it = m_Ways->cbegin(); it != m_Ways->cend(); it++)
	{
		Json::Value way;
		way["way"]["id"] = it->m_Id;
		way["way"]["length"] = it->m_Length;
		way["way"]["oneway"] = it->m_OneWay;
		way["way"]["maxspeed"] = it->m_Maxspeed;

		Json::Value nodes(Json::arrayValue);

		for (size_t i = 0; i < it->m_InnerNodes->size(); i++)
		{
			if (i == 0)
			{
				way["way"]["idfrom"] = it->m_InnerNodes->at(i);
			}
			else if (i == it->m_InnerNodes->size() - 1)
			{
				way["way"]["idto"] = it->m_InnerNodes->at(i);
			}
			else
			{
				nodes.append(it->m_InnerNodes->at(i));
			}
			
		}

		way["way"]["innernodes"] = nodes;

		ways.append(way);
	}

	//save chargingNodes

	Json::Value chargingNodes(Json::arrayValue);

	for (auto it = m_ChargingNodes->cbegin(); it != m_ChargingNodes->cend(); it++)
	{
		Json::Value chargingNode;
		chargingNode["chargingNode"]["id"] = it->m_Id;
		chargingNode["chargingNode"]["lat"] = it->m_Lat;
		chargingNode["chargingNode"]["lon"] = it->m_Lon;

		Json::Value chargingInfos(Json::arrayValue);

		for (auto innerIt = it->m_ChargingInfo.cbegin(); innerIt != it->m_ChargingInfo.cend(); innerIt++)
		{
			Json::Value chargingData;
			chargingData["chargingData"]["output"] = innerIt->m_Output;
			chargingData["chargingData"]["type"] = static_cast<int>(innerIt->m_Type);
			chargingInfos.append(chargingData);
		}

		chargingNode["chargingNode"]["chargingInfos"] = chargingInfos;

		chargingNodes.append(chargingNode);
	}

	//write to json
	Json::Value doc;
	doc["chargingNodes"] = chargingNodes;
	doc["nodes"] = nodes;
	doc["ways"] = ways;

	Json::StreamWriterBuilder builder;

	builder["commentStyle"] = "None";
	builder["indentation"] = "   ";

	std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
	std::ofstream outputFileStream;

	outputFileStream.open(fileName);
	writer->write(doc, &outputFileStream);
	outputFileStream.close();
}

void Converter::ConvertOsmDataToJson(const char* osmFileName, const char* jsonFileName)
{
	LoadOsmFile(osmFileName);
	SelectHighwayNodesNeeded();
	LoadHighwayNodes();
	LoadHighways();
	SelectChargingNodes();
	ConnectChargingNodes();
	//std::string json = PrepareForElevationData();
	//GetElevationData("https://api.open-elevation.com/api/v1/lookup?locations=41.161758,-8.583933|10,10|20,20", json);
	//GetElevationData("https://api.open-elevation.com/api/v1/lookup", json);
	SaveToJson(jsonFileName);
}

void Converter::ReadPreprocessedDataFromJson(const char* fileName, std::shared_ptr<std::unordered_map<int64_t, Junction*>> junctions, std::shared_ptr<std::vector<Segment*>> segments)
{
	Json::Value root;
	LoadJsonFile(fileName, root);
	GetPreprocessedData(root, junctions, segments);
	;
}

void Converter::SaveResultToGeoJson(std::shared_ptr<std::vector<const Junction*>> resultJunctions, const char* fileName)
{
	Json::Value root;

	Json::Value feature;

	feature["type"] = "Feature";
	feature["properties"] = Json::nullValue;

	Json::Value coordinateArray(Json::arrayValue);

	for (auto it = resultJunctions->rbegin(); it != resultJunctions->rend(); ++it)
	{
		Json::Value coordinate(Json::arrayValue);

		coordinate.append((*it)->m_Lon);
		coordinate.append((*it)->m_Lat);

		coordinateArray.append(coordinate);
	}

	feature["geometry"]["coordinates"] = coordinateArray;
	feature["geometry"]["type"] = "LineString";

	Json::Value featureArray(Json::arrayValue);
	featureArray.append(feature);

	root["type"] = "FeatureCollection";
	root["features"] = featureArray;


	Json::StreamWriterBuilder builder;

	builder["commentStyle"] = "None";
	builder["indentation"] = "   ";

	std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
	std::ofstream outputFileStream;

	outputFileStream.open(fileName);
	writer->write(root, &outputFileStream);
	outputFileStream.close();
}

std::shared_ptr<std::vector<int16_t>> Converter::LoadChargingSpeedData(std::string carName)
{
	std::shared_ptr<std::vector<int16_t>> chargingData = std::make_shared<std::vector<int16_t>>();

	std::ifstream file("ChargingSpeedDatas/" + carName + "_chargingdata.txt");

	if (!file.is_open()) 
		throw new std::exception("Error opening file.");

	std::string line;
	while (std::getline(file, line)) 
		chargingData->emplace_back(std::stoi(line));

	file.close();
	return chargingData;
}

void Converter::CalculateAndSetLength(Way* way)
{
	float_t length = 0;
	for (size_t i = 1; i < way->m_InnerNodes->size(); i++)
	{
			length += Util::CalculateDistanceBetweenTwoLatLonsInMetres(
			m_Nodes->at(way->m_InnerNodes->at(i - 1)).m_Lat, m_Nodes->at(way->m_InnerNodes->at(i)).m_Lat,
			m_Nodes->at(way->m_InnerNodes->at(i - 1)).m_Lon, m_Nodes->at(way->m_InnerNodes->at(i)).m_Lon);
	}

	way->m_Length = length;
}

bool Converter::IsRoad(const char* roadType, const tinyxml2::XMLElement* tag)
{
	if (strcmp(roadType, "motorway") &&
		strcmp(roadType, "trunk") && 
		strcmp(roadType, "primary") &&
		strcmp(roadType, "secondary") &&
		strcmp(roadType, "tertiary") &&
		strcmp(roadType, "unclassified") &&
		strcmp(roadType, "residential") &&
		strcmp(roadType, "motorway_link") &&
		strcmp(roadType, "trunk_link") &&
		strcmp(roadType, "primary_link") &&
		strcmp(roadType, "secondary_link") &&
		strcmp(roadType, "tertiary_link") &&
		strcmp(roadType, "service"))
	{
		return false;
	}
	else if (!strcmp(roadType, "service"))
	{
		while (tag != nullptr && strcmp(tag->FindAttribute("k")->Value(), "access"))
		{
			tag = tag->NextSiblingElement("tag");
		}

		if (tag != nullptr && 
			(!strcmp(tag->FindAttribute("v")->Value(), "private") || 
			 !strcmp(tag->FindAttribute("v")->Value(), "no")))
		{
			//access is restricted
			return false;
		}
		else
		{
			return true;
		}
	}
	else
	{
		return true;
	}
}
