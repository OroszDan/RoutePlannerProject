#include "Converter.h"

#include <string>
#include <cmath>
#include <numbers>
#include <tinyxml2.h>
#include <json/value.h>
#include <json/writer.h>
#include <json/reader.h>
#include "Util.h"

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

void Converter::SelectNodesNeeded()
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
					auto ref = m_Nd->FindAttribute("ref")->Int64Value();

					auto it = m_Node_Ids->find(ref);

					if ((i == 0) || (m_Nd->NextSiblingElement("nd") == nullptr)) //first or last node
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
					auto ref = m_Nd->FindAttribute("ref")->Int64Value();

					if (i == 0 || m_Nd->NextSiblingElement("nd") == nullptr)
					{
						way_temp.m_InnerNodes->push_back(ref);
					}
					else
					{
						auto numberOfRoadsCrossing = m_Node_Ids->at(ref);

						if (numberOfRoadsCrossing > 0)
						{
							way_temp.m_InnerNodes->push_back(ref);

							CalculateAndSetLength(&way_temp);

							m_Ways->push_back(way_temp);

							way_temp.m_InnerNodes = std::make_shared<std::vector<int64_t>>();

							way_temp.m_InnerNodes->push_back(ref);
							
						}
						else
						{
							way_temp.m_InnerNodes->push_back(ref);
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

void Converter::SaveToJson(const char* fileName)
{
	Json::Value nodes(Json::arrayValue);

	for (auto it = m_Nodes->cbegin(); it != m_Nodes->cend(); it++)
	{
		Json::Value node;
		node["node"]["id"] = it->second.m_Id;
		node["node"]["lat"] = it->second.m_Lat;
		node["node"]["lon"] = it->second.m_Lon;

		nodes.append(node);
	}

	Json::Value ways(Json::arrayValue);

	for (auto it = m_Ways->begin(); it != m_Ways->cend(); it++)
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

	Json::Value doc;
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
	SelectNodesNeeded();
	LoadHighwayNodes();
	LoadHighways();
	SaveToJson(jsonFileName);
}

void Converter::ReadPreprocessedDataFromJson(const char* fileName, std::shared_ptr<std::unordered_map<int64_t, Junction*>> junctions, std::shared_ptr<std::vector<Segment*>> segments)
{
	Json::Value root;
	LoadJsonFile(fileName, root);
	GetPreprocessedData(root, junctions, segments);


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
		strcmp(roadType, "tertiary_link"))
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
