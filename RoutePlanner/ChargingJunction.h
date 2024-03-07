#pragma once

#include "Junction.h"

enum class ChargerType
{
	undefined, ccs, type2, chademo
};

struct ChargingData
{
	uint16_t m_Output;

	ChargerType m_Type;
};

class ChargingJunction : public Junction 
{
public:

	ChargingJunction(int64_t id, float_t lon, float_t lat);

	std::vector<ChargingData> m_ChargingInfo;
};

struct ChargingNode : public Node
{
	std::vector<ChargingData> m_ChargingInfo;
};

