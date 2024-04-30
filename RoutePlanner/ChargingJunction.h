#pragma once

#include "Junction.h"
#include "ChargerType.h"

struct ChargingData
{
	uint16_t m_Output;

	ChargerType m_Type;
};

class ChargingJunction : public Junction 
{
public:

	ChargingJunction(int64_t id, float_t lon, float_t lat, const std::vector<ChargingData>& datas);

	~ChargingJunction() override = default;

	std::vector<ChargingData> m_ChargingInfo;
};

struct ChargingNode : public Node
{
	std::vector<ChargingData> m_ChargingInfo;
};

