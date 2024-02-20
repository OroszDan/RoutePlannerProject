#pragma once

#include "Junction.h"

enum ChargerType
{
	undefined, css, type2, chademo
};

class ChargingJunction : public Junction 
{
public:

	uint16_t m_Output;

	ChargerType m_Type;

};

struct ChargingNode : public Node
{
	uint16_t m_Output;

	ChargerType m_Type;
};

