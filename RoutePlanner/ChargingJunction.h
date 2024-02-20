#pragma once

#include "Junction.h"

enum ChargerType
{
	css, type2
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

