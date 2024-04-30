#pragma once

#include <cstdint>
#include <math.h>

#include "ChargerType.h"

struct Car
{
	ChargerType m_ChargerStandard;
	uint16_t m_MaxChargeInKW;
	float_t m_ChargeInPercent;
	float_t m_MinChargeInPercent;
	float_t m_MaxChargeInPercent;
	float_t m_DragCoefficient;
	int16_t m_WeightInKg;
	float_t m_NEDCConsumptionOnOneMetreInPercent;
	std::shared_ptr<const std::vector<int16_t>> m_ChargeSpeedData;

};

