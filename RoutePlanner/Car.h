#pragma once

#include <cstdint>
#include <math.h>
#include <string>

#include "ChargerType.h"

struct Car
{
	std::string m_Name;
	ChargerType m_ChargerStandard;
	float_t m_ChargeInPercent;
	float_t m_MinChargeInPercent;
	float_t m_MaxChargeInPercent;
	float_t m_DragCoefficient;
	int16_t m_WeightInKg;
	float_t m_BatteryCapacityInKWh;
	float_t m_NEDCConsumptionOnOneMetreInPercent;
	std::shared_ptr<std::vector<int16_t>> m_ChargeSpeedDataInKW;

};

