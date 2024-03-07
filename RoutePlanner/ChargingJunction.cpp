#include "ChargingJunction.h"

ChargingJunction::ChargingJunction(int64_t id, float_t lon, float_t lat) : Junction(id, lon, lat)
{
	m_ChargingInfo = std::vector<ChargingData>();
}
