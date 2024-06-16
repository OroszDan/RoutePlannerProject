#include "ChargingJunction.h"

ChargingJunction::ChargingJunction(int64_t id, float_t lon, float_t lat, float_t elevation, const std::vector<ChargingData>& datas) 
	: Junction(id, lon, lat, elevation)
{
	m_ChargingInfo = datas;
}
