#include "ChargingJunction.h"

ChargingJunction::ChargingJunction(int64_t id, float_t lon, float_t lat, const std::vector<ChargingData>& datas) : Junction(id, lon, lat)
{
	m_ChargingInfo = datas;
}
