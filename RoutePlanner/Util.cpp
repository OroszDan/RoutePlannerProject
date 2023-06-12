#include "Util.h"

#include <numbers>

float_t Util::CalculateDistanceBetweenTwoLatLonsInMetres(const float_t lat1, const float_t lat2, const float_t lon1, const float_t lon2)
{

	const float_t radLat1 = lat1 * std::numbers::pi / 180;
	const float_t radLat2 = lat2 * std::numbers::pi / 180;

	const float_t dLat = (lat2 - lat1) * std::numbers::pi / 180;
	const float_t dLon = (lon2 - lon1) * std::numbers::pi / 180;

	const float_t a = std::sinf(dLat / 2) * std::sinf(dLat / 2) +
		std::cosf(radLat1) * std::cosf(radLat2) *
		std::sinf(dLon / 2) * std::sinf(dLon / 2);

	const float_t c = 2 * std::atan2f(std::sqrtf(a), std::sqrtf(1 - a));

	const float_t d = earthRadius * c;

	return d;
}
