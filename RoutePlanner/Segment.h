#pragma once

#include "Junction.h"

#include <cmath>
#include <vector>
#include <cstdint>
#include <memory>

class Segment
{

public:

	Segment(const int64_t id, const float_t length, Junction* from, Junction* to, uint8_t maxSpeed);
	~Segment();

	Junction* GetEndJunction(const Junction* startJunction);

	int64_t m_Id;

	float_t m_Length;

	Junction* m_From;

	Junction* m_To;

	uint8_t m_MaxSpeed;

	std::shared_ptr<std::vector<Junction*>> m_InnerNodes;

};

struct Way
{
	int64_t m_Id;

	float_t m_Length;

	bool m_OneWay;

	int64_t m_IdFrom;

	int64_t m_IdTo;

	uint8_t m_Maxspeed;

	std::shared_ptr<std::vector<int64_t>> m_InnerNodes = std::make_shared<std::vector<int64_t>>();
};

