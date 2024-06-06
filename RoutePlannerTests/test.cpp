#include "pch.h"

#include "../RoutePlanner/Car.h"
#include "../RoutePlanner/RoutePlanner.h"
#include <chrono>

TEST(TestCaseName, TestName) {
    EXPECT_EQ(1, 1);
    EXPECT_TRUE(true);
}

TEST(Test1, RouteTest) {
    std::unique_ptr<RoutePlanner> planner = std::make_unique<RoutePlanner>();

    std::shared_ptr<std::vector<const Junction*>> resultJunctions = std::make_shared<std::vector<const Junction*>>();

    planner->Initialize();

    Car car = Car();
    car.m_Name = "peugeot_208";
    car.m_DragCoefficient = 0.28;
    car.m_ChargerStandard = ChargerType::ccs;
    car.m_MinChargeInPercent = 10;
    car.m_MaxChargeInPercent = 80;
    car.m_ChargeInPercent = 17;
    car.m_WeightInKg = 1500;
    car.m_BatteryCapacityInKWh = 46.3f;
    car.m_NEDCConsumptionOnOneMetreInPercent = 0.000244f; //100 km 24.4% energy
    car.m_ChargeSpeedDataInKW = Converter::LoadChargingSpeedData(car.m_Name);

    std::chrono::high_resolution_clock::time_point start(
        std::chrono::high_resolution_clock::now());

    //really short route
    //planner->FindFastestRoute(47.243446350097656f, 9.5248165130615234f, 47.244438171386719f, 9.5273571014404297f, car, resultJunctions);
    //planner->FindFastestRoute(47.06570898583726f, 9.496391087218626f, 47.2435417175293f, 9.524989128112793f, car, resultJunctions);
    planner->FindFastestRoute(47.2435417175293f, 9.524989128112793f, 47.06570898583726f, 9.496391087218626f, car, resultJunctions);
    //planner->FindFastestRoute(47.2435417175293, 9.524989128112793, 47.17247337653919, 9.533708386783616, car, resultJunctions);

    //std::cout << (std::chrono::high_resolution_clock::now() - start); //0.0658705 sec runtime

    Converter::SaveResultToGeoJson(resultJunctions, "../RoutePlannerClient/wwwroot/results/result.json");
}