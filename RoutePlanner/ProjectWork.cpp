
#include "RoutePlanner.h"
#include "Car.h"

#include "crow.h"
#include "crow/middlewares/cors.h"

#include <chrono>


int main()
{
    crow::App<crow::CORSHandler> app;

    // Middleware to handle CORS for all responses
    auto& cors = app.get_middleware<crow::CORSHandler>();

    cors.global()
        .origin("http://localhost:5083")  //frontend host
        .allow_credentials()
        .headers(
            "Accept",
            "Origin",
            "Content-Type",
            "Authorization",
            "Refresh"
        )
        .methods(
            crow::HTTPMethod::GET,
            crow::HTTPMethod::POST,
            crow::HTTPMethod::OPTIONS,
            crow::HTTPMethod::HEAD,
            crow::HTTPMethod::PUT
            /* crow::HTTPMethod::DELETE*/
        );

    //define your endpoint at the root directory
    CROW_ROUTE(app, "/search")
        .methods(crow::HTTPMethod::POST)
        ([](const crow::request req) {

            std::cout << "data received: " << req.body << std::endl;
            //return "Hello world";

            return crow::response("Success!");

    });

    CROW_ROUTE(app, "/getcars")
        .methods(crow::HTTPMethod::GET)
        ([](const crow::request req) {
        auto res = Converter::GetCarNames("../data/CarData");
        return res;
    });

    ////define your endpoint at the root directory
    //CROW_ROUTE(app, "/")([]() {
    //    return "Hello world";
    //});

    //set the port, set the app to run on multiple threads, and run the app
    app.port(18080).multithreaded().run();

   /* std::unique_ptr<Converter> converter = std::make_unique<Converter>();

    converter->ConvertOsmDataToJson("../data/RawMaps/liechtenstein-latest-srtm.osm", "../data/PreprocessedMaps/highwaydata.json");
    */
    //converter->ConvertOsmDataToJson("luxembourg-latest.osm", "highwaydata.json");

    //std::unique_ptr<RoutePlanner> planner = std::make_unique<RoutePlanner>();

    //std::shared_ptr<std::vector<const Junction*>> resultJunctions = std::make_shared<std::vector<const Junction*>>();

    //planner->Initialize("../data/PreprocessedMaps/highwaydata.json");

    //Car car = Car();
    //car.m_Name = "peugeot_208";
    //car.m_DragCoefficient = 0.28;
    //car.m_ChargerStandard = ChargerType::ccs;
    //car.m_MinChargeInPercent = 10;
    //car.m_MaxChargeInPercent = 80;
    //car.m_ChargeInPercent = 17;
    //car.m_WeightInKg = 1500;
    //car.m_BatteryCapacityInKWh = 46.3f;
    //car.m_NEDCConsumptionOnOneMetreInPercent = 0.000244f; //100 km 24.4% energy
    //car.m_ChargeSpeedDataInKW = Converter::LoadChargingSpeedData(car.m_Name);


    //std::chrono::high_resolution_clock::time_point start(
    //    std::chrono::high_resolution_clock::now());

    //really short route
    //planner->FindFastestRoute(47.243446350097656f, 9.5248165130615234f, 47.244438171386719f, 9.5273571014404297f, car, resultJunctions);
    //planner->FindFastestRoute(47.06570898583726f, 9.496391087218626f, 47.2435417175293f, 9.524989128112793f, car, resultJunctions);
 //   planner->FindFastestRoute(47.2435417175293f, 9.524989128112793f, 47.06570898583726f, 9.496391087218626f, car, resultJunctions);
    //planner->FindFastestRoute(47.2435417175293, 9.524989128112793, 47.17247337653919, 9.533708386783616, car, resultJunctions);

    //std::cout << (std::chrono::high_resolution_clock::now() - start); //0.0658705 sec runtime

    //Converter::SaveResultToGeoJson(resultJunctions, "../RoutePlannerClient/wwwroot/results/result.json");
    
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
