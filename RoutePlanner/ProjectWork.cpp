
#include "RoutePlanner.h"
#include "Car.h"

#include <chrono>

#include <curl/curl.h>

static size_t my_write(char* buffer, size_t size, size_t nmemb, void* param) 
{
    std::string& text = *static_cast<std::string*>(param);
    std::string resultJson(buffer, nmemb);
    size_t totalsize = size * nmemb;
    return totalsize;
}

int main()
{

    /*CURL* curl;
    CURLcode res;
    std::string result;

    curl_global_init(CURL_GLOBAL_DEFAULT);

    curl = curl_easy_init();

    if (curl)
    {
        curl_easy_setopt(curl, CURLOPT_URL, "https://api.open-elevation.com/api/v1/lookup?locations=41.161758,-8.583933|10,10|20,20");
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, my_write);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA);

        res = curl_easy_perform(curl);        
    }

    curl_global_cleanup();
    return 0;*/

    std::unique_ptr<Converter> converter = std::make_unique<Converter>();

    converter->ConvertOsmDataToJson("../data/RawMaps/liechtenstein-latest-srtm.osm", "highwaydata.json");
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
