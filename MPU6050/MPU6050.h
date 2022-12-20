#define _MPU_6050_
#ifdef _MPU_6050_
#include <tuple>

class MPU6050
{
    public:
        void update();
        std::tuple<float, float, float, float, float> calculateError();
        std::tuple<float, float, float> readAcc();
        std::tuple<float, float, float> readGyro();
        void initialize();
        // restituisce 0 se l'MPU può comunicare
        uint8_t testConnection();
};

#endif
