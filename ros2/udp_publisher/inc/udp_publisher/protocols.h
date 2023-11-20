#ifndef PROTOCOL_H
#define PROTOCOL_H

#pragma pack(push,1)

enum class e_CSMode : unsigned char
{ //режим работы
    MODE_MANUAL = 0, //ручной
    MODE_AUTOMATED, //автоматизированный
    MODE_AUTOMATIC, //автоматический
    MODE_GROUP //групповой
};

enum class power_Mode : unsigned char
{ //режим работы
    MODE_2 = 0, //включены вычислитель, wifi, uwb
    MODE_3, //включены вычислитель, wifi, uwb, гидроакустика
    MODE_4, //включены вычислитель, wifi, uwb, гидроакустика, ВМА
    MODE_5 //выключить вычислитель на 5 секунд и включить обратно
};


//структура данных, которая передается из Северова в Пульт
//тут описаны данные, которые Пульт принимает от Северова

struct ControlData
{ //данные, которые идут с пульта в СУ
    float yaw       = 0;
    float pitch     = 0;
    float roll      = 0;
    float march     = 0;
    float depth     = 0;
    float lag       = 0;
};

struct ControlVMA
{ //данные, которые идут на каждый ВМА
    float VMA1     = 0;
    float VMA2     = 0;
    float VMA3     = 0;
    float VMA4     = 0;
    float VMA5     = 0;
    float VMA6     = 0;
};

struct ControlContoursFlags
{ //флаги замыкания контуров (если 1, то замкнуты, 0 - разомкнуты)
    unsigned char yaw = 1;
    unsigned char pitch = 1;
    unsigned char roll = 1;
    unsigned char march = 1;
    unsigned char depth = 1;
    unsigned char lag = 1;
};


struct AUVCurrentData
{
    //!!тут все текущие параметры АНПА
    unsigned char modeReal = 0;//текущий режим
    ControlContoursFlags controlReal;//текущее состояние контуров, чтобы проверить что сигнал с пульта был обработан
    unsigned char modeAUV_Real = 0;//текущий выбор модель/реальный НПА
    ControlData ControlDataReal;//текущие курс, дифферент, крен, значения с имушки по сути
    ControlVMA signalVMA_real;//управление на ВМА (текущие управляющие сигнлы на движители)
};


struct HeaderSwap
{
    int senderID=0;
    int receiverID=0;
    int msgSize=0;
};

struct DataAH127C
{ //структура данных с датчика бесплатформенной системы ориентации
    float yaw       = 0; //курс градусы +/- 180
    float pitch     = 0; //...
    float roll      = 0;

    float X_accel   = 0;
    float Y_accel   = 0;
    float Z_accel   = 0;

    float X_rate    = 0;
    float Y_rate    = 0;
    float Z_rate    = 0;

    float X_magn    = 0;
    float Y_magn    = 0;
    float Z_magn    = 0;

    float quat [4];
};

struct FlagAH127C_bort
{
    unsigned char startCalibration = 0;
    unsigned char endCalibration = 0;
};

struct FlagAH127C_pult
{
    unsigned char initCalibration = 0;
    unsigned char saveCalibration = 0;
};

struct DataPressure
{ //структура данных с датчика давления
    float temperature = 0; //Temperature returned in deg C.
    float depth       = 0; //Depth returned in meters
    float pressure    = 0; // Pressure returned in mbar or mbar*conversion rate.
};

struct DataUWB
{ //структура данных с сверхширокополосного модуля
    float locationX = 0; //координата аппарата по оси X
    float locationY = 0; //координата аппарата по оси Y
    float distanceToBeacon[4]; //расстоние до i-го маяка
    float distanceToAgent[10]; //расстояние до i-го агента
};

struct FromBort
{
    FromBort(int auvID=0)
    {
        headerSwap.senderID = auvID;
        headerSwap.receiverID = 0;
        headerSwap.msgSize = sizeof (FromBort);
    }
    HeaderSwap headerSwap;
    AUVCurrentData auvData;// данные о текущих параметрах
    DataAH127C dataAH127C;// данные с БСО
    DataPressure dataPressure; //данные с датчика давления
    DataUWB dataUWB;//данные с UWB
    FlagAH127C_bort flagAH127C_bort;
    unsigned int checksum;

};

//структура данных, которая передается из пульта в АНПА
struct ToBort
{
    ControlData controlData; //данные, которые идут с пульта при замыканиии контуров
    e_CSMode cSMode = e_CSMode::MODE_MANUAL; //режим работы
    ControlContoursFlags controlContoursFlags; //флаги замыкания контуров (если больше 0, то замкнуты
    unsigned char modeAUV_selection;//текущий выбор модель/реальный НПА
    power_Mode pMode = power_Mode::MODE_2; //режим работы системы питания, структура с желаемыми параметрами системы питания
    FlagAH127C_pult flagAH127C_pult;
    unsigned int checksum;

};

#pragma pack (pop)


#endif // PROTOCOL_H
