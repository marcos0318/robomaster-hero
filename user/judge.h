#ifndef JUDGE_H
#define JUDGE_H

#include "stm32f4xx.h"

#define JUDGE_BUFFER_LENGTH           256
#define JUDGE_INFO_FRAME_LENGTH       ((uint8_t)44)
#define JUDGE_BLOOD_FRAME_LENGTH      ((uint8_t)12)
#define JUDGE_SHOOT_FRAME_LENGTH      ((uint8_t)25)
#define JUDGE_FRAME_HEADER_LENGTH     ((uint8_t)5)
#define JUDGE_EXTRA_LENGTH            ((uint8_t)9)

#define JUDGE_FRAME_HEADER            0xA5

#ifndef JUDGE_FILE
    #define JUDGE_EXT extern
#else
    #define JUDGE_EXT
#endif

#define UNUSED(x) ((void)(x))

JUDGE_EXT volatile uint32_t JUDGE_FrameCounter;
JUDGE_EXT volatile uint8_t JUDGE_Started, JUDGE_RemainByte, JUDGE_NextDecodeOffset;

void judging_system_init(void);
void Judge_InitConfig(void);

void JUDGE_Decode(uint32_t length);
void JUDGE_DecodeFrame(uint8_t type);
uint8_t GetCRC8(uint8_t idx, uint8_t len, uint8_t ucCRC8);
unsigned int VerifyCRC8(uint8_t idx, uint8_t len);
extern u32 counter;

typedef struct {
  uint8_t flag;
  float x;
  float y;
  float z;
  float compass;
} LocatorData_t;

typedef struct {
  //0x0001
    uint32_t RemainTime;
    int16_t LastBlood;
    float RealVoltage;
    float RealCurrent;
    //LocatorData_t Locator;
    float RemainBuffer;

  //0x0002
    uint8_t LastHartID;
    int16_t ArmorDecrease;
    int16_t BulletDecrease;
    int16_t GolfDecrease;
    int16_t CrashDecrease;
    int16_t OverShootSpeedDecrease;
    int16_t OverShootFreqDecrease;
    int16_t OverPowerDecrease;
    int16_t ModuleOfflineDecrease;
    int16_t FaultDecrease;
    int16_t AirportIncrease;
    int16_t AutoIncrease; // Engineer
    int16_t CardIncrease;

  //0x0003
    float LastShotSpeed;
    float LastShotFreq;
    float GolfShotSpeed;
    float GolfShotFreq;

  //0x0004
    uint8_t MySide; // 0: red 1: blue
    // all index follows: 0-red 1-blue
    uint8_t BaseState[2]; // 0: normal 1: invincible
    uint8_t HeroOnStage[2]; // 0/1: not/is on stage
    uint8_t AirportState[2]; /*
                                0: invalid
                                1: activatable
                                2: activating
                                3: activated
                                4: cooling
                              */
    uint8_t StandState[6]; /*
                              0: invalid
                              1: activatable
                              2: red occupying
                              3: blue occupying
                              4: red occupied
                              5: blue occupied
                            */
    uint8_t SupplyState[2]; // is adding or not
    uint16_t AddBulletCount[2];
    uint8_t RuneState[2]; /*
                              0: invalid
                              1: activatable
                              2: red occupying
                              3: blue occupying
                              4: red occupied
                              5: blue occupied
                            */
    uint8_t AddDefendPercent;
 
    uint8_t Updated;

}InfantryJudge_Struct;

// format transformation union
typedef union
{
    uint8_t U[4];
    float F;
    int I;
}FormatTrans;

//the data will be stored in the following struct
extern InfantryJudge_Struct InfantryJudge;

#endif
