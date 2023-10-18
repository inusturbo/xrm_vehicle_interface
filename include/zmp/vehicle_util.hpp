#ifndef VEHCILE_UTIL_HPP
#define VEHCILE_UTIL_HPP

#include <zmp/vehicle_info.hpp>
#include <zmp/HevControl.h>
#include <iostream>
#include <time.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <cmath>

using namespace std;
using namespace zmp::minivan;

struct DrvInf
{
  int mode;           // ドライブモード(manual=0x00, program=0x10) 驱动模式，可以为 manual 或 program。
  int contMode;       // ドライブ制御モード(velocity=0x00, stroke=0x10) contMode：驱动控制模式，可以为 velocity 或 stroke。
  int overrideMode;   // オーバーライドモード(ON=0x00, OFF=0x10) 覆盖模式，可以为 ON 或 OFF。
  int servo;          // 制御のON/OFF(ON=true, OFF=false) 控制开关状态，true 表示开，false 表示关。
  int actualPedalStr; // ペダルストローク現在値 当前油门踏板行程值。 (0～4095)
  int targetPedalStr; // ペダルストローク目標値 油门踏板目标行程值。 (0～4095)
  int inputPedalStr;  // ペダルストローク入力値 油门踏板输入行程值。 (0～4095)
  float targetVeloc;  // 目標速度[km/h*100]
  float veloc;        // 現在速度[km/h*100]
  // 档位有 R: 倒车 N: 空挡 D: 前进 S: 运动模式 P: 停车模式
  int actualShift; // シフトポジション現在値 当前变速器档位。
  int targetShift; // シフトポジション目標値 变速器目标档位。
  int inputShift;  // シフトポジション入力値 变速器输入档位值。
};

struct BrakeInf
{
  bool pressed;               // ペダルスイッチ状態(ON=true, OFF=false) 制动踏板开关状态，true 表示按下，false 表示未按下。
  int actualPedalStr;         // ペダルストローク現在値 当前制动踏板行程值。(0～4095)
  int targetPedalStr;         // ペダルストローク目標値 制动踏板目标行程值。(0～4095)
  int inputPedalStr;          // ペダルストローク入力値 制动踏板输入行程值。(0～4095)
  unsigned char brakeLamp;    // 制动灯状态。
  unsigned char blinkerLeft;  // 左转灯状态。
  unsigned char blinkerRight; // 右转灯状态。
  unsigned char brakeMode;    // ブレーキランプをブレーキと連動し点灯します。
};

struct StrInf
{
  int mode;          // ステアリングモード(manual=0x00, program=0x10) 转向模式，可以为 manual 或 program。
  int cont_mode;     // ステアリング制御モード(torque=0x00, angle=0x10) 转向控制模式，可以为 torque 或 angle。
  int overrideMode;  // オーバーライドモード(ON=0x00, OFF=0x10) 覆盖模式，可以为 ON 或 OFF。
  int servo;         // 制御のON/OFF(ON=true, OFF=false) 控制开关状态，true 表示开，false 表示关。
  int targetTorque;  // 目標トルク 目标转向力矩。
  int torque;        // 操舵トルク 当前转向力矩。
  float angle;       // 操舵角度[deg * 10] 当前转向角度，单位为 deg*10。
  float targetAngle; // 目標操舵角[deg*10] 目标转向角度，单位为 deg*10。
};

struct OtherInf
{
  float sideAcc;                 // 0x22 Yawレート 横向加速度。
  float acc;                     // 0x23 前後加速度 纵向加速度。
  float angleFromP;              // 0x25 ステアリング角度 从电控系统中获取的方向盘转角，单位为度。
  float brkPedalStrFromP;        // 0x30 ブレーキペダル状態 从电控系统中获取的制动踏板状态。
  float velocFrFromP;            // 0xB1 右前輪速度[km/h*100]
  float velocFlFromP;            // 0xB1 左前輪速度[km/h*100]
  float velocRrFromP;            // 0xB3 右後輪速度[km/h*100]
  float velocRlFromP;            // 0xB3 左後輪速度[km/h*100]
  float velocFromP2;             // 0xB4 速度 从电控系统中获取的速度，单位为 km/h。
  int drv_mode;                  // 0x120 ドライブモード 驱动模式。
  unsigned int drvPedalStrFromP; // 0x244 アクセルペダル状態 从电控系统中获取的油门踏板状态。
  int rpm;                       // 0x3C8 エンジンの回転数 发动机转速。
  float velocFromP;              // 0x3CA 速度 从电控系统中获取的速度，单位为 km/h。
  int ev_mode;                   // 0x529 EVモード 电动模式。
  int temp;                      // 0x52C 温度[℃ ] 温度，单位为℃。
  int shiftFromPrius;            // 0x540 シフト状態 从电控系统中获取的变速器状态。
  LIGHT_STATE light;             // 0x57F ライト状態 灯光状态。
  int level;                     // 0x5A4 燃料の残量 剩余燃料量。
  DOOR_STATE door;               // 0x5B6 ドアの状態 车门状态。
  bool cluise;                   // 0x5C8 クルーズコントロールON/OFF 巡航控制开关状态。
  char dtcData1;                 // 0x7E8,0x7EA,0x7EB //故障码数据，用于故障诊断。
  char dtcData2;                 // 0x7E8,0x7EA,0x7EB
  char dtcData3;                 // 0x7E8,0x7EA,0x7EB
  char dtcData4;                 // 0x7E8,0x7EA,0x7EB
  char dtcData5;                 // 0x7E8,0x7EA,0x7EB
  char dtcData6;                 // 0x7E8,0x7EA,0x7EB
  char dtcData7;                 // 0x7E8,0x7EA,0x7EB
  char dtcData8;                 // 0x7E8,0x7EA,0x7EB
};

struct ConfigInf
{
  int data[21];
};

class VehicleUtil : public ChangeStateObserver
{
public:
  VehicleUtil();
  virtual ~VehicleUtil();
  HevControl *_hevCnt;
  CANUSBZ *_canCom;

  BrakeInf _brakeInf;
  OtherInf _otherInf;
  DrvInf _drvInf;
  StrInf _strInf;
  ConfigInf _config;

  char _firm_version[9];
  int _errCode;
  int _errLevel;

  int _asistTrq;

  void ClearCntDiag(); // 清除计数器对话框。
  bool Init();         // 初始化。
  bool Start();        // 启动。
  bool Close();        // 关闭。

  void GetDrvInf();                // 获取驱动信息。
  void GetStrInf();                // 获取转向信息。
  void GetBrakeInf();              // 获取刹车信息。
  void GetOtherInf();              // 获取其他车辆信息。
  int isDrvControled();            // 判断驱动是否控制。
  int isStrControled();            // 判断转向是否控制。
  int GetCurrentGear();            // 获取档位。
  float GetStrAngle();             // 获取转向角度。
  float GetStrRad();               // 获取转向角度。
  float GetDrvSpeedKmh();          // 获取车速。
  float GetDrvSpeedMps();          // 获取车速。
  unsigned char GetHazardLights(); // 获取危险灯。
  unsigned char GetBlinkerLeft();  // 获取左转灯。
  unsigned char GetBlinkerRight(); // 获取右转灯。

  void UpdateSteerState(REP_STEER_INFO_INDEX index);
  void UpdateDriveState(REP_DRIVE_INFO_INDEX index);
  void UpdateOtherState(REP_OTHER_INFO_INDEX index);
  void UpdateState();

  // Set Steer
  void SetStrMode(int mode);
  void SetStrCMode(int cmode);
  void SetStrOMOde(int omode);
  void SetStrTorque(int torque);
  void SetStrAngle(float angle);
  void SetStrServo(int servo);
  void SteeringControl(float cmd_steering_angle, float steering_tire_rotation_rate);

  // Set Drive
  void SetDrvMode(int mode);
  void SetDrvCMode(int cmode);
  void SetDrvOMode(int omode);
  void SetDrvStroke(int stroke);
  void SetDrvVeloc(float veloc);
  void SetDrvShiftMode(int shift);
  void SetDrvServo(int servo);
  void StopVehicle();
  void VelocityControl(float veloc_kmh, float acc);

  // Set Brake
  void SetBrakeStroke(int stroke);
  void SetBrakeLamp(unsigned char lamp);
  void SetBlinkerLeft(unsigned char blink_left);
  void SetBlinkerRight(unsigned char blink_right);
  void SetBrakeMode(unsigned char mode);
  void SetHazardLights(unsigned char lamp);

  // Set Other
  void SetControlGain(int index, int gain);
  void SetTargetAngle(int target);
  void SndDiagReq(HEV_ECU kind);
  void SndDiagClear(HEV_ECU kind);
  void SndErrReq();
  void SndErrClear();
  void SndVersionReq();

  // Control Function
  void SetDrvManual();
  void SetDrvProgram();
  void SetStrManual();
  void SetStrProgram();
  void SetManual();
  void SetProgram();

  float DegToRad(float deg);
};

#endif // VEHCILE_UTIL_HPP