#include <zmp/vehicle_util.hpp>

VehicleUtil::VehicleUtil()
{
    std::cout << "VehicleUtil::VehicleUtil()" << std::endl;
}

bool VehicleUtil::Init()
{
    std::cout << "VehicleUtil::Init()" << std::endl;
    _hevCnt = new HevControl();
    _canCom = new CANUSBZ();

    _hevCnt->InitHevControl(_canCom, (char *)_DEVICE_NAME);

    _canCom->SetCANUSBZParam(CAN_CHANNEL_0, CAN_SPEED_500, CANID_KIND_11);
    _canCom->SetCANUSBZParam(CAN_CHANNEL_1, CAN_SPEED_1000, CANID_KIND_11);

    _hevCnt->SetStatusCallback(this);
    ClearCntDiag(); // Autoware Extension
    return true;
}

void VehicleUtil::ClearCntDiag()
{
    std::cout << "VehicleUtil::ClearCntDiag()" << std::endl;
    CANMsg msg;
    msg.LEN = 1;
    msg.ID = 0x18 << 6 | MSG_COMMON_REQ_ERROR_STATUS;
    msg.DATA[0] = 1;
    _canCom->SendMsg(CAN_CHANNEL_1, msg);
}

bool VehicleUtil::Start()
{
    std::cout << "VehicleUtil::Start()" << std::endl;
    _canCom->Start();
    return true;
}

bool VehicleUtil::Close()
{
    std::cout << "VehicleUtil::Close()" << std::endl;
    _canCom->Close();
    return true;
}

void VehicleUtil::GetDrvInf()
{
    std::cout << "VehicleUtil::GetDrvInf()" << std::endl;
    _hevCnt->GetDrvMode((int &)_drvInf.mode);
    _hevCnt->GetDrvControlMode((int &)_drvInf.contMode);
    _hevCnt->GetDrvOverrideMode((int &)_drvInf.overrideMode);
    _hevCnt->GetDrvServo((int &)_drvInf.servo);
    _hevCnt->GetGasStroke((int &)_drvInf.actualPedalStr, (int &)_drvInf.targetPedalStr, (int &)_drvInf.inputPedalStr);
    _hevCnt->GetVeloc((float &)_drvInf.veloc, (float &)_drvInf.targetVeloc);
    _hevCnt->GetShiftMode((int &)_drvInf.actualShift, (int &)_drvInf.targetShift, (int &)_drvInf.inputShift);
}

void VehicleUtil::GetBrakeInf()
{
    std::cout << "VehicleUtil::GetBrakeInf()" << std::endl;
    _hevCnt->GetBrakeStatus((unsigned char &)_brakeInf.brakeLamp, (unsigned char &)_brakeInf.blinkerLeft, (unsigned char &)_brakeInf.blinkerRight, (unsigned char &)_brakeInf.brakeMode);
    _hevCnt->GetBrakeStrokeFromOBD((float &)_otherInf.brkPedalStrFromP, (bool &)_brakeInf.pressed);
    _hevCnt->GetBrakeStroke((int &)_brakeInf.actualPedalStr, (int &)_brakeInf.targetPedalStr, (int &)_brakeInf.inputPedalStr);
}

void VehicleUtil::GetStrInf()
{
    std::cout << "VehicleUtil::GetStrInf()" << std::endl;
    _hevCnt->GetStrMode((int &)_strInf.mode);
    _hevCnt->GetStrControlMode((int &)_strInf.cont_mode);
    _hevCnt->GetStrOverrideMode((int &)_strInf.overrideMode);
    _hevCnt->GetStrServo((int &)_strInf.servo);
    _hevCnt->GetStrTorque((int &)_strInf.torque);
    _hevCnt->GetStrTargetTorque((int &)_strInf.targetTorque);
    _hevCnt->GetStrAngle((float &)_strInf.angle);
    _hevCnt->GetStrTargetAngle((float &)_strInf.targetAngle);
}

void VehicleUtil::GetOtherInf()
{
    std::cout << "VehicleUtil::GetOtherInf()" << std::endl;
    _hevCnt->GetBrakeStrokeFromOBD((float &)_otherInf.brkPedalStrFromP, (bool &)_brakeInf.pressed);
    _hevCnt->GetGasStrokeFromOBD((int &)_otherInf.drvPedalStrFromP);
    _hevCnt->GetVelocFromOBD((float &)_otherInf.velocFromP);
    _hevCnt->GetVelocFromOBD2((float &)_otherInf.velocFromP2);
    _hevCnt->GetWheelVelocF((float &)_otherInf.velocFrFromP, (float &)_otherInf.velocFlFromP);
    _hevCnt->GetWheelVelocR((float &)_otherInf.velocRrFromP, (float &)_otherInf.velocRlFromP);
    _hevCnt->GetShiftModeFromOBD((int &)_otherInf.shiftFromPrius);
    _hevCnt->GetEvMode((int &)_otherInf.ev_mode);
    _hevCnt->GetIceRpm((int &)_otherInf.rpm);
    _hevCnt->GetIceCoolantTemp((int &)_otherInf.temp);
    _hevCnt->GetAcc((float &)_otherInf.acc);
    _hevCnt->GetSideAcc((float &)_otherInf.sideAcc);
    _hevCnt->GetDriveMode((int &)_otherInf.drv_mode);
    _hevCnt->GetCruiseControl((bool &)_otherInf.cluise);
    _hevCnt->GetLightState((LIGHT_STATE &)_otherInf.light);
    _hevCnt->GetGasLevel((int &)_otherInf.level);
    _hevCnt->GetDoorState((DOOR_STATE &)_otherInf.door);
}

void VehicleUtil::UpdateSteerState(REP_STEER_INFO_INDEX index)
{
    std::cout << "VehicleUtil::UpdateSteerState()" << std::endl;
    switch (index)
    {
    case REP_STR_MODE:
        _hevCnt->GetStrMode((int &)_strInf.mode);
        _hevCnt->GetStrControlMode((int &)_strInf.cont_mode);
        _hevCnt->GetStrOverrideMode((int &)_strInf.overrideMode);
        _hevCnt->GetStrServo((int &)_strInf.servo);
        break;
    case REP_TORQUE:
        _hevCnt->GetStrTorque((int &)_strInf.torque);
        _hevCnt->GetStrTargetTorque((int &)_strInf.targetTorque);
        //_hevCnt->SetStrTorque(_strInf.targetTorque + _asistTrq);
        break;
    case REP_ANGLE:
        _hevCnt->GetStrAngle((float &)_strInf.angle, (float &)_strInf.targetAngle);
        break;
    case REP_ANGLE_FROMOBD:
        _hevCnt->GetStrAngleFromOBD((float &)_otherInf.angleFromP);
        break;
    default:
        printf("\n");
        break;
    }
}

void VehicleUtil::UpdateDriveState(REP_DRIVE_INFO_INDEX index)
{
    std::cout << "VehicleUtil::UpdateDriveState()" << std::endl;
    switch (index)
    {
    case REP_DRV_MODE:
        _hevCnt->GetDrvMode((int &)_drvInf.mode);
        _hevCnt->GetDrvControlMode((int &)_drvInf.contMode);
        _hevCnt->GetDrvOverrideMode((int &)_drvInf.overrideMode);
        _hevCnt->GetDrvServo((int &)_drvInf.servo);
    case REP_GAS_PEDAL:
        _hevCnt->GetGasStroke((int &)_drvInf.actualPedalStr, (int &)_drvInf.targetPedalStr, (int &)_drvInf.inputPedalStr);
        break;
    case REP_GAS_PEDAL_FROMOBD:
        _hevCnt->GetGasStrokeFromOBD((int &)_otherInf.drvPedalStrFromP);
        break;
    case REP_VELOCITY:
        _hevCnt->GetVeloc((float &)_drvInf.veloc, (float &)_drvInf.targetVeloc);
        break;
    case REP_VELOCITY_FROMOBD:
        _hevCnt->GetVelocFromOBD((float &)_otherInf.velocFromP);
        break;
    case REP_VELOCITY_FROMOBD2:
        _hevCnt->GetVelocFromOBD2((float &)_otherInf.velocFromP2);
        break;
    case REP_WHEEL_VELOCITY_F:
        _hevCnt->GetWheelVelocF((float &)_otherInf.velocFrFromP, (float &)_otherInf.velocFlFromP);
        break;
    case REP_WHEEL_VELOCITY_R:
        _hevCnt->GetWheelVelocR((float &)_otherInf.velocRrFromP, (float &)_otherInf.velocRlFromP);
        break;
    case REP_BRAKE_PEDAL:
        _hevCnt->GetBrakeStroke((int &)_brakeInf.actualPedalStr, (int &)_brakeInf.targetPedalStr,
                                (int &)_brakeInf.inputPedalStr);
        break;
    case REP_BRAKE_PEDAL_FROMOBD:
        _hevCnt->GetBrakeStrokeFromOBD((float &)_otherInf.brkPedalStrFromP, (bool &)_brakeInf.pressed);
        break;
    case REP_SHIFT_POS:
        _hevCnt->GetShiftMode((int &)_drvInf.actualShift, (int &)_drvInf.targetShift, (int &)_drvInf.inputShift);
        break;
    case REP_SHIFT_POS_FROMOBD:
        _hevCnt->GetShiftModeFromOBD((int &)_otherInf.shiftFromPrius);
        break;
    case REP_HEV_MODE:
        _hevCnt->GetEvMode((int &)_otherInf.ev_mode);
        break;
    case REP_ICE_RPM:
        _hevCnt->GetIceRpm((int &)_otherInf.rpm);
        break;
    case REP_ICE_COOLANT_TEMP:
        _hevCnt->GetIceCoolantTemp((int &)_otherInf.temp);
        break;
    case REP_ACCELERLATION:
        _hevCnt->GetAcc((float &)_otherInf.acc);
        break;
    case REP_SIDE_ACCELERLATION:
        _hevCnt->GetSideAcc((float &)_otherInf.sideAcc);
        break;
    case REP_DRIVE_MODE:
        _hevCnt->GetDriveMode((int &)_otherInf.drv_mode);
        break;
    case REP_CRUISE_STATE:
        _hevCnt->GetCruiseControl((bool &)_otherInf.cluise);
        break;
    case REP_DTC_STATUS:
        break;
    case REP_BRAKE_STATUS:
        _hevCnt->GetBrakeStatus((unsigned char &)_brakeInf.brakeLamp, (unsigned char &)_brakeInf.blinkerLeft,
                                (unsigned char &)_brakeInf.blinkerRight, (unsigned char &)_brakeInf.brakeMode);
        break;
    default:
        printf("\n");
        break;
    }
    return;
}

void VehicleUtil::UpdateOtherState(REP_OTHER_INFO_INDEX index)
{
    std::cout << "VehicleUtil::UpdateOtherState()" << std::endl;
    switch (index)
    {
    case REP_LIGHT_STATE:
        _hevCnt->GetLightState((LIGHT_STATE &)_otherInf.light);
        break;
    case REP_GAS_LEVEL:
        _hevCnt->GetGasLevel((int &)_otherInf.level);
        break;
    case REP_DOOR_STATE:
        _hevCnt->GetDoorState((DOOR_STATE &)_otherInf.door);
        break;
    default:
        printf("\n");
        break;
    }

    return;
}

void VehicleUtil::ReceiveConfig(int num, int index, int value[])
{
    std::cout << "VehicleUtil::ReceiveConfig()" << std::endl;
    printf("ReceiveConfig() num=%d index=%d value=%d\n", num, index, value[index]);
    int data[3];
    for (int i = 0; i < num; i++)
    {
        _config.data[index - 100] = value[i];
        data[i] = value[i];
    }
    if (NULL != _callback)
    {
        _callback->UpdateConfig(num, index, data);
    }
}

void VehicleUtil::ReceiveErrorStatus(int level, int error_code)
{
    std::cout << "VehicleUtil::ReceiveErrorStatus()" << std::endl;
    printf("ReceiveErrorStatus() level=%d errCode=%d\n", level, error_code);
    _errCode = error_code;
    _errLevel = level;
}

void VehicleUtil::ReceiveEcho(int kind, int no)
{
    std::cout << "VehicleUtil::ReceiveEcho()" << std::endl;
    printf("ReceiveEcho() kind=%d no=%d\n", kind, no);
}

void VehicleUtil::ReceiveVersion(char c0, char c1, char c2, char c3, char c4, char c5, char c6, char c7)
{
    std::cout << "VehicleUtil::ReceiveVersion()" << std::endl;
    sprintf(_firm_version, "%c%c%c%c%c%c%c%c", c0, c1, c2, c3, c4, c5, c6, c7);
}