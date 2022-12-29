package com.frcteam3255.components.motors;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.BaseTalonConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class SN_CIM extends TalonSRX implements SN_MotorInterface {

    public SN_CIM(int deviceNumber) {
        super(deviceNumber);
        super.configFactoryDefault();
    }

    @Override
    public ErrorCode configAllSettings(BaseTalonConfiguration allConfigs) {
        return super.configAllSettings(allConfigs);
    }

    
}
