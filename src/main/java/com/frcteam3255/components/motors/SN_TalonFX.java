package com.frcteam3255.components.motors;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.BaseTalonConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class SN_TalonFX extends TalonFX implements SN_MotorInterface {

	public SN_TalonFX(int deviceNumber) {
		super(deviceNumber);
		super.configFactoryDefault();
	}

	@Override
	public ErrorCode configAllSettings(BaseTalonConfiguration allConfigs) {
		return super.configAllSettings(allConfigs);
	}

}
