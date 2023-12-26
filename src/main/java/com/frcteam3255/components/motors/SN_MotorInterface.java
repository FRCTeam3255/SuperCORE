package com.frcteam3255.components.motors;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalonConfiguration;

/**
 * SN_MotorInterface
 */
public interface SN_MotorInterface {

	public void set(ControlMode mode, double value);
	public void set(ControlMode mode, double demand0, DemandType demand1Type, double demand1);
	public void setInverted(boolean invert);
	public void setNeutralMode(NeutralMode neutralMode);
	public ErrorCode configAllSettings(BaseTalonConfiguration allConfigs);
	public double getSelectedSensorPosition();
	public double getSelectedSensorVelocity();
	public double getMotorOutputPercent();
	public ErrorCode configFactoryDefault();
	public ErrorCode setSelectedSensorPosition(double sensorPos);
	public void neutralOutput();
	public double getClosedLoopError();

}
