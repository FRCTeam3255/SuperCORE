package com.frcteam3255.components.motors;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalonConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class SN_SparkMax extends CANSparkMax implements SN_MotorInterface{

    private SparkMaxPIDController pidController;
    public RelativeEncoder encoder;
    private ControlMode pidType = ControlMode.Disabled;
    private double pidSetpoint;

    /**
     * Creates a SparkMax object for use with a brushless motor.
     */
    public SN_SparkMax(int deviceId) {
        this(deviceId, MotorType.kBrushless);
    }

    /**
     * Creates a SparkMax object with the defined motorType
     * @param motorType
	 *            Motor type to use. (Brushed or Brushless)
     */
    public SN_SparkMax(int deviceId, MotorType motorType) {
        super(deviceId, motorType);
        pidController = super.getPIDController();
        encoder = super.getEncoder();
        super.restoreFactoryDefaults();
        super.burnFlash();
    }

    @Override
    public void set(ControlMode mode, double value) {
        pidSetpoint = value;
        pidType = mode;
        switch (mode) {
            case PercentOutput:
                super.set(value);
                break;

            case Velocity:
                pidController.setReference(value, CANSparkMax.ControlType.kVelocity);
                break;
                
            case Position:
                pidController.setReference(value, CANSparkMax.ControlType.kPosition);
                break;

            default:
                break;
       }
        
    }

    @Override
    public void set(ControlMode mode, double demand0, DemandType demand1Type, double demand1) {
        // TODO: Complete
    }

    @Override
    public void setNeutralMode(NeutralMode neutralMode) {
        switch (neutralMode) {
            case Brake:
                super.setIdleMode(IdleMode.kBrake);
                break;
            case Coast:
                super.setIdleMode(IdleMode.kCoast);
                break;
        
            default:
                break;
        }
    }

    
    @Override
    public ErrorCode configAllSettings(BaseTalonConfiguration allConfigs) {
        // Set slot0 PID
        pidController.setP(allConfigs.slot0.kP,0);
        pidController.setD(allConfigs.slot0.kD,0);
        pidController.setI(allConfigs.slot0.kI,0);
        pidController.setFF(allConfigs.slot0.kF,0);
        pidController.setIZone(allConfigs.slot0.integralZone, 0);
        pidController.setIMaxAccum(allConfigs.slot0.maxIntegralAccumulator, 0);
        pidController.setOutputRange(-allConfigs.slot0.closedLoopPeakOutput, allConfigs.slot0.closedLoopPeakOutput, 0);
        pidController.setSmartMotionAllowedClosedLoopError(allConfigs.slot0.allowableClosedloopError, 0);
        
        // Set slot1 PID
        pidController.setP(allConfigs.slot1.kP,1);
        pidController.setD(allConfigs.slot1.kD,1);
        pidController.setI(allConfigs.slot1.kI,1);
        pidController.setFF(allConfigs.slot1.kF,1);
        pidController.setIZone(allConfigs.slot1.integralZone, 1);
        pidController.setIMaxAccum(allConfigs.slot1.maxIntegralAccumulator, 1);
        pidController.setOutputRange(-allConfigs.slot1.closedLoopPeakOutput, allConfigs.slot1.closedLoopPeakOutput, 1);
        pidController.setSmartMotionAllowedClosedLoopError(allConfigs.slot1.allowableClosedloopError, 1);
        
        // Set slot3 PID
        pidController.setP(allConfigs.slot3.kP,3);
        pidController.setD(allConfigs.slot3.kD,3);
        pidController.setI(allConfigs.slot3.kI,3);
        pidController.setFF(allConfigs.slot3.kF,3);
        pidController.setIZone(allConfigs.slot3.integralZone, 3);
        pidController.setIMaxAccum(allConfigs.slot3.maxIntegralAccumulator, 3);
        pidController.setOutputRange(-allConfigs.slot3.closedLoopPeakOutput, allConfigs.slot3.closedLoopPeakOutput, 3);
        pidController.setSmartMotionAllowedClosedLoopError(allConfigs.slot3.allowableClosedloopError, 3);
        
        // Config softlimits
        super.enableSoftLimit(SoftLimitDirection.kForward, allConfigs.forwardSoftLimitEnable);
        super.enableSoftLimit(SoftLimitDirection.kReverse, allConfigs.reverseSoftLimitEnable);
        super.setSoftLimit(SoftLimitDirection.kForward, (float)allConfigs.forwardSoftLimitThreshold);
        super.setSoftLimit(SoftLimitDirection.kReverse, (float)allConfigs.reverseSoftLimitThreshold);

        super.burnFlash();
        return null;
    }
    

    /**
     * Get the position of the motor. This returns the native units of 'rotations' by default, and can
     * be changed by a scale factor using setPositionConversionFactor().
     *
     * @return Number of rotations of the motor
     */
	@Override
	public double getSelectedSensorPosition() {
		return encoder.getPosition();
	}
    
    /**
     * Get the velocity of the motor. This returns the native units of 'RPM' by default, and can be
     * changed by a scale factor using setVelocityConversionFactor().
     *
     * @return Number the RPM of the motor
     */
	@Override
	public double getSelectedSensorVelocity() {
        return encoder.getVelocity();
	}

    /**
     * Common interface for getting the current set speed of a speed controller.
     *
     * @return The current set speed. Value is between -1.0 and 1.0.
     */
	@Override
	public double getMotorOutputPercent() {
		return super.getAppliedOutput();
	}

    @Override
    public ErrorCode configFactoryDefault() {
        super.restoreFactoryDefaults();
        return null;
    }

    @Override
    public ErrorCode setSelectedSensorPosition(double sensorPos) {
        encoder.setPosition(sensorPos);
        return null;
    }

    @Override
    public void neutralOutput() {
        super.stopMotor();
    }

    @Override
    public double getClosedLoopError() {
        switch (pidType) {
            case Velocity:
                return pidSetpoint - getSelectedSensorVelocity();
                
            case Position:
                return pidSetpoint - getSelectedSensorPosition();
            default:
                return 0;
        }
    }

    
}
