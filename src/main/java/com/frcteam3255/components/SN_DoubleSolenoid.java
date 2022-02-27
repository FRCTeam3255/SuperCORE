package com.frcteam3255.components;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class SN_DoubleSolenoid extends DoubleSolenoid {

	Value forwardValue;
	Value reverseValue;

	
	/**
	 * Wrapper for the WPILib DoubleSolenoid
	 * <p>
	 * Adds custom methods like {@link #setDeployed()} and {@link #getDeployed()}
	 *
	 * @param a_moduleType
	 *            CTRE Pneumatics Control Module or REV Pneumatics Hub
	 * @param a_forwardChannel
	 *            The forward channel of the solenoid on the Pneumatics Module
	 * @param a_reverseChannel
	 *            The reverse channel of the solenoid on the Pneumatics Module
	 */
	public SN_DoubleSolenoid(PneumaticsModuleType a_moduleType, int a_forwardChannel, int a_reverseChannel) {
		super(a_moduleType, a_forwardChannel, a_reverseChannel);
		forwardValue = Value.kForward;
		reverseValue = Value.kReverse;
	}

	private Value getValue() {
		return this.get();
	}

	/**
	 * @return true if the solenoid is deployed, false if the solenoid is retracted
	 */
	public boolean isDeployed() {
		return getValue() == forwardValue;
	}

	/**
	 * @return true if the solenoid is retracted, false if the solenoid is deployed
	 */
	public boolean isRetracted() {
		return !isDeployed();
	}

	private void rawSetDeployed() {
		this.set(forwardValue);
	}

	private void rawSetReverse() {
		this.set(reverseValue);
	}

	/**
	 * Deploys the solenoid
	 */
	public void setDeployed() {
		if (!isDeployed()) {
			rawSetDeployed();
		}
	}

	/**
	 * Retracts the solenoid
	 */
	public void setRetracted() {
		if (isDeployed()) {
			rawSetReverse();
		}
	}

	/**
	 * Deploys the solenoid if it's retracted
	 * <p>
	 * Retracts the solenoid if it's deployed
	 */
	public void toggle() {
		if (!isDeployed()) {
			setDeployed();
		} else {
			setRetracted();
		}
	}

	public void invert() {
		if (forwardValue == Value.kForward) {
			forwardValue = Value.kReverse;
			reverseValue = Value.kForward;
		} else {
			forwardValue = Value.kForward;
			reverseValue = Value.kReverse;
		}
	}
}
