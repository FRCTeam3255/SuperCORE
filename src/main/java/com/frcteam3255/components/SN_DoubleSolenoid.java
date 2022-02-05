package com.frcteam3255.components;

import com.frcteam3255.preferences.SN_BooleanPreference;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class SN_DoubleSolenoid extends DoubleSolenoid {

    DoubleSolenoid solenoid;
    boolean inverted;

    Value forwardValue = Value.kForward;
    Value reverseValue = Value.kReverse;


    private Value getValue() {
        if (!inverted) {
            return solenoid.get();
        } else {
            if (solenoid.get() == forwardValue) {
                return reverseValue;
            } else {
                return forwardValue;
            }
        }
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
        if (!inverted) {
            solenoid.set(forwardValue);
        } else {
            solenoid.set(reverseValue);
        }
    }

    private void rawSetReverse() {
        if (inverted) {
            solenoid.set(reverseValue);
        } else {
            solenoid.set(forwardValue);
        }
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

    private boolean getBoolPrefValue(SN_BooleanPreference a_pref) {
        return a_pref.getValue();
    }


    /**
     * Wrapper for the WPILib DoubleSolenoid
     * <p>
     * Adds custom methods like {@link #setDeployed()} and {@link #getDeployed()}
     * @param a_moduleType
     *   CTRE Pneumatics Control Module or REV Pneumatics Hub
     * @param a_forwardChannel
     *   The forward channel of the solenoid on the Pneumatics Module
     * @param a_reverseChannel
     *   The reverse channel of the solenoid on the Pneumatics Module
     */
    public SN_DoubleSolenoid(PneumaticsModuleType a_moduleType, int a_forwardChannel, int a_reverseChannel) {
        super(a_moduleType, a_forwardChannel, a_reverseChannel);
        inverted = false;
    }


    /**
     * Wrapper for the WPILib DoubleSolenoid
     * <p>
     * Adds custom methods like {@link #setDeployed()} and {@link #getDeployed()}
     * @param a_moduleType
     *   CTRE Pneumatics Control Module or REV Pneumatics Hub
     * @param a_forwardChannel
     *   The forward channel of the solenoid on the Pneumatics Module
     * @param a_reverseChannel
     *   The reverse channel of the solenoid on the Pneumatics Module
     * @param a_inverted
     *   Inverts the direction of the Solenoid (equivalent to physically swapping the forward and reverse channels)
     */
    public SN_DoubleSolenoid(PneumaticsModuleType a_moduleType, int a_forwardChannel, int a_reverseChannel, boolean a_inverted) {
        super(a_moduleType, a_forwardChannel, a_reverseChannel);
        inverted = a_inverted;
    }

    /**
     * Wrapper for the WPILib DoubleSolenoid
     * <p>
     * Adds custom methods like {@link #setDeployed()} and {@link #getDeployed()}
     * @param a_moduleType
     *   CTRE Pneumatics Control Module or REV Pneumatics Hub
     * @param a_forwardChannel
     *   The forward channel of the solenoid on the Pneumatics Module
     * @param a_reverseChannel
     *   The reverse channel of the solenoid on the Pneumatics Module
     * @param a_inverted
     *   Inverts the direction of the Solenoid (equivalent to physically swapping the forward and reverse channels)
     */
    public SN_DoubleSolenoid(PneumaticsModuleType a_moduleType, int a_forwardChannel, int a_reverseChannel, SN_BooleanPreference a_inverted) {
        super(a_moduleType, a_forwardChannel, a_reverseChannel);
        inverted = getBoolPrefValue(a_inverted);
    }
}