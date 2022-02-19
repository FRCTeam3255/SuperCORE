package com.frcteam3255.preferences;

/**
 * SuperNURDs double zero preference base class
 */
public class SN_ZeroIntPreference extends SN_IntPreference{

    /**
     * Creates a preference with name "doubleZeroPreference" and value 0
     */
    public SN_ZeroIntPreference() {
        super("intZeroPreference", 0);
    }

    /**
     * @return 0, type int
     */
    @Override
    public int getValue() {
        return 0;
    }
}
