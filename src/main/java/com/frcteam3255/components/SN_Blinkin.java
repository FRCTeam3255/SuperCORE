package com.frcteam3255.components;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

/**
 * Custom REV Blinkin framework making it easier to set colors and patters over
 * PWM
 * <p>
 * Use {@link #setPattern(PatternType)} to set the color of the LEDs.
 * <p>
 * See https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf for
 * information on how to wire, setup, and use the REV Blinkin
 */

public class SN_Blinkin extends Spark {

	/** Enum containing every REV Blinkin pattern */
	public enum PatternType {
		RainbowRainbowPalette(-0.99), RainbowPartyPalette(-0.97), RainbowOceanPalette(-0.95), RainbowLavePalette(
				-0.93), RainbowForestPalette(-0.91), RainbowWithGlitter(-0.89), Confetti(-0.87), ShotRed(
						-0.85), ShotBlue(-0.83), ShotWhite(-0.81), SinelonRainbowPalette(-0.79), SinelonPartyPalette(
								-0.77), SineloneOceanPalette(-0.75), SinelonLavaPalette(-0.73), SinelonForestPalette(
										-0.71), BPMRainbowPalette(-0.69), BPMPartyPalette(-0.67), BPMOceanPalette(
												-0.65), BPMLavaPalette(-0.63), BPMForestPalette(-0.61), FireMedium(
														-0.59), FireLarge(-0.57), TwinklesRainbowPalette(
																-0.55), TwinklesPartyPalette(
																		-0.53), TwinklesOceanPalette(
																				-0.51), TwinklesLavaPalette(
																						-0.49), TwinklesForestPalette(
																								-0.47), ColorWavesRainbowPalette(
																										-0.45), ColorWavesPartyPalette(
																												-0.43), ColorWavesOceanPalette(
																														-0.41), ColorWavesLavaPalette(
																																-0.39), ColorWavesForestPalette(
																																		-0.37), LarsonScannerRed(
																																				-0.35), LarsonScannerGray(
																																						-0.33), LightChaseRed(
																																								-0.31), LightCaseBlue(
																																										-0.29), LightChaseGray(
																																												-0.27), HeartbeatRed(
																																														-0.25), HeartbeatBlue(
																																																-0.23), HeartbeatWhite(
																																																		-0.21), HeartbeatGray(
																																																				-0.19), BreathRed(
																																																						-0.17), BreathBlue(
																																																								-0.15), BreathGray(
																																																										-0.13), StrobeRed(
																																																												-0.11), StrobeBlue(
																																																														-0.09), StrobeGold(
																																																																-0.07), StrobeWhite(
																																																																		-0.05),

		EndToEndBlendToBlackC1P(-0.03), LarsonScannerC1P(-0.01), LightChaseC1P(0.01), HeartbeatSlowC1P(
				0.03), HeartbeatMediumC1P(0.05), HeartbeatFastC1P(
						0.07), BreathSlowC1P(0.09), BreathFastC1P(0.11), ShotC1P(0.13), StrobeC1P(0.15),

		EndToEndBlendToBlackC2P(0.17), LarsonScannerC2P(0.19), LightChaseC2P(0.21), HeartbeatSlowC2P(
				0.23), HeartbeatMediumC2P(0.25), HeartbeatFastC2P(
						0.27), BreathSlowC2P(0.29), BreathFastC2P(0.31), ShotC2P(0.33), StrobeC2P(0.35),

		SparkleColor1onColor2(0.37), SparkleColor2onColor1(0.39), ColorGradient(0.41), BPMColor1and2(
				0.43), EndtoEndBlendColor1to2(0.45), EndtoEndBlend(0.47), Color1andColor2noblending(
						0.49), TwinklesColor1and2(0.51), ColorWavesColor1and2(0.53), SinelonColor1and2(0.55),

		HotPink(0.57), DarkRed(0.59), Red(0.61), RedOrange(0.63), Orange(0.65), Gold(0.67), Yellow(0.69), LawnGreen(
				0.71), Lime(0.73), DarkGreen(0.75), Green(0.77), BlueGreen(0.79), Aqua(0.81), SkyBlue(0.83), DarkBlue(
						0.85), Blue(0.87), BlueViolet(
								0.89), Violet(0.91), White(0.93), Gray(0.95), DarkGray(0.97), Black(0.99);

		public final double Value;

		private PatternType(double Value) {
			this.Value = Value;
		}
	}

	/**
	 * Create an SN_Blinkin
	 *
	 * @param PWMChannel
	 *            PWM Port that Blinkin is plugged into on the roboRIO
	 */
	public SN_Blinkin(int PWMChannel) {
		super(PWMChannel);
	}

	/**
	 * Set the 5 volt (addressable) LED pattern. Safe to use with 12 volt
	 * (non-addressable) LEDs. A 12V LED strip will show the patterns with the same
	 * color palette as the pattern selected
	 *
	 * @param pattern
	 *            Pattern to display
	 */
	public void setPattern(PatternType pattern) {
		this.set(pattern.Value);
	}
}
