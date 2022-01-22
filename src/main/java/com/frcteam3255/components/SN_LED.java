package com.frcteam3255.components;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

/**
 * Custom REV Blinkin framework making it easier to set colors and patters over
 * PWM
 * <p>
 * Use {@link #setBlinkin(BlinkinPatterns)} with the {@link BlinkinPatterns}
 * enum which contains every available color and pattern for the REV Blinkin
 */

public class SN_LED extends Spark {

	/**
	 * The REV Blinkin
	 *
	 * @param channel
	 *            The PWM port the Blinkin is plugged into
	 */
	public SN_LED(int channel) {
		super(channel);
	}

	private enum BlinkinFixedPalettePatterns {
		RainbowRainbowPalette, RainbowPartyPalette, RainbowOceanPalette, RainbowLavePalette, RainbowForestPalette, RainbowWithGlitter, Confetti, ShotRed, ShotBlue, ShotWhite, SinelonRainbowPalette, SinelonPartyPalette, SineloneOceanPalette, SinelonLavaPalette, SinelonForestPalette, BPMRainbowPalette, BPMPartyPalette, BPMOceanPalette, BPMLavaPalette, BPMForestPalette, FireMedium, FireLarge, TwinklesRainbowPalette, TwinklesPartyPalette, TwinklesOceanPalette, TwinklesLavaPalette, TwinklesForestPalette, ColorWavesRainbowPalette, ColorWavesPartyPalette, ColorWavesOceanPalette, ColorWavesLavaPalette, ColorWavesForestPalette, LarsonScannerRed, LarsonScannerGray, LightChaseRed, LightCaseBlue, LightChaseGray, HeartbeatRed, HeartbeatBlue, HeartbeatWhite, HeartbeatGray, BreathRed, BreathBlue, BreathGray, StrobeRed, StrobeBlue, StrobeGold, StrobeWhite
	}

	private enum BlinkinColor1Patterns {
		EndToEndBlendToBlackC1P, LarsonScannerC1P, LightChaseC1P, HeartbeatSlowC1P, HeartbeatMediumC1P, HeartbeatFastC1P, BreathSlowC1P, BreathFastC1P, ShotC1P, StrobeC1P
	}

	private enum BlinkinColor2Patterns {
		EndToEndBlendToBlackC2P, LarsonScannerC2P, LightChaseC2P, HeartbeatSlowC2P, HeartbeatMediumC2P, HeartbeatFastC2P, BreathSlowC2P, BreathFastC2P, ShotC2P, StrobeC2P
	}

	private enum BlinkinColor1and2Pattern {
		SparkleColor1onColor2, SparkleColor2onColor1, ColorGradient, Color1and2, BPMColor1and2, EndtoEndBlendColor1to2, EndtoEndBlend, Color1andColor2noblending, TwinklesColor1and2, ColorWavesColor1and2, SinelonColor1and2
	}

	private enum BlinkinSolidColors {
		HotPink, DarkRed, Red, RedOrange, Orange, Gold, Yellow, LawnGreen, Lime, DarkGreen, Green, BlueGreen, Aqua, SkyBlue, DarkBlue, Blue, BlueViolet, Violet, White, Gray, DarkGray, Black
	}

	/** Enum containing every REV Blinkin pattern */
	public enum BlinkinPatterns {
		RainbowRainbowPalette, RainbowPartyPalette, RainbowOceanPalette, RainbowLavePalette, RainbowForestPalette, RainbowWithGlitter, Confetti, ShotRed, ShotBlue, ShotWhite, SinelonRainbowPalette, SinelonPartyPalette, SineloneOceanPalette, SinelonLavaPalette, SinelonForestPalette, BPMRainbowPalette, BPMPartyPalette, BPMOceanPalette, BPMLavaPalette, BPMForestPalette, FireMedium, FireLarge, TwinklesRainbowPalette, TwinklesPartyPalette, TwinklesOceanPalette, TwinklesLavaPalette, TwinklesForestPalette, ColorWavesRainbowPalette, ColorWavesPartyPalette, ColorWavesOceanPalette, ColorWavesLavaPalette, ColorWavesForestPalette, LarsonScannerRed, LarsonScannerGray, LightChaseRed, LightCaseBlue, LightChaseGray, HeartbeatRed, HeartbeatBlue, HeartbeatWhite, HeartbeatGray, BreathRed, BreathBlue, BreathGray, StrobeRed, StrobeBlue, StrobeGold, StrobeWhite,

		EndToEndBlendToBlackC1P, LarsonScannerC1P, LightChaseC1P, HeartbeatSlowC1P, HeartbeatMediumC1P, HeartbeatFastC1P, BreathSlowC1P, BreathFastC1P, ShotC1P, StrobeC1P,

		EndToEndBlendToBlackC2P, LarsonScannerC2P, LightChaseC2P, HeartbeatSlowC2P, HeartbeatMediumC2P, HeartbeatFastC2P, BreathSlowC2P, BreathFastC2P, ShotC2P, StrobeC2P,

		SparkleColor1onColor2, SparkleColor2onColor1, ColorGradient, Color1and2, BPMColor1and2, EndtoEndBlendColor1to2, EndtoEndBlend, Color1andColor2noblending, TwinklesColor1and2, ColorWavesColor1and2, SinelonColor1and2,

		HotPink, DarkRed, Red, RedOrange, Orange, Gold, Yellow, LawnGreen, Lime, DarkGreen, Green, BlueGreen, Aqua, SkyBlue, DarkBlue, Blue, BlueViolet, Violet, White, Gray, DarkGray, Black

	}

	private double getBlinkinValues(BlinkinPatterns color) {

		switch (color) {
			case HotPink :
				return 0.57;
			case DarkRed :
				return 0.59;
			case Red :
				return 0.61;
			case RedOrange :
				return 0.63;
			case Orange :
				return 0.65;
			case Gold :
				return 0.67;
			case Yellow :
				return 0.69;
			case LawnGreen :
				return 0.71;
			case Lime :
				return 0.73;
			case DarkGreen :
				return 0.75;
			case Green :
				return 0.77;
			case BlueGreen :
				return 0.79;
			case Aqua :
				return 0.81;
			case SkyBlue :
				return 0.83;
			case DarkBlue :
				return 0.85;
			case Blue :
				return 0.87;
			case BlueViolet :
				return 0.89;
			case Violet :
				return 0.91;
			case White :
				return 0.93;
			case Gray :
				return 0.95;
			case DarkGray :
				return 0.97;
			case Black :
				return 0.99;
			default :
				return 0.99;
		}
	}

	/**
	 * Sets SN_Blinkin to the specified value
	 *
	 * @param a_color
	 *            Pattern to assign module to display
	 */
	public void setBlinkin(BlinkinPatterns a_color) {
		this.set(getBlinkinValues(a_color));
	}
}
