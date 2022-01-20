package com.frcteam3255.utils;

public class SN_LED {

    public enum BlinkinFixedPalettePatterns {
        RainbowRainbowPalette, RainbowPartyPalette, RainbowOceanPalette, RainbowLavePalette, RainbowForestPalette,
        RainbowWithGlitter, Confetti, ShotRed, ShotBlue, ShotWhite, SinelonRainbowPalette, SinelonPartyPalette,
        SineloneOceanPalette, SinelonLavaPalette, SinelonForestPalette, BPMRainbowPalette, BPMPartyPalette,
        BPMOceanPalette, BPMLavaPalette, BPMForestPalette, FireMedium, FireLarge, TwinklesRainbowPalette,
        TwinklesPartyPalette, TwinklesOceanPalette, TwinklesLavaPalette, TwinklesForestPalette,
        ColorWavesRainbowPalette, ColorWavesPartyPalette, ColorWavesOceanPalette, ColorWavesLavaPalette,
        ColorWavesForestPalette, LarsonScannerRed, LarsonScannerGray, LightChaseRed, LightCaseBlue, LightChaseGray,
        HeartbeatRed, HeartbeatBlue, HeartbeatWhite, HeartbeatGray, BreathRed, BreathBlue, BreathGray, StrobeRed,
        StrobeBlue, StrobeGold, StrobeWhite
    }

    public enum BlinkinColor1Patterns {
        EndToEndBlendToBlackC1P, LarsonScannerC1P, LightChaseC1P, HeartbeatSlowC1P,
        HeartbeatMediumC1P, HeartbeatFastC1P, BreathSlowC1P, BreathFastC1P, ShotC1P, StrobeC1P
    }

    public enum BlinkinColor2Patterns {
        EndToEndBlendToBlackC2P, LarsonScannerC2P, LightChaseC2P, HeartbeatSlowC2P,
        HeartbeatMediumC2P, HeartbeatFastC2P, BreathSlowC2P, BreathFastC2P, ShotC2P, StrobeC2P
    }

    public enum BlinkinColor1and2Pattern {
        SparkleColor1onColor2, SparkleColor2onColor1, ColorGradient, Color1and2, BPMColor1and2, EndtoEndBlendColor1to2,
        EndtoEndBlend, Color1andColor2noblending, TwinklesColor1and2, ColorWavesColor1and2, SinelonColor1and2
    }

    public enum BlinkinSolidColors {
        HotPink, DarkRed, Red, RedOrange, Orange,
        Gold, Yellow, LawnGreen, Lime, DarkGreen,
        Green, BlueGreen, Aqua, SkyBlue, DarkBlue,
        Blue, BlueViolet, Violet, White, Gray, DarkGray, Black
    }

    public enum BlinkinPatterns {
        RainbowRainbowPalette, RainbowPartyPalette, RainbowOceanPalette, RainbowLavePalette, RainbowForestPalette,
        RainbowWithGlitter, Confetti, ShotRed, ShotBlue, ShotWhite, SinelonRainbowPalette, SinelonPartyPalette,
        SineloneOceanPalette, SinelonLavaPalette, SinelonForestPalette, BPMRainbowPalette, BPMPartyPalette,
        BPMOceanPalette, BPMLavaPalette, BPMForestPalette, FireMedium, FireLarge, TwinklesRainbowPalette,
        TwinklesPartyPalette, TwinklesOceanPalette, TwinklesLavaPalette, TwinklesForestPalette,
        ColorWavesRainbowPalette, ColorWavesPartyPalette, ColorWavesOceanPalette, ColorWavesLavaPalette,
        ColorWavesForestPalette, LarsonScannerRed, LarsonScannerGray, LightChaseRed, LightCaseBlue, LightChaseGray,
        HeartbeatRed, HeartbeatBlue, HeartbeatWhite, HeartbeatGray, BreathRed, BreathBlue, BreathGray, StrobeRed,
        StrobeBlue, StrobeGold, StrobeWhite,

        EndToEndBlendToBlackC1P, LarsonScannerC1P, LightChaseC1P, HeartbeatSlowC1P,
        HeartbeatMediumC1P, HeartbeatFastC1P, BreathSlowC1P, BreathFastC1P, ShotC1P, StrobeC1P,

        EndToEndBlendToBlackC2P, LarsonScannerC2P, LightChaseC2P, HeartbeatSlowC2P, HeartbeatMediumC2P,
        HeartbeatFastC2P, BreathSlowC2P, BreathFastC2P, ShotC2P, StrobeC2P,

        SparkleColor1onColor2, SparkleColor2onColor1, ColorGradient, Color1and2,
        BPMColor1and2, EndtoEndBlendColor1to2, EndtoEndBlend, Color1andColor2noblending, TwinklesColor1and2,
        ColorWavesColor1and2, SinelonColor1and2,

        HotPink, DarkRed, Red, RedOrange, Orange, Gold, Yellow, LawnGreen, Lime, DarkGreen, Green, BlueGreen, Aqua,
        SkyBlue, DarkBlue, Blue, BlueViolet, Violet, White, Gray, DarkGray, Black

    }

    public double getBlinkinSolidColorValue(BlinkinSolidColors color) {

        switch (color) {
            case HotPink:
                return 0.57;
            case DarkRed:
                return 0.59;
            case Red:
                return 0.61;
            case RedOrange:
                return 0.63;
            case Orange:
                return 0.65;
            case Gold:
                return 0.67;
            case Yellow:
                return 0.69;
            case LawnGreen:
                return 0.71;
            case Lime:
                return 0.73;
            case DarkGreen:
                return 0.75;
            case Green:
                return 0.77;
            case BlueGreen:
                return 0.79;
            case Aqua:
                return 0.81;
            case SkyBlue:
                return 0.83;
            case DarkBlue:
                return 0.85;
            case Blue:
                return 0.87;
            case BlueViolet:
                return 0.89;
            case Violet:
                return 0.91;
            case White:
                return 0.93;
            case Gray:
                return 0.95;
            case DarkGray:
                return 0.97;
            case Black:
                return 0.99;
            default:
                return 0.99;
        }
    }

}
