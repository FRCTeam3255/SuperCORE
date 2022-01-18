package com.frcteam3255.utils;

public class SN_LED {

    public enum BlinkinSolidColors{

        HotPink, DarkRed, Red, RedOrange, Orange,
        Gold, Yellow, LawnGreen, Lime, DarkGreen,
        Green, BlueGreen, Aqua, SkyBlue, DarkBlue,
        Blue, BlueViolet, Violet, White, Gray, DarkGray, Black 
    }

    public double getBlinkinSolidColorValue(BlinkinSolidColors color) {

        switch (color) {
            case HotPink: return 0.57;
            case DarkRed: return 0.59;
            case Red: return 0.61;
            case RedOrange: return 0.63;
            case Orange: return 0.65;
            case Gold: return 0.67;
            case Yellow: return 0.69;
            case LawnGreen: return 0.71;
            case Lime: return 0.73;
            case DarkGreen: return 0.75;
            case Green: return 0.77;
            case BlueGreen: return 0.79;
            case Aqua: return 0.81;
            case SkyBlue: return 0.83;
            case DarkBlue: return 0.85;
            case Blue: return 0.87;
            case BlueViolet: return 0.89;
            case Violet: return 0.91;
            case White: return 0.93;
            case Gray: return 0.95;
            case DarkGray: return 0.97;
            case Black: return 0.99;
            default: return 0.99;
        }
    }




    
}
