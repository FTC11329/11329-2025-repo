package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.Constants;


public class ColorFunctions {
    public static ColorEnum toColor(NormalizedRGBA rgba) {
        double[] colorList = {rgba.red, rgba.green, rgba.blue, rgba.alpha};

        double blueDistance = Math.sqrt(Math.pow(colorList[0] - Constants.Color.blue[0], 2) + Math.pow(colorList[1] - Constants.Color.blue[1], 2) + Math.pow(colorList[3] - Constants.Color.blue[3], 2));
        double redDistance = Math.sqrt(Math.pow(colorList[0] - Constants.Color.red[0], 2) + Math.pow(colorList[1] - Constants.Color.red[1], 2) + Math.pow(colorList[3] - Constants.Color.red[3], 2));
        double yellowDistance = Math.sqrt(Math.pow(colorList[0] - Constants.Color.yellow[0], 2) + Math.pow(colorList[1] - Constants.Color.yellow[1], 2) + Math.pow(colorList[3] - Constants.Color.yellow[3], 2));

        if (redDistance <= blueDistance && yellowDistance <= blueDistance) {
            return ColorEnum.blue;
        } else if (yellowDistance <= redDistance && blueDistance <= redDistance) {
            return ColorEnum.red;
        } else {
            return ColorEnum.yellow;
        }

    }


}