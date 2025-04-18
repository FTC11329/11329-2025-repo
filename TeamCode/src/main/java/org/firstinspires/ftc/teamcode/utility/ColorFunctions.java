package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.Constants;


public class ColorFunctions {
    public static ColorEnum toColor(NormalizedRGBA rgba) {
        double[] colorList = {rgba.red, rgba.green, rgba.blue, rgba.alpha};

        double blueDistance       = Math.sqrt(Math.pow(colorList[0] - Constants.Color.blue[0], 2) + Math.pow(colorList[1] - Constants.Color.blue[1], 2) + Math.pow(colorList[2] - Constants.Color.blue[2], 2));
        double redDistance        = Math.sqrt(Math.pow(colorList[0] - Constants.Color.red[0], 2) + Math.pow(colorList[1] - Constants.Color.red[1], 2) + Math.pow(colorList[2] - Constants.Color.red[2], 2));
        double redEdgeDistance    = Math.sqrt(Math.pow(colorList[0] - Constants.Color.redEdge[0], 2) + Math.pow(colorList[1] - Constants.Color.redEdge[1], 2) + Math.pow(colorList[2] - Constants.Color.redEdge[2], 2));
        double yellowDistance     = Math.sqrt(Math.pow(colorList[0] - Constants.Color.yellow[0], 2) + Math.pow(colorList[1] - Constants.Color.yellow[1], 2) + Math.pow(colorList[2] - Constants.Color.yellow[2], 2));
        double yellowEdgeDistance = Math.sqrt(Math.pow(colorList[0] - Constants.Color.yellowEdge[0], 2) + Math.pow(colorList[1] - Constants.Color.yellowEdge[1], 2) + Math.pow(colorList[2] - Constants.Color.yellowEdge[2], 2));
        double emptyDistance      = Math.sqrt(Math.pow(colorList[0] - Constants.Color.empty[0], 2) + Math.pow(colorList[1] - Constants.Color.empty[1], 2) + Math.pow(colorList[2] - Constants.Color.empty[2], 2));

        redDistance = Math.min(redEdgeDistance, redDistance);
        yellowDistance = Math.min(yellowEdgeDistance, yellowDistance);

        if (redDistance >= blueDistance && yellowDistance >= blueDistance && emptyDistance >= blueDistance) {
            return ColorEnum.blue;
        } else if (yellowDistance >= redDistance && blueDistance >= redDistance && emptyDistance >= redDistance) {
            return ColorEnum.red;
        } else if (redDistance >= yellowDistance && blueDistance >= yellowDistance && emptyDistance >= yellowDistance){
            return ColorEnum.yellow;
        } else {
            return ColorEnum.empty;
        }

    }


}