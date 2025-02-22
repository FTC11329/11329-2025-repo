package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.Encoder;
import com.pedropathing.localization.constants.OTOSConstants;
import com.pedropathing.localization.constants.ThreeWheelConstants;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Mat;

public class LConstants {
    static {
        OTOSConstants.useCorrectedOTOSClass = false;
        OTOSConstants.hardwareMapName = "sensor_otos";
        OTOSConstants.linearUnit = DistanceUnit.INCH;
        OTOSConstants.angleUnit = AngleUnit.RADIANS;
        OTOSConstants.offset = new SparkFunOTOS.Pose2D(0.0, 0.0, Math.toRadians(-90));
        OTOSConstants.linearScalar = 0.9766443303;
        OTOSConstants.angularScalar = 0.9977134153;
        ;
    }

}
