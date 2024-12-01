package org.firstinspires.ftc.teamcode.roadrunner;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.Pose2d;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MyOpticalLocalizer {
    private SparkFunOTOS otos;

    public MyOpticalLocalizer(HardwareMap hardwareMap) {
        otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        configureOtos();
    }

    private void configureOtos() {
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.RADIANS);

        otos.calibrateImu();
        otos.resetTracking();

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, Math.toRadians(-90)); //x = -8.1941 y = -6.3078
        otos.setOffset(offset);

        otos.setLinearScalar(1.00908);
        otos.setAngularScalar(0.99319);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        otos.getVersionInfo(hwVersion, fwVersion);
    }

    public PoseVelocity2d getVelocity() {
        SparkFunOTOS.Pose2D OTOSVelocity = otos.getVelocity();

        return new PoseVelocity2d(new Vector2d(OTOSVelocity.x, OTOSVelocity.y), OTOSVelocity.h);
    }

    public SparkFunOTOS.Pose2D getPosition() {
        return otos.getPosition();
    }

    public Pose2d getPositionACM() {
        return new Pose2d(otos.getPosition().x, otos.getPosition().y, otos.getPosition().h);
    }


    public void setPosition(double x, double y, double h) {
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(x, y, h);
        otos.setPosition(currentPosition);
    }
    public void setPosition(SparkFunOTOS.Pose2D currentPosition) {
        otos.setPosition(currentPosition);
    }
    public void setPosition(Pose2d currentPosition) {
        otos.setPosition(new SparkFunOTOS.Pose2D(currentPosition.position.x, currentPosition.position.y, currentPosition.heading.toDouble()));
    }
}
