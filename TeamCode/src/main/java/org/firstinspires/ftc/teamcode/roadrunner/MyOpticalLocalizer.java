package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MyOpticalLocalizer {
    SparkFunOTOS myOtos;

    public MyOpticalLocalizer(HardwareMap hardwareMap) {
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        configureOtos();
    }

    public Twist2dDual<Time> update() {
        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                1,
                                2,
                        }),
                        new DualNum<Time>(new double[] {
                                3,
                                4,
                        })
                ),
                new DualNum<>(new double[] {
                        5,
                        6,
                })
        );
        return twist;
    }

    private void configureOtos() {
        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.RADIANS);

        myOtos.calibrateImu();
        myOtos.resetTracking();

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(4.09705, 3.1539, Math.toRadians(-90)); //x = -8.1941 y = -6.3078
        myOtos.setOffset(offset);

        myOtos.setLinearScalar(1.00908);
        myOtos.setAngularScalar(0.99319);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);
    }

    public SparkFunOTOS.Pose2D getPosition() {
        return myOtos.getPosition();
    }

    public void setPosition(double x, double y, double h) {
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(x, y, h);
        myOtos.setPosition(currentPosition);
    }
    public void setPosition(SparkFunOTOS.Pose2D currentPosition) {
        myOtos.setPosition(currentPosition);
    }
}
