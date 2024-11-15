package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MyOpticalLocalizer implements Localizer {
    private SparkFunOTOS myOtos;

    private double lastXPos;
    private double lastYPos;
    private double lastHeading;

    boolean initialized = false;


    public MyOpticalLocalizer(HardwareMap hardwareMap) {
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        configureOtos();
    }

    public Twist2dDual<Time> update() {

        if (!initialized) {
            initialized = true;

            lastXPos = myOtos.getPosition().x;
            lastYPos = myOtos.getPosition().y;
            lastHeading = myOtos.getPosition().h;



            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );
        }

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                myOtos.getPosition().x - lastXPos,
                                myOtos.getVelocity().x,
                        }),
                        new DualNum<Time>(new double[] {
                                myOtos.getPosition().y - lastYPos,
                                myOtos.getVelocity().y,
                        })
                ),
                new DualNum<>(new double[] {
                        myOtos.getPosition().h - lastHeading,
                        myOtos.getVelocity().h,
                })
        );

        lastXPos = myOtos.getPosition().x;
        lastYPos = myOtos.getPosition().y;
        lastHeading = myOtos.getPosition().h;

        return twist;
    }

    private void configureOtos() {
        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.RADIANS);

        myOtos.calibrateImu();
        myOtos.resetTracking();

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, Math.toRadians(0)); //x = -8.1941 y = -6.3078
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
