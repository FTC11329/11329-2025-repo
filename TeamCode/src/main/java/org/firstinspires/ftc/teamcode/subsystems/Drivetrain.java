package org.firstinspires.ftc.teamcode.subsystems;



import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.MyOpticalLocalizer;
import org.firstinspires.ftc.teamcode.utility.DriveSpeedEnum;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class Drivetrain extends MecanumDrive {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(9, 5.5, 2); //D3
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(9, 0, 0);
    public static double LATERAL_MULTIPLIER = 1;
    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;
    MyOpticalLocalizer myOtos;
    public final DcMotorEx leftFront;
    public final DcMotorEx leftRear;
    public final DcMotorEx rightRear;
    public final DcMotorEx rightFront;
    private final List<DcMotorEx> motors;

    private final VoltageSensor batteryVoltageSensor;

    private final List<Integer> lastEncoderPositions = new ArrayList<>();
    private final List<Integer> lastEncoderVelocities = new ArrayList<>();

    private Telemetry telemetry;

    public Drivetrain(HardwareMap hardwareMap) {
        super(hardwareMap, new Pose2d(0,0,0));
        myOtos = new MyOpticalLocalizer(hardwareMap);


        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightBack");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        List<Integer> lastTrackingEncoderPositions = new ArrayList<>();
        List<Integer> lastTrackingEncoderVelocities = new ArrayList<>();

    }

    public void drive(double forward, double strafe, double turn, DriveSpeedEnum driveSpeed) {
        double speed = 0;
        if (driveSpeed == DriveSpeedEnum.Fast) {
            speed = Constants.Drivetrain.fastSpeed;
        } else if (driveSpeed == DriveSpeedEnum.Slow) {
            speed = Constants.Drivetrain.slowSpeed;
        } else if (driveSpeed == DriveSpeedEnum.Auto) {
            speed = 1;
        } else if (driveSpeed == DriveSpeedEnum.SuperFast) {
            speed = 0.3;
        }

        setWeightedDrivePower(new Pose2d(forward * speed, strafe * speed, turn * speed));
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.position.x) + Math.abs(drivePower.position.y)
                + Math.abs(drivePower.heading.toDouble()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.position.x)
                    + VY_WEIGHT * Math.abs(drivePower.position.y)
                    + OMEGA_WEIGHT * Math.abs(drivePower.heading.toDouble());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.position.x / denom,
                    VY_WEIGHT * drivePower.position.y / denom,
                    OMEGA_WEIGHT * drivePower.heading.toDouble() / denom
            );
        }

        setDrivePowers(new PoseVelocity2d(vel.position, vel.heading.toDouble()));
    }

    public Pose2d getPoseEstimateOptical() {
        return new Pose2d(myOtos.getPosition().x, myOtos.getPosition().y, myOtos.getPosition().h);
    }

    public void setOtosPosition(double x, double y, double h) {
        myOtos.setPosition(x, y, h);
    }

    public void stopDrive() {
        drive(0, 0, 0, DriveSpeedEnum.Slow);
    }
}
