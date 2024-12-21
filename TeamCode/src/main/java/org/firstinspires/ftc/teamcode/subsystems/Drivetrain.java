package org.firstinspires.ftc.teamcode.subsystems;



import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.DriveSpeedEnum;
import org.firstinspires.ftc.teamcode.utility.SimplePIDControl;

import java.util.Arrays;
import java.util.List;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class Drivetrain extends MecanumDrive {
    public final DcMotorEx leftFront;
    public final DcMotorEx leftBack;
    public final DcMotorEx rightBack;
    public final DcMotorEx rightFront;
    private final List<DcMotorEx> motors;

    public SimplePIDControl pidControl;

    public boolean isAtPTOPosition = false;

    public Drivetrain(HardwareMap hardwareMap) {
        super(hardwareMap, new Pose2d(0,0,0));

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
//        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
//        leftBack = hardwareMap.get(DcMotorEx.class, "backLeft");
//        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");
//        rightBack = hardwareMap.get(DcMotorEx.class, "backRight");

        motors = Arrays.asList(leftFront, leftBack, rightBack, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        pidControl = new SimplePIDControl(Constants.PTO.p, Constants.PTO.i, Constants.PTO.d);
    }

    public void drive(double forward, double strafe, double turn, DriveSpeedEnum driveSpeed) {
        double speed = 0;
        if (driveSpeed == DriveSpeedEnum.Fast) {
            speed = Constants.Drivetrain.fastSpeed;
        } else if (driveSpeed == DriveSpeedEnum.Slow) {
            speed = Constants.Drivetrain.slowSpeed;
        } else if (driveSpeed == DriveSpeedEnum.Auto) {
            speed = 1;
        } else if (driveSpeed == DriveSpeedEnum.PTOSpeed) {
            speed = Constants.PTO.speed;
        }

        setWeightedDrivePower(new Pose2d(forward * speed, strafe * speed, turn * speed));
    }

    public void PTOLoop(double feedForward) {
        rightFront.setPower(pidControl.update(rightFront.getCurrentPosition(), feedForward));
        rightBack.setPower(pidControl.update(rightFront.getCurrentPosition(), feedForward));
        leftFront.setPower(pidControl.update(leftFront.getCurrentPosition(), feedForward));
        leftBack.setPower(pidControl.update(leftFront.getCurrentPosition(), feedForward));
    }

    public void setRunToPos() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void setPTOPos(int ptoPos) {
        pidControl.setTargetValue(ptoPos);
        pidControl.setTargetValue(ptoPos);
    }
    public void setPTOPower(double power) {
        rightFront.setPower(power);
        rightBack.setPower(power);
        leftFront.setPower(power);
        leftBack.setPower(power);
    }
    public int getPTOTPos() {
        return (int) pidControl.getTargetValue();
    }
    public int getPTOPos() {
        return leftFront.getCurrentPosition();
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.position.x) + Math.abs(drivePower.position.y)
                + Math.abs(drivePower.heading.toDouble()) > 1) {
            // re-normalize the powers according to the weights
            double denom = Math.abs(drivePower.position.x)
                    + Math.abs(drivePower.position.y)
                    + Math.abs(drivePower.heading.toDouble());

            vel = new Pose2d(
                    drivePower.position.x / denom,
                    drivePower.position.y / denom,
                    drivePower.heading.toDouble() / denom
            );
        }

        setDrivePowers(new PoseVelocity2d(vel.position, vel.heading.toDouble()));
    }
    public double[] getDrivePowers() {
        return new double[]{leftFront.getPower(),
                           rightFront.getPower(),
                           leftBack.getPower(),
                           rightBack.getPower()
                          };
    }

    public void stopDrive() {
        drive(0, 0, 0, DriveSpeedEnum.Slow);
    }
}
