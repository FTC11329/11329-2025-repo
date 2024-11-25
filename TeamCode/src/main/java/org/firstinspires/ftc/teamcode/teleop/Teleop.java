package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.utility.DriveSpeedEnum;

@TeleOp(name = "Tele-op", group = "Allen op mode")
public class Teleop extends OpMode {
    Drivetrain driveTrain;
    //INPUTS
    double driveForward;
    double driveStrafe;
    double driveRotation;
    boolean driveFast;
    DriveSpeedEnum driveSpeed;


    @Override
    public void init() {
        driveTrain = new Drivetrain(hardwareMap);
    }

    @Override
    public void loop() {

        //INPUTS
        driveForward = gamepad1.left_stick_y;
        driveStrafe = gamepad1.left_stick_x;
        driveRotation = gamepad1.right_stick_x;
        driveFast = gamepad1.right_bumper;


        //DRIVING
        if (driveFast) {
            driveSpeed = DriveSpeedEnum.Fast;
        } else {
            driveSpeed = DriveSpeedEnum.Slow;
        }
        driveTrain.drive(driveForward, driveStrafe, driveRotation, driveSpeed);

    }
}
