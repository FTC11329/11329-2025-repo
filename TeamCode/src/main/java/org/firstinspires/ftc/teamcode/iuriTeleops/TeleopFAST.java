package org.firstinspires.ftc.teamcode.iuriTeleops;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.PowerTakeOff;
import org.firstinspires.ftc.teamcode.utility.DriveSpeedEnum;
import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

public class TeleopFAST {
    Drivetrain driveTrain;
    PowerTakeOff powerTakeOff;

    //Input Variables
    double driveForward;
    double driveStrafe;
    double driveRotation;
    DriveSpeedEnum driveSpeed;

    HardwareMap hardwareMap;
    Telemetry telemetry;
    Gamepad gamepad1;
    Gamepad gamepad2;
    RobotSideEnum robotSide;

    public TeleopFAST(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, RobotSideEnum robotSide) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.robotSide = robotSide;
    }

    public void init() {
        driveTrain = new Drivetrain(hardwareMap);
    }

    public void start() {
        powerTakeOff = new PowerTakeOff(hardwareMap);
    }


    public void loop() {
        // Inputs
        driveForward = -gamepad1.left_stick_y;
        driveStrafe = -gamepad1.left_stick_x;
        driveRotation = -gamepad1.right_stick_x;
        if (gamepad1.right_bumper) {
            driveSpeed = DriveSpeedEnum.Fast;
        } else {
            driveSpeed = DriveSpeedEnum.Slow;
        }


        //Drivetrain *****************************************************************************~D
        driveTrain.drive(driveForward, driveStrafe, driveRotation, driveSpeed);
    }
    public void stop() {
        driveTrain.stopDrive();
    }
}