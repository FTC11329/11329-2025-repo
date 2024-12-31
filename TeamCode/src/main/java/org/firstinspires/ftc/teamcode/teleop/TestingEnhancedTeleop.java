package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;
import org.firstinspires.ftc.teamcode.subsystems.BlockVision;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.PowerTakeOff;
import org.firstinspires.ftc.teamcode.utility.DriveSpeedEnum;
import org.firstinspires.ftc.teamcode.utility.PadButton;
import org.firstinspires.ftc.teamcode.utility.PlacePosEnum;
import org.firstinspires.ftc.teamcode.utility.PoseFunctions;
import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

@TeleOp(name = "Tele-op enhansed", group = " Comp mode")
public class TestingEnhancedTeleop extends OpMode {
    ElapsedTime time = new ElapsedTime();

    Follower follower;
    Drivetrain driveTrain;

    PIDFController headingPIDF = new PIDFController(FollowerConstants.headingPIDFCoefficients);
    PIDFController xPIDF = new PIDFController(FollowerConstants.translationalPIDFCoefficients);
    PIDFController yPIDF = new PIDFController(FollowerConstants.translationalPIDFCoefficients);

    //INPUTS
    double rotError;
    double driveForward;
    double driveStrafe;
    double driveRotation;
    double targetHeading = Math.toRadians(-90); //in radians
    double targetX = 24;
    double targetY = -48;

    //Various Variables
    DriveSpeedEnum driveSpeed;
    boolean manualRotation = false;
    boolean manualRotationDebounce = false;
    boolean manualDrive = true;
    boolean manualDriveDebounce = false;
    boolean hasSample = false;
    boolean sampleDebounce = false;
    PadButton padButton = PadButton.None;

    Pose currentPose = new Pose(0,0,0);

    public void init() {
        follower = new Follower(hardwareMap);
        driveTrain = new Drivetrain(hardwareMap);

        follower.setStartingPose(new Pose(0,-48,Math.toRadians(90)));
    }

    public void start() {
        follower.startTeleopDrive();
    }

    public void loop() {
        //INPUTS
        currentPose = follower.getPose();
        switch (PoseFunctions.getLocation(currentPose)) {
            case leftSideSub:
                targetHeading = Math.toRadians(0);
                break;
            case basket:
                if (hasSample) {
                    targetHeading = Math.toRadians(45);
                } else {
                    targetHeading = Math.toRadians(90);
                }
                break;
            case frontSub:
                targetHeading = Math.toRadians(90);
                break;
            case observation:
                if (hasSample) {
                    targetHeading = Math.toRadians(315);
                } else {
                    targetHeading = Math.toRadians(90);
                }
                break;
            case rightSideSub:
                targetHeading = Math.toRadians(180);
                break;
            case otherSide:
                break;
        }
        rotError = currentPose.getHeading() - targetHeading;
        //uses the fastest rotation to the goal
        if (rotError > Math.PI) {
            rotError -= 2 * Math.PI;
        } else if (rotError < -Math.PI) {
            rotError += 2 * Math.PI;
        }
        headingPIDF.updateError(rotError);

        if (gamepad1.touchpad_finger_1) {
            targetY = (gamepad1.touchpad_finger_1_y * 39.5) - 24;
            targetX = gamepad1.touchpad_finger_1_x * 60;
        }

        yPIDF.updateError(currentPose.getY() - targetY);
        xPIDF.updateError(currentPose.getX() - targetX);

        if (gamepad1.left_stick_button && !manualDriveDebounce) {
            manualDrive = !manualDrive;
            manualDriveDebounce = true;
        }
        if (!gamepad1.left_stick_button) {
            manualDriveDebounce = false;
        }
        if (manualDrive) {
            driveForward = -gamepad1.left_stick_y; //1
            driveStrafe = -gamepad1.left_stick_x; //1
        } else {
            driveForward = -yPIDF.runPIDF(); //1
            driveStrafe = xPIDF.runPIDF(); //1
        }


        if (gamepad1.b) {
            //relocalize
            follower.setPose(new Pose(0,18,Math.toRadians(0)));
        }

        if (gamepad1.right_stick_button && !manualRotationDebounce) {
            manualRotation = !manualRotation;
            manualRotationDebounce = true;
        }
        if (!gamepad1.right_stick_button) {
            manualRotationDebounce = false;
        }

        if (!manualRotation) {
            driveRotation = -headingPIDF.runPIDF();
        } else {
            driveRotation = -gamepad1.right_stick_x;
        }

        if (gamepad1.right_bumper) {
            driveSpeed = DriveSpeedEnum.Fast;
        } else {
            driveSpeed = DriveSpeedEnum.Slow;
        }
        if (gamepad1.a && !sampleDebounce) {
            hasSample = !hasSample;
            sampleDebounce = true;
        }
        if (!gamepad1.a) {
            sampleDebounce = false;
        }

        follower.TeleopDrive(driveForward, driveStrafe, driveRotation, driveSpeed);
        follower.update();

        if (gamepad2.ps) {
            gamepad2.runRumbleEffect(Gamepad.RumbleEffect.deserialize("Allen is"));
        }
        if (gamepad2.touchpad && gamepad2.touchpad_finger_1) {
            if (gamepad2.touchpad_finger_1_x > 0) {
                if (gamepad2.touchpad_finger_1_y > 0) {
                    padButton = PadButton.TopR;
                } else {
                    padButton = PadButton.BotR;
                }
            } else if (gamepad2.touchpad_finger_1_x < 0) {
                if (gamepad2.touchpad_finger_1_y > 0) {
                    padButton = PadButton.TopL;
                } else {
                    padButton = PadButton.BotL;
                }
            }
        } else {
            padButton = PadButton.None;
        }

        telemetry.addData("padButton", padButton);

        telemetry.addData("1", gamepad2.touchpad_finger_1);
        telemetry.addData("x", Math.round(gamepad2.touchpad_finger_1_x*100.0)/100.0);
        telemetry.addData("y", Math.round(gamepad2.touchpad_finger_1_y*100.0)/100.0);
        telemetry.addData("2", gamepad2.touchpad_finger_2);
        telemetry.addData("x", Math.round(gamepad2.touchpad_finger_2_x*100.0)/100.0);
        telemetry.addData("y", Math.round(gamepad2.touchpad_finger_2_y*100.0)/100.0);
        telemetry.addLine();
        telemetry.addData("x", targetX);
        telemetry.addData("y", targetY);
        telemetry.addData("x", yPIDF.runPIDF());
        telemetry.addData("y", yPIDF.runPIDF());
    }

    public void stop() {
        driveTrain.stopDrive();
    }
}
