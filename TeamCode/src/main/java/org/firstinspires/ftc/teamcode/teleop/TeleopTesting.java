package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.BlockVision;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.PowerTakeOff;
import org.firstinspires.ftc.teamcode.utility.DriveSpeedEnum;
import org.firstinspires.ftc.teamcode.utility.PlacePosEnum;
import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

public class TeleopTesting {

    Climber climber;
    Drivetrain driveTrain;
    BlockVision blockVision;
    PowerTakeOff powerTakeOff;
    IntakeSystem intakeSystem;
    OuttakeSystem outtakeSystem;
    //INPUTS
    double driveForward;
    double driveStrafe;
    double driveRotation;
    boolean driveFast;
    DriveSpeedEnum driveSpeed;


    //Various Variables
    double testValue = Constants.Intake.wristStore;
    double testValue2 = 0.33333;
    boolean intakeingColor = false;
    boolean intakeing = false;

    int PTOError = 0;

    //this is here because I have to have a teleop blue and teleop red
    HardwareMap hardwareMap;
    Telemetry telemetry;
    Gamepad gamepad1;
    Gamepad gamepad2;
    RobotSideEnum robotSide;

    public TeleopTesting(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, RobotSideEnum robotSide) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.robotSide = robotSide;
    }


    public void init() {
        climber = new Climber(hardwareMap);
        driveTrain = new Drivetrain(hardwareMap);
        powerTakeOff = new PowerTakeOff(hardwareMap);
        intakeSystem = new IntakeSystem(hardwareMap, robotSide);
        outtakeSystem = new OuttakeSystem(hardwareMap);
    }


    public void loop() {
        //INPUTS
        driveForward = -gamepad1.left_stick_y;
        driveStrafe = -gamepad1.left_stick_x;

        driveRotation = -gamepad1.right_stick_x;
        driveFast = gamepad1.right_bumper;

        //DRIVING
        if (driveFast) {
            driveSpeed = DriveSpeedEnum.Fast;
        } else {
            driveSpeed = DriveSpeedEnum.Slow;
        }

        if (!powerTakeOff.isEnabled()) {
            //Regular time
            driveTrain.drive(driveForward, driveStrafe, driveRotation, driveSpeed);
        } else {
            //PTO Time
            //fancy math for PTO feedforward
            PTOError = Math.abs(driveTrain.getPTOPos() - driveTrain.getPTOTPos());
            driveTrain.PTOLoop(Math.min(0.25, Math.max( ( (PTOError - 60) / 500), 0 )));

//            driveTrain.setPTOPower(gamepad1.right_trigger - gamepad1.left_trigger);

            if (gamepad1.dpad_up) {
                driveTrain.setPTOPos(Constants.PTO.motorClimb);
            } else if (gamepad1.dpad_down) {
                driveTrain.setPTOPos(Constants.PTO.motorDrop);
            }
            if (gamepad1.a) {
                powerTakeOff.disable();
            }
        }

        //PTO Enabling
        if (gamepad1.back) {
            powerTakeOff.enable();
            driveTrain.setRunToPos();
        }

        if (gamepad1.a) {
            intakeingColor = false;
        }
        if (gamepad1.b) {
            intakeSystem.storePos();
        }
        if (gamepad1.x) {
            intakeingColor = true;
        }
        if (gamepad1.y) {
            intakeing = true;
        }



        if (gamepad1.left_bumper) {
            outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
        } else {
            outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
        }
        if (gamepad1.dpad_right) {
            intakeSystem.setIntakePower(Constants.Intake.intakeSpeed);
        }

        if (gamepad1.dpad_left) {
            intakeSystem.setIntakePower(-Constants.Intake.intakeSpeed);
        }
        if (gamepad1.dpad_up && !powerTakeOff.isEnabled()) {
            outtakeSystem.placePos(PlacePosEnum.highSpecimen);
        }
        if (gamepad1.dpad_down && !powerTakeOff.isEnabled()) {
            outtakeSystem.storePos();
        }

        //intaking loop
        if (intakeing) {
            intakeSystem.pickupPosWithTime();
            if (intakeSystem.intakeUntil()) {
                intakeSystem.storePos();
                intakeing = false;
            }
        }
        if (intakeingColor) {
            intakeSystem.pickupPosWithTime();
            if (intakeSystem.intakeUntilColor()) {
                intakeSystem.storePos();
                intakeingColor = false;
            }
        }


        //HSlides
//        testValue += 4 * (gamepad1.right_trigger - gamepad1.left_trigger);
        testValue += 0.003 * (gamepad1.right_trigger - gamepad1.left_trigger);
        testValue2 += 0.003 * (gamepad1.right_stick_y);
//        outtakeSystem.setVSlidePos((int)testValue);
//        intakeSystem.setHSlidePos((int) testValue);
//        intakeSystem.setIntakeServoPos(testValue);
        outtakeSystem.setArmPos(testValue2);
        telemetry.addData("testValue", testValue);
        telemetry.addData("testValue2", testValue2);
        telemetry.addData("HSlidepos", intakeSystem.getHSlidePos());
        telemetry.addData("VSlidepos", outtakeSystem.getVSlidePos());
        telemetry.addData("power", driveTrain.pidControl.update(driveTrain.leftFront.getCurrentPosition()));

        /*
        //HSlide
        testValue += 3 * (gamepad1.right_trigger - gamepad1.left_trigger);
        intakeSystem.setHSlidePos((int) testValue);
        telemetry.addData("targetPos1", (int) testValue);
        telemetry.addData("targetPos2", intakeSystem.getHSlideTargetPos());
        telemetry.addData(" Pos", intakeSystem.getHSlidePos());
        */
        /*
        //VSlide
        testValue2 += 3 * (gamepad1.right_trigger - gamepad1.left_trigger);
        outtakeSystem.setVSlidePos((int) testValue2);
        telemetry.addData("targetPos", (int) testValue2);
        telemetry.addData(" Pos", outtakeSystem.getVSlidePos());
        */
        /*
        //PTO
        telemetry.addData("Tpos", driveTrain.getPTOTPos());
        telemetry.addData("pos", driveTrain.getPTOPos());
        telemetry.addData("power", driveTrain.pidControl.update(driveTrain.leftFront.getCurrentPosition()));
        powerTakeOff.setLeftPos(testValue);
        powerTakeOff.setRightPos(testValue);
        telemetry.addData("testValue", testValue);
        telemetry.addData("l", powerTakeOff.PTOLeft.getPosition());
        telemetry.addData("r", powerTakeOff.PTORight.getPosition());
        */
        //Climber
        telemetry.addData("Pos", climber.getPos());
        climber.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        telemetry.addData("Current Position", climber.getPos());
        //intake color
//        intakeSystem.setIntakePower(gamepad1.right_trigger - gamepad1.left_trigger);
//        intakeSystem.setIntakePower(testValue);
//        telemetry.addData("power", testValue);
        telemetry.addData("Claw r", intakeSystem.directColor().red);
        telemetry.addData("Claw g", intakeSystem.directColor().green);
        telemetry.addData("Claw b", intakeSystem.directColor().blue);
        telemetry.addData("Claw a", intakeSystem.directColor().alpha);
        telemetry.addData("Claw distance", intakeSystem.distance());
        telemetry.addData("color", intakeSystem.color());
//
//        telemetry.addData("claw", outtakeSystem.outtakeArm.getClawPos());

        outtakeSystem.update();
        telemetry.update();
    }

    public void stop() {
        driveTrain.stopDrive();
    }
}
