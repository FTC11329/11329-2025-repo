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

public class Teleop {

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
    double testValue = 0.3333;
    double testValue2 = 0;
    boolean intakeingColor = false;
    boolean intakeing = false;


    //this is here because I have to have a teleop blue and teleop red
    HardwareMap hardwareMap;
    Telemetry telemetry;
    Gamepad gamepad1;
    Gamepad gamepad2;
    RobotSideEnum robotSide;

    public Teleop(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, RobotSideEnum robotSide) {
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
            driveTrain.PTOLoop();

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
            intakeSystem.pickupPos();
            if (intakeSystem.intakeUntil()) {
                intakeSystem.storePos();
                intakeing = false;
            }
        }
        if (intakeingColor) {
            intakeSystem.pickupPos();
            if (intakeSystem.intakeUntilColor()) {
                intakeSystem.storePos();
                intakeingColor = false;
            }
        }


        //HSlides
        testValue += 0.002 * (gamepad1.right_trigger - gamepad1.left_trigger);
        testValue2 += 5 * (gamepad1.right_stick_y);
//        outtakeSystem.setClawPos(testValue);
        outtakeSystem.setArmPos(testValue);
        outtakeSystem.setVSlidePos((int)testValue2);
//        outtakeSystem.setVSlidePos((int)testValue);
//        climber.setPos((int)testValue);
        telemetry.addData("targetPos1", testValue);
        telemetry.addData("targetPos2", testValue2);

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

        //intake color
//        intakeSystem.setIntakePower(testValue);
//        telemetry.addData("power", testValue);
        telemetry.addData("Claw r", intakeSystem.directColor().red);
        telemetry.addData("Claw g", intakeSystem.directColor().green);
        telemetry.addData("Claw b", intakeSystem.directColor().blue);
        telemetry.addData("Claw a", intakeSystem.directColor().alpha);
        telemetry.addData("Claw distance", intakeSystem.distance());
        telemetry.addData("color", intakeSystem.color());


        telemetry.update();
    }

    public void stop() {
        driveTrain.stopDrive();
    }
}
