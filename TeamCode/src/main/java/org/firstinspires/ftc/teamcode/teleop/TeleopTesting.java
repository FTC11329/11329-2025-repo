package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Attempt89;
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
    Attempt89 blockvision;
    PowerTakeOff powerTakeOff;
    IntakeSystem intakeSystem;
    OuttakeSystem outtakeSystem;
    //INPUTS
    double driveForward;
    double driveStrafe;
    double driveRotation;
    boolean driveFast;
    DriveSpeedEnum driveSpeed;

    boolean climbInit = false;
    boolean climbL1P1 = false;
    boolean climbL2P1 = false;
    boolean climbL2P2 = false;

    //Various Variables
    double testValue = Constants.Intake.wristStore;
    double testValue2 = Constants.Outtake.intakeWallArm;
    double testValue3 = Constants.Outtake.initTeleopArm;
    boolean intakeingColor = false;
    boolean intakeing = false;

    int climberPos = 0;
    int PTOError = 0;

    Pose blockOffset = new Pose();

    Gamepad.RumbleEffect.Builder builder = new Gamepad.RumbleEffect.Builder();

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

        for (int i = 0; i < 10; i++) {
            builder.addStep(1, 0,10);
            builder.addStep(0,1,10);
        }
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
            //Climbing
            if (gamepad1.b) {
                powerTakeOff.disable();
            }

            //fancy math for PTO feedforward
            PTOError = Math.abs(driveTrain.getPTOPos() - driveTrain.getPTOTPos());
            if (PTOError > 250) {
                driveTrain.PTOLoop(0.5);
            } else {
                driveTrain.PTOLoop(0);
            }

            if (gamepad2.dpad_up) {
                driveTrain.setPTOPos(Constants.PTO.motorClimb);
            }
            if (gamepad2.dpad_down) {
                driveTrain.setPTOPos(Constants.PTO.motorDrop);
            }

            climberPos += (int) (20 * (gamepad1.right_trigger - gamepad1.left_trigger));
            climber.setPos(climberPos);
        }

        //PTO Enabling
        if (gamepad1.back) {
            powerTakeOff.enable();
            driveTrain.setRunToPos();
        }

        if (gamepad1.a) {
            intakeingColor = false;
        }
//        if (gamepad1.b) {
//            intakeSystem.storePos();
//        }
//        if (gamepad1.x) {
//            intakeingColor = true;
//        }
//        if (gamepad1.y) {
//            intakeing = true;
//        }



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
//        intakeSystem.setIntakePower(gamepad2.right_trigger);
        telemetry.addData("unjam", intakeSystem.isJammed());
        telemetry.addData("current", intakeSystem.intakeClaw.intakeMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("power", intakeSystem.intakeClaw.intakeMotor.getPower());

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
//        testValue += 5 * (gamepad2.right_trigger - gamepad2.left_trigger);
        testValue += 0.002 * (gamepad2.right_trigger - gamepad2.left_trigger);
        testValue2 += 5 * (-gamepad2.left_stick_y);
        testValue3 += 0.003 * (gamepad2.right_stick_y);

        if (gamepad1.touchpad) {
            gamepad1.runRumbleEffect(builder.build());
        }

        //HSlides
//        outtakeSystem.setVSlidePos((int)testValue);
//        intakeSystem.setHSlidePos((int) testValue);
        intakeSystem.setIntakeServoPos(testValue);
//        outtakeSystem.setArmPos(testValue3);
        telemetry.addData("testValue H", testValue);
        telemetry.addData("testValue2V", testValue2);
        telemetry.addData("testValue3A", testValue3);
        telemetry.addData("HSlidepos  ", intakeSystem.getHSlidePos());
        telemetry.addData("VSlidepos  ", outtakeSystem.getVSlidePos());
        telemetry.addData("power", driveTrain.pidControl.update(driveTrain.leftFront.getCurrentPosition()));


        //HSlide
//        testValue += 3 * (gamepad1.right_trigger - gamepad1.left_trigger);
//        intakeSystem.setHSlidePos((int) testValue);
//        telemetry.addData("targetPos1", (int) testValue);
//        telemetry.addData("targetPos2", intakeSystem.getHSlideTargetPos());
//        telemetry.addData("H Pos", intakeSystem.getHSlidePos());


        //VSlide
        outtakeSystem.setVSlidePos((int) testValue2);
//        telemetry.addData("targetPos", (int) testValue2);
//        telemetry.addData(" Pos", outtakeSystem.getVSlidePos());
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
//        telemetry.addData("claw", outtakeSystem.outtakeArm.getClawPos());

        /*
        PS5
        telemetry.addData("1", gamepad2.touchpad_finger_1);
        telemetry.addData("x", Math.round(gamepad2.touchpad_finger_1_x*100.0)/100.0);
        telemetry.addData("y", Math.round(gamepad2.touchpad_finger_1_y*100.0)/100.0);
        telemetry.addData("2", gamepad2.touchpad_finger_2);
        telemetry.addData("x", Math.round(gamepad2.touchpad_finger_2_x*100.0)/100.0);
        telemetry.addData("y", Math.round(gamepad2.touchpad_finger_2_y*100.0)/100.0);
        telemetry.addData("touchpad", gamepad2.touchpad);
        telemetry.addData("options", gamepad2.options);
        telemetry.addData("ps", gamepad2.ps);
        telemetry.addData("guide", gamepad2.guide);
        telemetry.addData("id", gamepad2.id);
        telemetry.addData("share", gamepad2.share);
        telemetry.addData("timestamp", gamepad2.timestamp);
        telemetry.addData("type", gamepad2.type);
        telemetry.addData("atRest", gamepad2.atRest());
         */

        //limeLight


        intakeSystem.update();
        outtakeSystem.update();
        telemetry.update();
    }

    public void stop() {
        driveTrain.stopDrive();
    }
}
