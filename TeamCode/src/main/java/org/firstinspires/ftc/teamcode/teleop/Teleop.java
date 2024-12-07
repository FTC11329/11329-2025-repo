package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    ElapsedTime elapsedTime = new ElapsedTime();
    //INPUTS
    double driveForward;
    double driveStrafe;
    double driveRotation;

    boolean PTOEnable;
    boolean PTODisable;
    boolean PTOClimb;
    boolean PTODrop;
    boolean winchIn;
    boolean winchOut;

    boolean intakeExtendMin = false;
    boolean intakeExtend = false;
    boolean intakeExtendMax = false;
    boolean intakeSpit;
    boolean unjamIntake;

    boolean autoIntakeColor;
    boolean autoIntake;
    boolean autoIntakeCancel;

    double manualHSlidePower;
    double manualVSlidePower;
    double manualArmPower;

    boolean transfer;

    boolean specimenOffWall;
    boolean highBasket;
    boolean highSpecimen;

    boolean endArmSpecimen;
    boolean dropClaw;
    boolean grabClaw;

    boolean resetState;



    //Various Variables
    DriveSpeedEnum driveSpeed;
    boolean intakeingColor = false;
    boolean intakeing = false;
    boolean extended = false;
    boolean transferred = false;
    boolean transferring = false;
    boolean unjammingIntake;
    boolean hasSample = false;
    boolean hasSpecimen = false;
    boolean spitDebounce = false;
    boolean dropDebounce = false;
    boolean grabDebounce = false;
    boolean endArmDebounce = false;
    boolean unjamIntakeDebounce = false;
    boolean goingToWall = false;

    double goingToWallTime = 2000000000;
    double transferTime = 2000000000;
    double unjamTime = 2000000000;

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
        driveForward = -gamepad1.left_stick_y; //1
        driveStrafe = -gamepad1.left_stick_x; //1
        driveRotation = -gamepad1.right_stick_x; //1
        if (gamepad1.right_bumper) { //1
            driveSpeed = DriveSpeedEnum.Fast;
        } else {
            driveSpeed = DriveSpeedEnum.Slow;
        }

        PTOEnable = gamepad2.back; //1
        PTODisable = gamepad1.back; //2
        PTOClimb = powerTakeOff.isEnabled() && gamepad2.dpad_up; //2
        PTODrop = powerTakeOff.isEnabled() && gamepad2.dpad_down; //2

        winchIn = gamepad2.right_stick_button;

//        intakeExtendMin = !hasSpecimen && !hasSample && gamepad2.dpad_left; //2
//        intakeExtend = !hasSpecimen && !hasSample && !powerTakeOff.isEnabled() && gamepad2.dpad_down; //2
//        intakeExtendMax = !hasSpecimen && !hasSample && gamepad2.dpad_right; //2
        intakeSpit = (hasSample && !transferred && gamepad1.left_bumper) || gamepad2.left_bumper; //1
        unjamIntake = gamepad2.b; //1 2

        autoIntakeColor = gamepad2.x; //1
        autoIntake = gamepad2.a; //1
        autoIntakeCancel = gamepad2.y; //2

        manualHSlidePower = gamepad2.right_trigger - gamepad2.left_trigger; //2
        manualVSlidePower = -gamepad2.left_stick_y; //2
        manualArmPower = -gamepad2.right_stick_y; //2

        transfer = gamepad1.a || (!hasSpecimen && !powerTakeOff.isEnabled() && gamepad2.dpad_up); //1 2

        specimenOffWall = !hasSpecimen && !hasSample && !powerTakeOff.isEnabled() && (gamepad1.dpad_up); //1 2
        highBasket = hasSample && !powerTakeOff.isEnabled() && (gamepad1.dpad_up); //1 2
//        highBasket
//        lowSpecimen;
        highSpecimen = hasSpecimen && !powerTakeOff.isEnabled() && (gamepad1.dpad_up || gamepad2.dpad_up); //1 2

        endArmSpecimen = hasSpecimen && (gamepad1.left_bumper || gamepad2.right_bumper); //1
        dropClaw = hasSample && transferred && (gamepad1.left_bumper || gamepad2.right_bumper); //1
        grabClaw = !hasSpecimen && !hasSample && !transferred && (gamepad1.left_bumper || gamepad2.right_bumper); //1

        resetState = gamepad1.b;
        if (resetState) {
            hasSample = false;
            hasSpecimen = false;
            transferred = false;
        }

        //DRIVING
        if (!powerTakeOff.isEnabled()) {
            //Regular time
            driveTrain.drive(driveForward, driveStrafe, driveRotation, driveSpeed);
        } else {
            if (PTOClimb) {
                driveTrain.setPTOPos(Constants.PTO.motorClimb);
            } else if (PTODrop) {
                driveTrain.setPTOPos(Constants.PTO.motorDrop);
            }
            if (PTODisable) {
                powerTakeOff.disable();
            }

            //PTO Time
            if (Math.abs(driveTrain.getPTOPos() - driveTrain.getPTOTPos()) > 30) {
                driveTrain.PTOLoop();
            } else {
                driveTrain.PTOLoopLight();
            }

            if (winchIn) {
                climber.setPos(Constants.Climber.inPos);
            }
        }

        //PTO Enabling
        if (PTOEnable) {
            powerTakeOff.enable();
            driveTrain.setRunToPos();
            climber.setPos(Constants.Climber.outPos);
            outtakeSystem.setArmPos(Constants.Outtake.intakeWallArm);
            outtakeSystem.setVSlidePos(Constants.Outtake.intakeWallSlides);
        }

        if (intakeExtendMin) {
            intakeSystem.pickupPos(Constants.Intake.minWhileDownPos);
        }
        if (intakeExtend) {
            intakeSystem.pickupPos();
        }
        if (intakeExtendMax) {
            intakeSystem.pickupPos(Constants.Intake.maxSlidePos);
        }



        //Intake
        if (autoIntakeCancel) {
            intakeing = false;
            intakeingColor = false;
            unjammingIntake = false;
            intakeSystem.storePos();
            outtakeSystem.setVSlidePos(0);
            outtakeSystem.setArmPos(Constants.Outtake.upArm);
            outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
        }
        if (autoIntakeColor) {
            intakeSystem.setHSlidePos(Constants.Intake.intakeSlidePos);
            intakeing = false;
            intakeingColor = true;
        }
        if (autoIntake) {
            intakeSystem.setHSlidePos(Constants.Intake.intakeSlidePos);
            intakeingColor = false;
            intakeing = true;
        }
        //unjam
        if (unjamIntake && !unjamIntakeDebounce)  {
            unjammingIntake = true;
            unjamIntakeDebounce = true;
            unjamTime = elapsedTime.milliseconds();
        }
        if (!unjamIntake) {
            unjamIntakeDebounce = false;
        }

        //intaking loop
        if (intakeing && !unjammingIntake) {
            if (!hasSpecimen) {
                if (intakeSystem.intakeUntil()) {
                    intakeSystem.storePos();
//                    transferring = true;
//                    transferTime = elapsedTime.milliseconds() + 1000;
                    hasSample = true;
                    intakeing = false;
                }
            }
        }
        if (intakeingColor && !unjammingIntake) {
            if (!hasSpecimen) {
                if (intakeSystem.intakeUntilColor()) {
                    intakeSystem.storePos();
                    hasSample = true;
                    intakeingColor = false;
                }
            }
        }
        if (intakeingColor && !autoIntakeColor) {
            intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
        }
        if (intakeing && !autoIntake) {
            intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
        }
        //unjamming
        if (unjammingIntake) {
            if (elapsedTime.milliseconds() < unjamTime + 50) {
                intakeSystem.setIntakePower(Constants.Intake.unjamSpeed);
            }
            if (elapsedTime.milliseconds() > unjamTime + 250) {
                if (intakeSystem.intakeUntilColor()) {
                    unjammingIntake = false;
                    unjamTime = 2100000000;
                }
            }
        }
        //Spit
        if (intakeSpit && !spitDebounce) {
            intakeSystem.setIntakePower(Constants.Intake.spitSpeed);
            intakeSystem.setIntakeServoPos(Constants.Intake.wristSpit);
            spitDebounce = true;
        }
        if (spitDebounce && !intakeSpit) {
            intakeSystem.setIntakePower(0);
            intakeSystem.setIntakeServoPos(Constants.Intake.wristStore);
            goingToWallTime = elapsedTime.milliseconds();
            goingToWall = true;
            hasSample = false;
            hasSpecimen = false;
            spitDebounce = false;
        }

        //Presets
        if (transfer) {
            transferTime = elapsedTime.milliseconds();
            transferring = true;
        }
        //Store and transfer
        if (transferring) {
            if (elapsedTime.milliseconds() < transferTime + 50) {
                outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                outtakeSystem.setVSlidePos(Constants.Outtake.safeFromHSlides);
                intakeSystem.setIntakeServoPos(Constants.Intake.wristStore);
                intakeSystem.setIntakePower(Constants.Intake.transferSpeed);
            }
            if (elapsedTime.milliseconds() > transferTime + 400 && elapsedTime.milliseconds() < transferTime + 450) {
                outtakeSystem.setArmPos(Constants.Outtake.intakeArm);
            }
            if (elapsedTime.milliseconds() > transferTime + 700 && elapsedTime.milliseconds() < transferTime + 750) {
                intakeSystem.storePos();
                outtakeSystem.setVSlidePos(Constants.Outtake.intakeSlides);
                intakeSystem.setIntakePower(0);
                intakeSystem.setIntakeServoPos(Constants.Intake.wristClear);
            }
            if (elapsedTime.milliseconds() > transferTime + 1100 && elapsedTime.milliseconds() < transferTime + 1150) {
                outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
            }
            if (elapsedTime.milliseconds() > transferTime + 1400 && elapsedTime.milliseconds() < transferTime + 1450) {
                outtakeSystem.setVSlidePos(Constants.Outtake.highBasketSlides);
                outtakeSystem.setArmPos(Constants.Outtake.basketArm);
                transferred = true;
            }
            if (elapsedTime.milliseconds() > transferTime + 1800 && elapsedTime.milliseconds() < transferTime + 1850) {
                intakeSystem.storePos();
                transferTime = 2000000000;
                transferring = false;
            }
        }

        if (specimenOffWall) {
            goingToWallTime = elapsedTime.milliseconds();
            goingToWall = true;
            hasSpecimen = false;
        }
        if (highSpecimen) {
            outtakeSystem.placePos(PlacePosEnum.highSpecimen);
        }
        if (highBasket) {
            outtakeSystem.placePos(PlacePosEnum.highBasket);
        }
        if (goingToWall) {
            if (elapsedTime.milliseconds() < goingToWallTime + 50) {
                outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                outtakeSystem.setVSlidePos(Constants.Outtake.intakeWallSlides);
            }
            if (elapsedTime.milliseconds() > goingToWallTime + 200 && elapsedTime.milliseconds() < goingToWallTime + 300) {
                outtakeSystem.setArmPos(Constants.Outtake.intakeWallArm);
            }
            if (elapsedTime.milliseconds() > goingToWallTime + 500 && elapsedTime.milliseconds() < goingToWallTime + 550) {
                outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                goingToWallTime = 2000000000;
                goingToWall = false;
            }
        }
        //Manual
        intakeSystem.manualHSlide(manualHSlidePower);
        outtakeSystem.manualVSlide(manualVSlidePower);
        if (gamepad1.right_stick_button) {
            outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
            outtakeSystem.manualArm(manualArmPower);
        }

        //drop into basket
        if (dropClaw && !dropDebounce) {
            outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
            dropDebounce = true;
        }
        if (dropDebounce && !dropClaw) {
            outtakeSystem.setArmPos(Constants.Outtake.upArm);
            outtakeSystem.setVSlidePos(0);
            hasSample = false;
            transferred = false;
            dropDebounce = false;
        }

        //grab from wall
        if (grabClaw && !grabDebounce) {
            outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
            grabDebounce = true;
        }
        if (grabDebounce && !grabClaw) {
            outtakeSystem.setVSlidePos(Constants.Outtake.safeFromWallSlides);
            hasSpecimen = true;
            grabDebounce = false;
        }
        //drop specimen
        if (endArmSpecimen && !endArmDebounce) {
            outtakeSystem.setArmPos(Constants.Outtake.specimenArmEnd);
            endArmDebounce = true;
        }
        if (endArmDebounce && !endArmSpecimen) {
            outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
            outtakeSystem.setArmPos(Constants.Outtake.upArm);
            endArmDebounce = false;
            hasSpecimen = false;
        }

        outtakeSystem.update();
        telemetry.addData("sample", hasSample);
        telemetry.addData("specimen", hasSpecimen);
        telemetry.addData("transferred", transferred);
        telemetry.addData("","");
        telemetry.addData("Vslidepos", outtakeSystem.getVSlidePos());
        telemetry.addData("VslideTpos", outtakeSystem.getVSlideTargetPos());
        telemetry.addData("Hslidepos", intakeSystem.getHSlidePos());
        telemetry.addData("HslideTpos", intakeSystem.getHSlideTargetPos());
        telemetry.addData("","");
        telemetry.addData("intakeing", intakeing);
        telemetry.addData("unjammingIntake", unjammingIntake);
        telemetry.addData("intakeingColor", intakeingColor);
        telemetry.addData("goingToWall", goingToWall);
        telemetry.addData("","");
        telemetry.addData("color", intakeSystem.color());
        telemetry.addData("distance", intakeSystem.distance() < 1.5);
        telemetry.addData("intakePos", intakeSystem.intakeClaw.getIntakeServoPos());
        telemetry.addData("outtakePos", outtakeSystem.outtakeArm.getClawPos());
        telemetry.addData("Climber Position", climber.getPos());
        telemetry.addData("power", driveTrain.pidControl.update(driveTrain.leftFront.getCurrentPosition()));
        telemetry.update();
    }

    public void stop() {
        driveTrain.stopDrive();
    }
}
