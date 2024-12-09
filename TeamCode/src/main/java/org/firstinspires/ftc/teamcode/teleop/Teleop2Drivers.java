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

public class Teleop2Drivers {

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

    boolean intakeExtendMin;
    boolean intakeExtend;
    boolean intakeExtendMax;
    boolean intakeSpit;
    boolean unjamIntake;

    boolean intakeOuttake;

    boolean autoIntakeColor;
    boolean autoIntake;
    boolean autoIntakeCancel;

    double manualHSlidePower;
    double manualVSlidePower;

    boolean transfer;

    boolean specimenOffWall;
    boolean highBasket;
    boolean lowBasket;
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
    boolean unjammingIntake = false;
    boolean intakeOuttakeOnce = false;
    boolean hasSample = false;
    boolean hasSpecimen = false;
    boolean spitDebounce = false;
    boolean dropDebounce = false;
    boolean grabDebounce = false;
    boolean endArmDebounce = false;
    boolean unjamIntakeDebounce = false;
    boolean goingToWall = false;

    int PTOError;
    double goingToWallTime = 2000000000;
    double transferTime = 2000000000;
    double unjamTime = 2000000000;

    //this is here because I have to have a teleop blue and teleop red
    HardwareMap hardwareMap;
    Telemetry telemetry;
    Gamepad gamepad1;
    Gamepad gamepad2;
    RobotSideEnum robotSide;

    public Teleop2Drivers(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, RobotSideEnum robotSide) {
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

        PTOEnable = gamepad1.back; //1
        PTODisable = gamepad2.back; //2
        PTOClimb = powerTakeOff.isEnabled() && gamepad2.dpad_up; //2
        PTODrop = powerTakeOff.isEnabled() && gamepad2.dpad_down; //2

        intakeExtendMin = !hasSpecimen && !hasSample && !powerTakeOff.isEnabled() && gamepad2.a; //2
        intakeExtend = !hasSpecimen && !hasSample && gamepad2.b; //2
        intakeExtendMax = !hasSpecimen && !hasSample && !powerTakeOff.isEnabled() && gamepad2.y; //2
        intakeSpit = (hasSample && !transferred && gamepad1.left_bumper); //1
        unjamIntake = gamepad1.b; //1 2
        intakeOuttake = gamepad2.left_bumper;

        autoIntakeColor = gamepad1.x; //1
        autoIntake = gamepad1.y; //1
        autoIntakeCancel = gamepad2.x; //2

        manualHSlidePower = gamepad2.right_trigger - gamepad2.left_trigger; //2
        manualVSlidePower = 0;

        transfer = gamepad1.a; //1

        specimenOffWall = gamepad2.dpad_right; //1 2
        highBasket = hasSample && transferred && gamepad2.dpad_up; //1 2
        lowBasket = hasSample && transferred && gamepad2.dpad_down;
        highSpecimen = hasSpecimen && gamepad2.dpad_up; //1 2

        endArmSpecimen = hasSpecimen && gamepad2.right_bumper; //1
        dropClaw = hasSample && transferred && gamepad2.right_bumper; //1
        grabClaw = !hasSpecimen && !hasSample && gamepad2.right_bumper; //1

        resetState = gamepad2.a;
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
            //PTO Time
            PTOError = Math.abs(driveTrain.getPTOPos() - driveTrain.getPTOTPos());
            driveTrain.PTOLoop(Math.min(0.25, Math.max( ( (PTOError - 60) / 500), 0 )));

            if (PTOClimb) {
                driveTrain.setPTOPos(Constants.PTO.motorClimb);
            } else if (PTODrop) {
                driveTrain.setPTOPos(Constants.PTO.motorDrop);
            }
            if (PTODisable) {
                powerTakeOff.disable();
            }
        }

        //PTO Enabling
        if (PTOEnable) {
            powerTakeOff.enable();
            driveTrain.setRunToPos();
        }
        if (intakeExtendMin) {
            intakeSystem.pickupPosWithTime(Constants.Intake.minWhileDownPos);
        }
        if (intakeExtend) {
            intakeSystem.pickupPosWithTime();
        }
        if (intakeExtendMax) {
            intakeSystem.pickupPosWithTime(Constants.Intake.maxSlidePos);
        }

        //Intake
        if (autoIntakeCancel) {
            intakeing = false;
            intakeingColor = false;
            intakeSystem.storePos();
        }
        if (autoIntakeColor) {
            intakeing = false;
            intakeingColor = true;
        }
        if (autoIntake) {
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
        if (intakeOuttake) {
            intakeOuttakeOnce = true;
            intakeSystem.setIntakePower(Constants.Intake.spitSpeed);
        }
        if (!intakeOuttake && intakeOuttakeOnce) {
            intakeOuttakeOnce = false;
            intakeSystem.setIntakePower(0);
        }

        //intaking loop
        if (intakeing && !unjammingIntake) {
            intakeSystem.pickupPosWithTime();
            if (intakeSystem.intakeUntil()) {
                intakeSystem.storeOutPos();
                hasSample = true;
                intakeing = false;
            }
        }
        if (intakeingColor && !unjammingIntake) {
            intakeSystem.pickupPosWithTime();
            if (intakeSystem.intakeUntilColor()) {
                intakeSystem.storeOutPos();
                hasSample = true;
                intakeingColor = false;
            }
        }
        //unjamming
        if (unjammingIntake) {
            if (elapsedTime.milliseconds() < unjamTime + 50) {
                intakeSystem.setIntakePower(Constants.Intake.unjamSpeed);
            }
            if (elapsedTime.milliseconds() > unjamTime + 250) {
                if (intakeSystem.intakeUntilColor()) {
                    unjammingIntake = false;
                    unjamTime = 2000000000;
                }
            }
        }
        //Spit
        if (intakeSpit && !spitDebounce) {
            intakeSystem.setIntakePower(Constants.Intake.spitSpeed);
            spitDebounce = true;
        }
        if (spitDebounce && !intakeSpit) {
            intakeSystem.setIntakePower(0);
            outtakeSystem.setArmPos(Constants.Outtake.intakeWallArm);
            outtakeSystem.setVSlidePos(Constants.Outtake.intakeWallSlides);
            hasSample = false;
            spitDebounce = false;
        }

        intakeSystem.manualHSlide(manualHSlidePower);
        outtakeSystem.manualVSlide(manualVSlidePower);
        //Presets
        if (transfer) {
            transferTime = elapsedTime.milliseconds();
            transferring = true;
        }
        //Store and transfer
        if (transferring) {
            if (elapsedTime.milliseconds() < transferTime + 50) {
                outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                outtakeSystem.setVSlidePos(Constants.Outtake.intakeSlides + 100);
                intakeSystem.storeOutPos();
            }
            if (elapsedTime.milliseconds() > transferTime + 500 && elapsedTime.milliseconds() < transferTime + 550) {
                outtakeSystem.setArmPos(Constants.Outtake.intakeArm);
            }
            if (elapsedTime.milliseconds() > transferTime + 1000 && elapsedTime.milliseconds() < transferTime + 1050) {
                intakeSystem.storePos();
                intakeSystem.setIntakePower(Constants.Intake.transferSpeed);
                outtakeSystem.setVSlidePos(Constants.Outtake.intakeSlides);
            }
            if (elapsedTime.milliseconds() > transferTime + 1500 && elapsedTime.milliseconds() < transferTime + 1550) {
                outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
            }
            if (elapsedTime.milliseconds() > transferTime + 1750 && elapsedTime.milliseconds() < transferTime + 1800) {
                intakeSystem.setIntakeServoPos(Constants.Intake.wristClear);
            }
            if (elapsedTime.milliseconds() > transferTime + 2500 && elapsedTime.milliseconds() < transferTime + 2550) {
                intakeSystem.setIntakePower(0);
                outtakeSystem.setVSlidePos(Constants.Outtake.safeFromHSlides);
            }
            if (elapsedTime.milliseconds() > transferTime + 2500 && elapsedTime.milliseconds() < transferTime + 2550) {
                outtakeSystem.setArmPos(Constants.Outtake.initArm);
            }
            if (elapsedTime.milliseconds() > transferTime + 3000 && elapsedTime.milliseconds() < transferTime + 3050) {
                outtakeSystem.setVSlidePos(Constants.Outtake.intakeSlides);
                intakeSystem.storePos();
                transferTime = 2000000000;
                transferring = false;
                transferred = true;
            }
        }

        if (specimenOffWall) {
            goingToWall = true;
            goingToWallTime = elapsedTime.milliseconds();
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
            if (elapsedTime.milliseconds() > goingToWallTime + 200 && elapsedTime.milliseconds() < goingToWallTime + 250) {
                outtakeSystem.setArmPos(Constants.Outtake.intakeWallArm);
            }
            if (elapsedTime.milliseconds() > goingToWallTime + 500 && elapsedTime.milliseconds() < goingToWallTime + 550) {
                outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                goingToWallTime = 2000000000;
                goingToWall = false;
            }
        }

        //drop into basket
        if (dropClaw && !dropDebounce) {
            outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
            dropDebounce = true;
        }
        if (dropDebounce && !dropClaw) {
            hasSample = false;
            transferred = false;
            dropDebounce = false;
            outtakeSystem.storePos();
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
        }
        if (endArmDebounce && !endArmSpecimen) {
            outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
            hasSpecimen = false;
        }

        outtakeSystem.update();
        telemetry.addData("sample", hasSample);
        telemetry.addData("specimen", hasSpecimen);
        telemetry.addData("transferred", transferred);
        telemetry.addData("Vslidepos", outtakeSystem.getVSlidePos());
        telemetry.addData("VslideTpos", outtakeSystem.getVSlideTargetPos());
        telemetry.addData("Hslidepos", intakeSystem.getHSlidePos());
        telemetry.addData("HslideTpos", intakeSystem.getHSlideTargetPos());
        telemetry.addData("intakeing", intakeing);
        telemetry.addData("unjammingIntake", unjammingIntake);
        telemetry.addData("intakeingColor", intakeingColor);
        telemetry.update();
    }

    public void stop() {
        driveTrain.stopDrive();
    }
}
