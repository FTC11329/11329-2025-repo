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

    boolean intakeExtendMin;
    boolean intakeExtend;
    boolean intakeExtendMax;
    boolean intakeSpit;
    boolean unjamIntake;

    boolean autoIntakeColor;
    boolean autoIntake;
    boolean autoIntakeCancel;

    double manualHSlidePower;
    double manualVSlidePower;

    boolean storePos;
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
    boolean goingToStorePos = false;
    boolean unjammingIntake;
    boolean hasSample = false;
    boolean hasSpecimen = false;
    boolean spitDebounce = false;
    boolean dropDebounce = false;
    boolean grabDebounce = false;
    boolean endArmDebounce = false;
    boolean unjamIntakeDebounce = false;

    double storeTime = 2000000000;

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

        PTOEnable = gamepad1.back; //1
        PTODisable = gamepad2.back; //2
        PTOClimb = powerTakeOff.isEnabled() && gamepad2.dpad_up; //2
        PTODrop = powerTakeOff.isEnabled() && gamepad2.dpad_down; //2

        intakeExtendMin = !hasSpecimen && !hasSample && !powerTakeOff.isEnabled() && gamepad2.dpad_up; //2
        intakeExtend = !hasSpecimen && !hasSample && gamepad2.dpad_right; //2
        intakeExtendMax = !hasSpecimen && !hasSample && !powerTakeOff.isEnabled() && gamepad2.dpad_down; //2
        intakeSpit = hasSample && !transferred && gamepad1.left_bumper; //1
        unjamIntake = gamepad2.x;

        autoIntakeColor = gamepad1.x; //1
        autoIntake = gamepad1.y; //1
        autoIntakeCancel = gamepad2.b; //2

        if (!hasSample && !hasSpecimen) {
            manualHSlidePower = gamepad2.right_trigger - gamepad2.left_trigger; //2
            manualVSlidePower = 0;
        }
        if (hasSpecimen || hasSample) {
            manualHSlidePower = 0;
            manualVSlidePower = gamepad2.right_trigger - gamepad2.left_trigger; //2
        }

        storePos = gamepad2.dpad_left;
        specimenOffWall = gamepad1.dpad_left; //1 2
        highBasket = hasSpecimen && gamepad1.dpad_left; //1
//        highBasket
//        lowSpecimen;
        highSpecimen = hasSpecimen && gamepad1.dpad_right; //1

        endArmSpecimen = hasSpecimen && gamepad1.left_bumper; //1
        dropClaw = hasSample && transferred && gamepad1.left_bumper;
        grabClaw = !hasSpecimen && !hasSample && gamepad1.left_bumper; //1

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
            driveTrain.PTOLoop();

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
            intakeSystem.unjamP2 = false;
            unjammingIntake = true;
            unjamIntakeDebounce = true;
        }
        if (!unjamIntake) {
            unjamIntakeDebounce = false;
        }

        //intaking loop
        if (intakeing && !unjammingIntake) {
            intakeSystem.pickupPos();
            if (intakeSystem.intakeUntil()) {
                intakeSystem.storeOutPos();
                hasSample = true;
                intakeing = false;
            }
        }
        if (intakeingColor && !unjammingIntake) {
            intakeSystem.pickupPos();
            if (intakeSystem.intakeUntilColor()) {
                intakeSystem.storeOutPos();
                hasSample = true;
                intakeingColor = false;
            }
        }
        //unjamming
        if (unjammingIntake) {
            if (intakeSystem.unjam()) {
                unjammingIntake = false;
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
        if (storePos) {
            storeTime = elapsedTime.milliseconds() ;
            goingToStorePos = true;
        }
        //Store and transfer
        if (goingToStorePos) {
            if (elapsedTime.milliseconds() < storeTime + 50) {
                outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                outtakeSystem.setVSlidePos(Constants.Outtake.intakeSlides + 100);
                intakeSystem.storeOutPos();
            }
            if (elapsedTime.milliseconds() > storeTime + 1000 && elapsedTime.milliseconds() < storeTime + 1050) {
                outtakeSystem.setArmPos(Constants.Outtake.intakeArm);
            }
            if (elapsedTime.milliseconds() > storeTime + 2000 && elapsedTime.milliseconds() < storeTime + 2050) {
                intakeSystem.storePos();
                outtakeSystem.setVSlidePos(Constants.Outtake.intakeSlides);
            }
            if (elapsedTime.milliseconds() > storeTime + 3000 && elapsedTime.milliseconds() < storeTime + 3050) {
                outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
            }
            if (elapsedTime.milliseconds() > storeTime + 4000 && elapsedTime.milliseconds() < storeTime + 4050) {
                intakeSystem.setIntakePower(Constants.Intake.transferSpeed);
                intakeSystem.setIntakeServoPos(Constants.Intake.wristClear);
            }
            if (elapsedTime.milliseconds() > storeTime + 5000) {
                intakeSystem.setIntakePower(0);
                goingToStorePos = false;
                transferred = true;
            }
        }

        if (specimenOffWall) {
            outtakeSystem.setArmPos(Constants.Outtake.intakeWallArm);
            outtakeSystem.setVSlidePos(Constants.Outtake.intakeWallSlides);
        }
        if (highSpecimen) {
            outtakeSystem.placePos(PlacePosEnum.highSpecimen);
        }
        if (highBasket) {
            outtakeSystem.placePos(PlacePosEnum.highBasket);
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
