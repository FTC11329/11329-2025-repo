package org.firstinspires.ftc.teamcode.teleop;

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
import org.firstinspires.ftc.teamcode.utility.PlacePosEnum;
import org.firstinspires.ftc.teamcode.utility.PoseFunctions;
import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

public class EnhancedTeleop {

    Climber climber;
    Follower follower;
    Drivetrain driveTrain;
    BlockVision blockVision;
    PowerTakeOff powerTakeOff;
    IntakeSystem intakeSystem;
    OuttakeSystem outtakeSystem;

    PIDFController headingPIDF = new PIDFController(FollowerConstants.headingPIDFCoefficients);

    ElapsedTime elapsedTime = new ElapsedTime();
    //INPUTS
    double driveForward;
    double driveStrafe;
    double driveRotation;
    double targetHeading = -90; //in radians

    boolean PTOEnable;
    boolean PTODisable;
    boolean PTOClimb;
    boolean PTODrop;
    boolean winchOut;

    boolean intakeExtendMin = false;
    boolean intakeExtend = false;
    boolean intakeExtendMax = false;
    boolean intakeSpit;
    boolean unjamIntake;
    boolean spitHard = false;

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
    boolean spitHardDebounce = false;
    boolean goingToWall = false;
    boolean autoDriving = false;
    boolean autoIntakeDebounce = false;
    boolean climbInit = false;
    boolean climbL1P1 = false;
    boolean climbL2P1 = false;
    boolean climbL2P2 = false;

    int climberPos = 0;
    int PTOError = 0;

    double goingToWallTime = 2000000000;
    double transferTime = 2000000000;
    double unjamTime = 2000000000;
    double placeSpecimenTime = 2000000000;
    double autoWristTime = 2000000000;
    double spitTime = 2000000000;

    Pose currentPose = new Pose(0,0,0);

    //this is here because I have to have a teleop blue and teleop red
    HardwareMap hardwareMap;
    Telemetry telemetry;
    Gamepad gamepad1;
    Gamepad gamepad2;
    RobotSideEnum robotSide;


    public EnhancedTeleop(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, RobotSideEnum robotSide) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.robotSide = robotSide;
    }

    public void init() {
        climber = new Climber(hardwareMap);
        follower = new Follower(hardwareMap);
        driveTrain = new Drivetrain(hardwareMap);
        powerTakeOff = new PowerTakeOff(hardwareMap);

        follower.setStartingPose(new Pose(48,48,90));
    }

    public void start() {
        intakeSystem = new IntakeSystem(hardwareMap, robotSide);
        outtakeSystem = new OuttakeSystem(hardwareMap);
        follower.startTeleopDrive();
    }

    public void loop() {
        currentPose = follower.getPose();

        //INPUTS
        driveForward = -gamepad1.left_stick_y; //1
        if (autoDriving) {
            driveForward = -1;
            if (Math.abs(gamepad1.left_stick_y) > 0.1) {
                autoDriving = false;
            }
        }
        driveStrafe = -gamepad1.left_stick_x; //1

        switch (PoseFunctions.getLocation(currentPose)) {
            case leftSideSub:
                targetHeading = Math.toRadians(0);
                break;
            case basket:
                targetHeading = Math.toRadians(45);
                break;
            case frontSub:
                targetHeading = Math.toRadians(90);
                break;
            case observation:
                targetHeading = Math.toRadians(-45);
                break;
            case rightSideSub:
                targetHeading = Math.toRadians(180);
                break;
        }
        headingPIDF.updateError(targetHeading - currentPose.getHeading());
        driveRotation = headingPIDF.runPIDF();

        if (gamepad1.right_bumper) { //1
            driveSpeed = DriveSpeedEnum.Fast;
        } else {
            driveSpeed = DriveSpeedEnum.Slow;
        }

        PTOEnable = gamepad1.back; //1
        PTODisable = gamepad2.back; //2
        PTOClimb = powerTakeOff.isEnabled() && gamepad2.dpad_up; //2
        PTODrop = powerTakeOff.isEnabled() && gamepad2.dpad_down; //2

//        intakeExtendMin = !hasSpecimen && !hasSample && gamepad2.dpad_left; //2
//        intakeExtend = !hasSpecimen && !hasSample && !powerTakeOff.isEnabled() && gamepad2.dpad_down; //2
//        intakeExtendMax = !hasSpecimen && !hasSample && gamepad2.dpad_right; //2
        intakeSpit = (hasSample && !transferred && gamepad1.left_bumper) || gamepad2.left_bumper; //1
        unjamIntake = gamepad2.b; //1 2
//        spitHard = gamepad1.left_stick_button;

        autoIntakeColor = gamepad2.x; //1
        autoIntake = gamepad2.a; //1
        autoIntakeCancel = gamepad2.y; //2
        if (!powerTakeOff.isEnabled()) {
            manualHSlidePower = gamepad2.right_trigger - gamepad2.left_trigger; //2
        }
        manualVSlidePower = -gamepad2.left_stick_y; //2
        manualArmPower = -gamepad2.right_stick_y; //2

        transfer = !hasSpecimen && !powerTakeOff.isEnabled() && gamepad2.dpad_up; //1 2

        specimenOffWall = !hasSpecimen && !hasSample && !powerTakeOff.isEnabled() && (gamepad1.dpad_up); //1 2
        highBasket = hasSample && !powerTakeOff.isEnabled() && (gamepad1.dpad_up); //1 2
//        highBasket
//        lowSpecimen;
        highSpecimen = hasSpecimen && !powerTakeOff.isEnabled() && (gamepad1.dpad_up || gamepad2.dpad_up); //1 2

        endArmSpecimen = hasSpecimen && gamepad2.right_bumper; //1
        dropClaw = transferred && gamepad2.right_bumper; //1
        grabClaw = !hasSpecimen && !hasSample && !transferred && (gamepad2.right_bumper); //1

        resetState = gamepad1.b;

        if (resetState) {
            hasSample = false;
            hasSpecimen = false;
            transferred = false;
        }

        //DRIVING
        if (!powerTakeOff.isEnabled()) {
            //Regular time
            follower.setTeleOpMovementVectors(driveForward, driveStrafe, driveRotation, false);
        } else {
            //Climbing
            if (PTODisable) {
                powerTakeOff.disable();
                climbInit = false;
                climbL1P1 = false;
                climbL2P1 = false;
                climbL2P2 = false;
            }
            //big auto movement
            if (!climbInit) {
                climbInit = true;
                climberPos = Constants.Climber.outPos;
                climbL1P1 = true;
            }
            if (climbL1P1 && climber.getPos() > climberPos - 100 && gamepad1.a) {
                climbL1P1 = false;
                driveTrain.setPTOPos(Constants.PTO.motorClimb);
                climbL2P1 = true;
            }
            if (climbL2P1 && driveTrain.getPTOPos() > driveTrain.getPTOTPos() - 50) {
                climbL2P1 = false;
                climberPos = Constants.Climber.inPos;
                climbL2P2 = true;
            }
            if (climbL2P2 && climber.getPos() < climberPos + 5000) {
                climbL2P2 = false;
                driveTrain.setPTOPos(Constants.PTO.motorDrop);
            }

            //fancy math for PTO feedforward
            PTOError = Math.abs(driveTrain.getPTOPos() - driveTrain.getPTOTPos());
            if (PTOError > 50) {
                driveTrain.PTOLoop(0.5);
            } else {
                driveTrain.PTOLoop(0);
            }
//            driveTrain.PTOLoop(Math.min(0.25, Math.max( ( (PTOError - 60) / 500), 0 )));

            climberPos += (int) (20 * (gamepad1.right_trigger - gamepad1.left_trigger));
            climber.setPos(climberPos);

        }
        if (gamepad1.a) {
            climberPos = Constants.Climber.outPos;
            climber.setPos(climberPos);
        }

        //PTO Enabling
        if (PTOEnable) {
            powerTakeOff.enable();
            driveTrain.setRunToPos();
            outtakeSystem.setArmPos(Constants.Outtake.intakeWallArm);
            outtakeSystem.setVSlidePos(Constants.Outtake.intakeWallSlides);
        }

        //intake presets
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
        if (autoIntakeColor && !autoIntakeDebounce) {
            autoWristTime = elapsedTime.milliseconds();
            intakeSystem.setHSlidePos(Constants.Intake.intakeSlidePos);
            autoIntakeDebounce = true;
            intakeing = false;
            intakeingColor = true;
        }
        if (autoIntake && !autoIntakeDebounce) {
            autoWristTime = elapsedTime.milliseconds();
            intakeSystem.setHSlidePos(Constants.Intake.intakeSlidePos);
            autoIntakeDebounce = true;
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
        if (intakeingColor && !autoIntakeColor && elapsedTime.milliseconds() > autoWristTime + 300) {
            autoIntakeDebounce = false;
            intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
            autoWristTime = 2000000000;
        }
        if (intakeing && !autoIntake && elapsedTime.milliseconds() > autoWristTime + 300) {
            autoIntakeDebounce = false;
            intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
            autoWristTime = 2000000000;
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
            spitTime = elapsedTime.milliseconds();
            intakeSystem.setHSlidePos(Constants.Intake.minWhileDownPos);
            spitDebounce = true;
        }
        if (intakeSpit) {
            if (elapsedTime.milliseconds() > spitTime + 100 && elapsedTime.milliseconds() < spitTime + 150) {
                intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
                intakeSystem.setIntakePower(Constants.Intake.spitSpeed);
            }
        }
        if (spitDebounce && !intakeSpit) {
            intakeSystem.setIntakePower(0);
            intakeSystem.setIntakeServoPos(Constants.Intake.wristStore);
            intakeSystem.setHSlidePos(0);
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
                outtakeSystem.setArmPos(Constants.Outtake.preTransferArm);
                intakeSystem.setIntakeServoPos(Constants.Intake.wristStore);
                intakeSystem.setIntakePower(Constants.Intake.transferSpeed);
            }
            if (elapsedTime.milliseconds() > transferTime + 400 && elapsedTime.milliseconds() < transferTime + 450) {
                outtakeSystem.setArmPos(Constants.Outtake.intakeArm);
            }
            if (elapsedTime.milliseconds() > transferTime + 700 && elapsedTime.milliseconds() < transferTime + 750) {
                intakeSystem.storePos();
                outtakeSystem.setVSlidePos(Constants.Outtake.intakeSlides);
                intakeSystem.setIntakeServoPos(Constants.Intake.wristClear);
            }
            if (elapsedTime.milliseconds() > transferTime + 1100 && elapsedTime.milliseconds() < transferTime + 1150) {
                outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                intakeSystem.setIntakePower(0);
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

        if (specimenOffWall && transferring) {
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
            placeSpecimenTime = elapsedTime.milliseconds();
            endArmDebounce = true;
        }
        if (endArmSpecimen && elapsedTime.milliseconds() > placeSpecimenTime + 200 && elapsedTime.milliseconds() < placeSpecimenTime + 250) {
            outtakeSystem.setVSlidePos(Constants.Outtake.endSpecimenSlides);
            driveForward = -1;
            autoDriving = true;
        }
        if (endArmDebounce && !endArmSpecimen) {
            outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
            outtakeSystem.setArmPos(Constants.Outtake.upArm);
            endArmDebounce = false;
            hasSpecimen = false;
        }



        if (spitHard && !spitHardDebounce) {
            intakeSystem.setIntakePower(-1);
        }
        if (!spitHard && spitHardDebounce) {
            intakeSystem.setIntakePower(0);
        }

        telemetry.addData("sample", hasSample);
        telemetry.addData("specimen", hasSpecimen);
        telemetry.addData("transferred", transferred);
        telemetry.addData("0", climbInit);
        telemetry.addData("1", climbL1P1);
        telemetry.addData("2", climbL2P1);
        telemetry.addData("3", climbL2P2);
        telemetry.addData("pos", climberPos);
        /*
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
        telemetry.addData("Climber T Position", climber.getTargetPos());
        telemetry.addData("power", driveTrain.pidControl.update(driveTrain.leftFront.getCurrentPosition()));
         */
        telemetry.update();
    }

    public void stop() {
        driveTrain.stopDrive();
    }
}
