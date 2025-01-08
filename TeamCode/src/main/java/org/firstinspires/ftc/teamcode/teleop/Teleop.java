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

    boolean store;
    boolean transfer;

    boolean specimenOffWall;
    boolean highBasket;
    boolean highSpecimen;

    boolean dropSpecimen;
    boolean dropClawBasket;
    boolean dropClawWall;
    boolean grabClawWall;

    boolean resetState;



    //Various Variables
    DriveSpeedEnum driveSpeed;
    boolean intakeingColor = false;
    boolean intakeing = false;
    boolean extended = false;
    boolean atHighBasket = false;
    boolean atPickupWall = false;
    boolean atStore = false;
    boolean storing = false;
    boolean transferring = false;
    boolean unjammingIntake;
    boolean hasSample = false;
    boolean hasColor = false;
    boolean hasSpecimen = false;
    boolean spitDebounce = false;
    boolean dropDebounce = false;
    boolean grabDebounce = false;
    boolean endArmDebounce = false;
    boolean unjamIntakeDebounce = false;
    boolean spitHardDebounce = false;
    boolean autoIntakeDebounce = false;
    boolean dropClawWallDebounce = false;
    boolean climbInit = false;
    boolean climbInitLoop = false;
    boolean climbL1P1 = false;
    boolean climbL2P1 = false;
    boolean climbL2P2 = false;
    boolean hardPID = true;


    int climberPos = 0;
    int PTOError = 0;

    double climbSlidesTime = 2000000000;
    double storeTime = 2000000000;
    double transferTime = 2000000000;
    double unjamTime = 2000000000;
    double autoWristTime = 2000000000;
    double spitTime = 2000000000;
    double PTOEnableTime = 2000000000;

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
    }

    public void start() {
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

        intakeSpit = hasSample && !atHighBasket && !atPickupWall && gamepad2.left_bumper; //1
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

        store = !hasSpecimen && gamepad2.dpad_left; //1
        transfer = hasSample && gamepad2.dpad_right;

        specimenOffWall = !hasSpecimen && gamepad2.dpad_right; //1 2
        highBasket = hasSample && !hasSpecimen && gamepad2.dpad_up; //1 2
//        highBasket
//        lowSpecimen;
        highSpecimen = hasSpecimen && gamepad2.dpad_up;

        dropSpecimen = hasSpecimen && !hasSample && gamepad2.left_bumper; //1
        dropClawBasket = !hasSpecimen && hasSample && atHighBasket && gamepad2.left_bumper; //1
        dropClawWall = !hasSpecimen & hasSample && atPickupWall && gamepad2.left_bumper; //1
        grabClawWall = !hasSpecimen && !hasSample && !atHighBasket && gamepad2.right_bumper; //1

        resetState = gamepad1.b;

        if (resetState) {
            hasSample = false;
            hasSpecimen = false;
            atHighBasket = false;
            atPickupWall = false;
        }

        //DRIVING
        if (!powerTakeOff.isEnabled()) {
            //Regular time
            driveTrain.drive(driveForward, driveStrafe, driveRotation, driveSpeed);
        } else {
            //Climbing
            if (PTODisable) {
                powerTakeOff.disable();
                climbInit = false;
                climbL1P1 = false;
                climbL2P1 = false;
                climbL2P2 = false;
                hardPID = true;
                climbInitLoop = false;
            }
            //big auto movement
            if (!climbInit) {
                climbInit = true;
                outtakeSystem.setVSlidePos(Constants.Outtake.climbSlides);
                if (!atPickupWall) {
                    climbInitLoop = true;
                    climbSlidesTime = elapsedTime.milliseconds();
                }
                climber.setPos(climberPos);
                climbL1P1 = true;
                PTOEnableTime = elapsedTime.milliseconds();
            }
            if (climbInitLoop && elapsedTime.milliseconds() > climbSlidesTime + 150) {
                climbInitLoop = false;
                outtakeSystem.setArmPos(Constants.Outtake.intakeArm);
            }
            if (climbL1P1 && elapsedTime.milliseconds() > PTOEnableTime + 800) {
                climbL1P1 = false;
                driveTrain.setPTOPos(Constants.PTO.motorClimb);
                climbL2P1 = true;
            }
            if (climbL2P1 && driveTrain.getPTOPos() > driveTrain.getPTOTPos() - 100) {
                climbL2P1 = false;
                climberPos = Constants.Climber.inPos;
                climbL2P2 = true;
            }
            if (climbL2P2 && climber.getPos() < climberPos + 1000) {
                climbL2P2 = false;
                driveTrain.setPTOPos(Constants.PTO.motorDrop);
                outtakeSystem.setArmPos(Constants.Outtake.upArm);
            }

            //fancy math for PTO feedforward
            PTOError = Math.abs(driveTrain.getPTOPos() - driveTrain.getPTOTPos());
            if (!climbL1P1) {
                if (PTOError > 500) {
                    driveTrain.PTOLoop(0.5);
                } else {
                    driveTrain.PTOLoop(0);
                }
            } else {
                driveTrain.moveBackWheels();
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
            storing = true;
            storeTime = elapsedTime.milliseconds();
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
                    hasSample = true;
                    hasColor = false;
                    intakeing = false;
                }
            }
        }
        if (intakeingColor && !unjammingIntake) {
            if (!hasSpecimen) {
                if (intakeSystem.intakeUntilColor()) {
                    intakeSystem.storePos();
                    hasSample = true;
                    hasColor = true;
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
        //todo: oidwojwq9hejwfuiwefhiuwefhiufhq fix plz tune 350 and 1000
        if (unjammingIntake) {
            if (elapsedTime.milliseconds() < unjamTime + 50) {
                intakeSystem.setIntakePower(Constants.Intake.unjamSpeed);
            }
            if (elapsedTime.milliseconds() > unjamTime + 350) {
                if (intakeSystem.intakeUntilColor() || elapsedTime.milliseconds() > unjamTime + 1000) {
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
            outtakeSystem.placePos(PlacePosEnum.wall);
            hasSample = false;
            hasSpecimen = false;
            spitDebounce = false;
        }

        //Presets
        if (store) {
            storeTime = elapsedTime.milliseconds();
            atHighBasket = false;
            atPickupWall = false;
            storing = true;
        }
        //Store
        if (storing) {
            if (elapsedTime.milliseconds() < storeTime + 50) {
                outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                outtakeSystem.setVSlidePos(Constants.Outtake.safeFromHSlides);
                outtakeSystem.setArmPos(Constants.Outtake.preTransferArm);
                intakeSystem.setIntakeServoPos(Constants.Intake.wristStore);
            }
            if (elapsedTime.milliseconds() > storeTime + 400 && elapsedTime.milliseconds() < storeTime + 450) {
                outtakeSystem.setArmPos(Constants.Outtake.intakeArm);
            }
            if (elapsedTime.milliseconds() > storeTime + 700 && elapsedTime.milliseconds() < storeTime + 750) {
                outtakeSystem.setVSlidePos(Constants.Outtake.intakeWaitSlides);
                storing = false;
                atStore = true;
            }
        }

        if (transfer && !atStore) {
            storeTime = elapsedTime.milliseconds();
            storing = true;
            transferTime = elapsedTime.milliseconds() + 300;
            transferring = true;
        } else if (transfer && storing) {
            transferTime = elapsedTime.milliseconds() + 300;
            transferring = true;
        } else if (transfer) {
            transferTime = elapsedTime.milliseconds();
            transferring = true;
        }

        if (transferring) {
            if (intakeSystem.getHSlidePos() > 100) {
                transferTime = elapsedTime.milliseconds() - 10;
            }
            if (intakeSystem.getHSlidePos() > 600) {
                transferTime = elapsedTime.milliseconds() + 500;
            }
            if (elapsedTime.milliseconds() > transferTime + 0 && elapsedTime.milliseconds() < transferTime + 50) {
                intakeSystem.setIntakePower(Constants.Intake.transferSpeed);
            }
            if (elapsedTime.milliseconds() > transferTime + 500 && elapsedTime.milliseconds() < transferTime + 550) {
                outtakeSystem.setVSlidePos(Constants.Outtake.intakeSlides);
            }
            if (elapsedTime.milliseconds() > transferTime + 800 && elapsedTime.milliseconds() < transferTime + 850) {
                intakeSystem.setIntakeServoPos(Constants.Intake.wristClear);
                intakeSystem.setIntakePower(0);
                outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
            }
            if (elapsedTime.milliseconds() > transferTime + 1100 && elapsedTime.milliseconds() < transferTime + 1150) {
                outtakeSystem.setVSlidePos(Constants.Outtake.safeFromHSlides);
            }
            if (elapsedTime.milliseconds() > transferTime + 1500 && elapsedTime.milliseconds() < transferTime + 1550) {
                if (!hasColor) {
                    outtakeSystem.setVSlidePos(Constants.Outtake.highBasketSlides);
                    outtakeSystem.setArmPos(Constants.Outtake.basketArm);
                    atHighBasket = true;
                    atPickupWall = false;
                } else {
                    outtakeSystem.setArmPos(Constants.Outtake.intakeWallArm);
                }
            }
            if (elapsedTime.milliseconds() > transferTime + 2000 && elapsedTime.milliseconds() < transferTime + 2050) {
                if (hasColor) {
                    outtakeSystem.setVSlidePos(Constants.Outtake.intakeWallSlides);
                    atPickupWall = true;
                    atHighBasket = false;
                }
                intakeSystem.setIntakeServoPos(Constants.Intake.wristStore);
                transferring = false;
            }
        }

        //Presets
        if (specimenOffWall && !atHighBasket && !atPickupWall) {
            hasColor = true;
            transferTime = elapsedTime.milliseconds();
            transferring = true;
        } else if (specimenOffWall && atHighBasket) {
            outtakeSystem.placePos(PlacePosEnum.wall);
            atPickupWall = true;
            atHighBasket = false;
            hasSpecimen = false;
        }

        if (highBasket && !atHighBasket && !atPickupWall) {
            hasColor = false;
            transferTime = elapsedTime.milliseconds();
            transferring = true;
        } else if (highBasket && atPickupWall) {
            outtakeSystem.placePos(PlacePosEnum.highBasket);
            atPickupWall = false;
            atHighBasket = true;
            hasSpecimen = false;
        }

        if (highSpecimen && !atHighBasket && !atPickupWall) {
            outtakeSystem.placePos(PlacePosEnum.highSpecimen);
        }


        //Manual
        intakeSystem.manualHSlide(manualHSlidePower);
        outtakeSystem.manualVSlide(manualVSlidePower);
        if (gamepad1.right_stick_button) {
            outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
            outtakeSystem.manualArm(manualArmPower);
        }

        //drop into basket
        if (dropClawBasket && !dropDebounce) {
            outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
            dropDebounce = true;
        }
        if (dropDebounce && !dropClawBasket) {
            storing = true;
            storeTime = elapsedTime.milliseconds();
            hasSample = false;
            atHighBasket = false;
            dropDebounce = false;
        }

        //grab from wall
        if (grabClawWall && !grabDebounce) {
            outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
            grabDebounce = true;
        }
        if (grabDebounce && !grabClawWall) {
            outtakeSystem.setVSlidePos(Constants.Outtake.safeFromWallSlides);
            hasSpecimen = true;
            grabDebounce = false;
        }
        //drop at wall
        if (dropClawWall && !dropClawWallDebounce) {
            outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
            dropClawWallDebounce = true;
        }
        if (!dropClawWall && dropClawWallDebounce) {
            hasSample = false;
            dropClawWallDebounce = false;
        }
        //drop specimen
        if (dropSpecimen && !endArmDebounce) {
            outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
            hasSpecimen = false;
            endArmDebounce = true;
        }

        if (endArmDebounce && !dropSpecimen) {
            storing = true;
            storeTime = elapsedTime.milliseconds();
            endArmDebounce = false;
            hasSpecimen = false;
        }


        if (spitHard && !spitHardDebounce) {
            intakeSystem.setIntakePower(-1);
        }
        if (!spitHard && spitHardDebounce) {
            intakeSystem.setIntakePower(0);
        }


        intakeSystem.update();
        outtakeSystem.update();
        telemetry.addData("sample", hasSample);
        telemetry.addData("hasColor", hasColor);
        telemetry.addData("specimen", hasSpecimen);
        telemetry.addData("atHighBasket", atHighBasket);
        telemetry.addData("atPickupWall", atPickupWall);
        telemetry.addData("","");
        telemetry.addData("0", climbInit);
        telemetry.addData("0.5", climbInitLoop);
        telemetry.addData("1", climbL1P1);
        telemetry.addData("2", climbL2P1);
        telemetry.addData("3", climbL2P2);
        telemetry.addData("pos", climberPos);
        telemetry.addData("pto Pos", driveTrain.getPTOPos());
        telemetry.addData("PtoTpos", driveTrain.getPTOTPos());
        telemetry.addData("","");
        telemetry.addData("Vslidepos", outtakeSystem.getVSlidePos());
        telemetry.addData("VslideTpos", outtakeSystem.getVSlideTargetPos());
        telemetry.addData("Hslidepos", intakeSystem.getHSlidePos());
        telemetry.addData("HslideTpos", intakeSystem.getHSlideTargetPos());
        telemetry.addData("ArmTPos", outtakeSystem.getArmPos());
        telemetry.addData("IntakeServo", intakeSystem.intakeClaw.getIntakeServoPos());
        telemetry.addData("tuchyWuchyH", intakeSystem.HSlidePressed());
        telemetry.addData("tuchyWuchyV", outtakeSystem.VSlidePressed());
        telemetry.addData("","");
        telemetry.addData("intakeing", intakeing);
        telemetry.addData("unjammingIntake", unjammingIntake);
        telemetry.addData("intakeingColor", intakeingColor);
        telemetry.addData("storing", storing);
        telemetry.addData("transferring", transferring);
        telemetry.addData("","");
        telemetry.addData("color", intakeSystem.color());
        telemetry.addData("distance", intakeSystem.distance() < 1.5);
        telemetry.addData("intakePos", intakeSystem.intakeClaw.getIntakeServoPos());
        telemetry.addData("outtakePos", outtakeSystem.outtakeArm.getClawPos());
        telemetry.addData("Climber Position", climber.getPos());
        telemetry.addData("Climber T Position", climber.getTargetPos());
        telemetry.addData("power", driveTrain.pidControl.update(driveTrain.leftFront.getCurrentPosition()));
        telemetry.update();
    }

    public void stop() {
        driveTrain.stopDrive();
    }
}
