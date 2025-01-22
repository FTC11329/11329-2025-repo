package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.BlockVision;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.PowerTakeOff;
import org.firstinspires.ftc.teamcode.utility.DriveSpeedEnum;
import org.firstinspires.ftc.teamcode.utility.PlacePosEnum;
import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;
import org.firstinspires.ftc.teamcode.utility.StateMachine;

public class NewTeleop {
    Climber climber;
    Drivetrain driveTrain;
    BlockVision blockVision;
    PowerTakeOff powerTakeOff;
    IntakeSystem intakeSystem;
    OuttakeSystem outtakeSystem;

    StateMachine stateMachine;
    ElapsedTime elapsedTime = new ElapsedTime();

    //Debug Variables
    boolean debugAll = true;
    boolean debugState = false;
    boolean debugStateMachine = false;
    boolean debugPos = false;
    boolean debugClimber = false;
    boolean debugMisc = false;

    //Input Variables
    double driveForward;
    double driveStrafe;
    double driveRotation;
    DriveSpeedEnum driveSpeed;
    boolean climbToggButton;

    double manualVSlide;
    double manualHSlide;
    double manualArm;
    double manualClimber;

    boolean highSpecimen;
    boolean highBasket;
    boolean lowBasket;
    boolean wallPreset;
    boolean storePos;
    boolean transfer;

    boolean clawToggleButton;

    boolean intake;
    boolean intakeColor;
    boolean unJam;

    //State Machine Variables
    boolean hasInIntake;
    boolean atStorePos;
    boolean transferred;
    PlacePosEnum whereAmI = PlacePosEnum.highSpecimen;

    //Various Variables
    int climberStage = 0;
    Timer climberTimer = new Timer();
    int climberPos = 0;

    boolean climbTogg = false;
    boolean climbDebounce = false;

    boolean onceTime = true;
    double storeTime = 2000000000;
    double transferTime = 2000000000;
    double unStoringTime = 2000000000;

    boolean clawToggle = false;
    boolean clawDebounce = false;

    boolean grabbingOffWall = false;
    double grabbingOffWallTime = 2000000000;
    boolean droppingBasket = false;
    double droppingBasketTime = 2000000000;

    boolean intakeingColor = false;
    boolean intakeing = false;
    boolean intakeingDebounce = false;
    double intakeWristTime = 2000000000;

    boolean unjamming = false;
    double unjammingTime = 2000000000;


    //this is here because I have to have a teleop blue and teleop red
    HardwareMap hardwareMap;
    Telemetry telemetry;
    Gamepad gamepad1;
    Gamepad gamepad2;
    RobotSideEnum robotSide;

    public NewTeleop(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, RobotSideEnum robotSide) {
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

        stateMachine = new StateMachine();
    }

    public void start() {
        intakeSystem = new IntakeSystem(hardwareMap, robotSide);
        outtakeSystem = new OuttakeSystem(hardwareMap);
        elapsedTime.reset();
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

        climbToggButton = gamepad1.back;

        manualVSlide = -gamepad2.right_stick_y;
        manualHSlide = gamepad2.right_trigger - gamepad2.left_trigger;
        manualArm = gamepad2.left_stick_y;
        manualClimber = gamepad1.right_trigger - gamepad1.left_trigger;

        highSpecimen = gamepad2.dpad_up;
        highBasket = gamepad2.dpad_right;
        lowBasket = gamepad2.dpad_down;
        wallPreset = gamepad2.dpad_left;
        storePos = gamepad2.y;
        transfer = gamepad2.left_bumper;

        clawToggleButton = gamepad1.left_bumper || gamepad2.right_bumper;

        intake = gamepad2.a;
        intakeColor = gamepad2.x;
        unJam = gamepad2.b;

        //Toggle Climber
        if (climbToggButton && !climbDebounce) {
            climbTogg = !climbTogg;
        }
        if (!climbToggButton) {
            climbDebounce = true;
        }


        //Drivetrain
        if (!powerTakeOff.isEnabled()) {
            //Drive time
            driveTrain.drive(driveForward, driveStrafe, driveRotation, driveSpeed);

            if (climbTogg) {
                powerTakeOff.enable();
                driveTrain.setRunToPos();
                climberTimer.resetTimer();
            }
        } else {
            if (!climbTogg) {
                powerTakeOff.disable();
            }
            switch (climberStage) {
                case 0:
                    powerTakeOff.setHookRelease(Constants.PTO.releaseTheHooks);
                    //puts arm to safe space
                    if (outtakeSystem.getArmPos() > 0.6) {
                        stateMachine.doGoToStore();
                    } else {
                        stateMachine.goWall(false, false, false);
                    }

                    climberTimer.resetTimer();
                    climberStage = 1;
                    break;
                case 1:
                    if (climberTimer.getElapsedTimeSeconds() > 0.5) {
                        driveTrain.setRunToPos();
                        powerTakeOff.enable();
                        //disables to save power
                        outtakeSystem.disable();
                        intakeSystem.disable();

                        climberTimer.resetTimer();
                        climberStage = 2;
                    }
                    break;
                case 2:
                    if (climberTimer.getElapsedTimeSeconds() > 0.8) {
                        driveTrain.setPTOPos(Constants.PTO.motorClimb);

                        climberTimer.resetTimer();
                        climberStage = 3;
                    }
                    break;
                case 3:
                    if (Math.abs(driveTrain.getPTOPos() - driveTrain.getPTOTPos()) < 100) {
                        climber.setPos(Constants.Climber.hookPos);
                        outtakeSystem.reEnable(Constants.Outtake.maxSlides);

                        climberTimer.resetTimer();
                        climberStage = 4;
                    }
                    break;
                case 4:
                    if (Math.abs(climber.getPos() - climber.getTargetPos()) < 300) {
                        //disable PTO to conserve power
                        driveTrain.setPTOPower(0);
                        climber.setPos(Constants.Climber.inPos);

                        climberTimer.resetTimer();
                        climberStage = 5;
                    }
                    break;
                case 5:
                    if (Math.abs(climber.getPos() - climber.getTargetPos()) < 500) {
                        driveTrain.setPTOPos(Constants.PTO.motorDrop);

                        climberTimer.resetTimer();
                        climberStage = 6;
                    }

            }
            if (climberStage <= 2) {
                driveTrain.moveBackWheels();
            } else if (climberStage <= 4) {
                driveTrain.PTOLoop(0.5);
            } else if (climberStage == 6) {
                driveTrain.PTOLoop(0);
            }


            //manual movement
            climberPos += (int) (20 * (manualClimber));
            climber.setPos(climberPos);
        }

        //Manual Movements
        intakeSystem.manualHSlide(manualHSlide);
        outtakeSystem.manualVSlide(manualVSlide);
        outtakeSystem.manualArm(manualArm);

        //Presets
        //Button to State Machine class
        if (highSpecimen) {
            stateMachine.goHighSpecimen(atStorePos);
        }
        if (highBasket) {
            stateMachine.goHighBasket(hasInIntake, transferred, atStorePos);
        }
        if (lowBasket) {
            stateMachine.goLowBasket(hasInIntake, transferred, atStorePos);
        }
        if (wallPreset) {
            stateMachine.goWall(hasInIntake, transferred, atStorePos);
        }
        if (storePos) {
            stateMachine.goStore();
        }
        if (transfer) {
            stateMachine.goTransfer(atStorePos);
        }


        //State Machine class to movement
        if (stateMachine.doGoToStore()) {
            if (onceTime) {
                storeTime = elapsedTime.milliseconds();
                onceTime = false;
            }
            if (elapsedTime.milliseconds() < storeTime + 100) {
                outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                clawToggle = false;
                outtakeSystem.setVSlidePos(Constants.Outtake.safeFromHSlides);
                outtakeSystem.setArmPos(Constants.Outtake.preTransferArm);
                intakeSystem.setIntakeServoPos(Constants.Intake.wristStore);
            }
            if (elapsedTime.milliseconds() > storeTime + 400 && elapsedTime.milliseconds() < storeTime + 500) {
                outtakeSystem.setArmPos(Constants.Outtake.intakeArm);
            }
            if (elapsedTime.milliseconds() > storeTime + 700 && elapsedTime.milliseconds() < storeTime + 800) {
                outtakeSystem.setVSlidePos(Constants.Outtake.safeFromHSlides);
                atStorePos = true;
                onceTime = true;
                stateMachine.finishGoToStore();
            }
        }

        if (stateMachine.doTransfer()) {
            if (onceTime) {
                transferTime = elapsedTime.milliseconds();
                onceTime = false;
            }
            if (elapsedTime.milliseconds() > transferTime + 0 && elapsedTime.milliseconds() < transferTime + 100) {
                intakeSystem.setIntakePower(Constants.Intake.transferSpeed);
            }
            if (elapsedTime.milliseconds() > transferTime + 500 && elapsedTime.milliseconds() < transferTime + 600) {
                outtakeSystem.setVSlidePos(Constants.Outtake.intakeSlides);
            }
            if (elapsedTime.milliseconds() > transferTime + 900 && elapsedTime.milliseconds() < transferTime + 1000) {
                intakeSystem.setIntakeServoPos(Constants.Intake.wristClear);
                intakeSystem.setIntakePower(0);
                outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                clawToggle = true;
            }
            if (elapsedTime.milliseconds() > transferTime + 1200 && elapsedTime.milliseconds() < transferTime + 1300) {
                outtakeSystem.setVSlidePos(Constants.Outtake.safeFromHSlides);
                intakeSystem.setIntakeServoPos(Constants.Intake.wristStore);
            }
            if (elapsedTime.milliseconds() > transferTime + 1400 && elapsedTime.milliseconds() < transferTime + 1500) {
                hasInIntake = false;
                transferred = true;
                onceTime = true;
                stateMachine.finishTransfer();
            }
        }

        if (stateMachine.doUnStore()) {
            if (onceTime) {
                unStoringTime = elapsedTime.milliseconds();
                onceTime = false;
            }
            if (elapsedTime.milliseconds() > unStoringTime + 0 && elapsedTime.milliseconds() < unStoringTime + 100) {
                outtakeSystem.setArmPos(Constants.Outtake.intakeWallArm);
            }
            if (elapsedTime.milliseconds() > unStoringTime + 200 && elapsedTime.milliseconds() < unStoringTime + 300) {
                atStorePos = false;
                onceTime = true;
                stateMachine.finishUnStore();
            }
        }

        if (stateMachine.doHighSpecimen()) {
            outtakeSystem.placePos(PlacePosEnum.highSpecimen);
            whereAmI = PlacePosEnum.highSpecimen;
            stateMachine.finishHighSpecimen();
        }

        if (stateMachine.doLowBasket()) {
            outtakeSystem.placePos(PlacePosEnum.lowBasket);
            whereAmI = PlacePosEnum.lowBasket;
            stateMachine.finishLowBasket();
        }

        if (stateMachine.doHighBasket()) {
            outtakeSystem.placePos(PlacePosEnum.highBasket);
            whereAmI = PlacePosEnum.highBasket;
            stateMachine.finishHighBasket();
        }

        if (stateMachine.doWall()) {
            outtakeSystem.placePos(PlacePosEnum.wall);
            if (!transferred) {
                outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                clawToggle = true;
            }
            whereAmI = PlacePosEnum.wall;
            stateMachine.finishWall();
        }



        // Claw Controls
        if (clawToggleButton && !clawDebounce) {
            clawToggle = !clawToggle;
            clawDebounce = true;

            if (clawToggle && whereAmI == PlacePosEnum.wall) {
                outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                grabbingOffWall = true;
                grabbingOffWallTime = elapsedTime.milliseconds();
            } else if (clawToggle) {
                //remove to lock down more
                outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
            }

            if (!clawToggle && (whereAmI == PlacePosEnum.lowBasket || whereAmI == PlacePosEnum.highBasket)) {
                transferred = false;
                outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                droppingBasket = true;
                droppingBasketTime = elapsedTime.milliseconds();
            } else if (!clawToggle) {
                transferred = false;
                outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
            }
        }
        if (!clawToggleButton) {
            clawDebounce = false;
        }

        if (grabbingOffWall) {
            if (elapsedTime.milliseconds() > grabbingOffWallTime + 300) {
                outtakeSystem.setVSlidePos(Constants.Outtake.safeFromWallSlides);
                grabbingOffWall = false;
            }
        }

        if (droppingBasket) {
            if (elapsedTime.milliseconds() > droppingBasketTime + 300 && elapsedTime.milliseconds() < droppingBasketTime + 400) {
                outtakeSystem.setArmPos(Constants.Outtake.upArm);
            }
            if (elapsedTime.milliseconds() > droppingBasketTime + 500) {
                stateMachine.goStore();
                droppingBasket = false;
            }
        }

        //Intakes
        if (intakeColor && !intakeingDebounce && !unjamming) {
            intakeSystem.setHSlidePos(Constants.Intake.intakeSlidePos);
            intakeingColor = true;
            intakeWristTime = elapsedTime.milliseconds();
        }
        if (intake && !intakeingDebounce && !unjamming) {
            intakeSystem.setHSlidePos(Constants.Intake.intakeSlidePos);
            intakeing = true;
            intakeWristTime = elapsedTime.milliseconds();
        }

        if (intakeing && !unjamming) {
            if (intakeSystem.intakeUntil()) {
                intakeSystem.storePos();
                hasInIntake = true;
                intakeing = false;
            }
        }
        if (intakeingColor && !unjamming) {
            if (intakeSystem.intakeUntilColor()) {
                intakeSystem.storePos();
                hasInIntake = true;
                intakeingColor = false;
            }
        }

        //makes the intake wrist not hit the robot while coming out
        if ((intakeingColor && !intakeColor) || (intakeing && !intake) && elapsedTime.milliseconds() > intakeWristTime + 300) {
            intakeingDebounce = false;
            intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
            intakeWristTime = 2000000000;
        }
        //Unjamming intake
        if (unJam) {
            unjamming = true;
            unjammingTime = elapsedTime.milliseconds();
        }
        if (unjamming) {
            if (elapsedTime.milliseconds() < unjammingTime + 100) {
                intakeSystem.setIntakePower(Constants.Intake.unjamSpeed);
            }
            if (elapsedTime.milliseconds() > unjammingTime + Constants.Intake.unjamTimeMillis) {
                if (intakeSystem.intakeUntil() || elapsedTime.milliseconds() > unjammingTime + 1000) {
                    intakeSystem.setIntakePower(0);
                    unjamming = false;
                }
            }
        }

        intakeSystem.update();
        outtakeSystem.update();

        //DEBUG
        if (debugState || debugAll) {
            telemetry.addLine("STATE");
            telemetry.addData("hasInIntake", hasInIntake);
            telemetry.addData("transferred", transferred);
            telemetry.addData("atStore", atStorePos);
            telemetry.addData("where am I", whereAmI);
            telemetry.addLine();
        }
        if (debugStateMachine || debugAll) {
            telemetry.addLine("STATE MACHINE");
            telemetry.addData("doGoToStore", stateMachine.doGoToStore());
            telemetry.addData("doTransfer", stateMachine.doTransfer());
            telemetry.addData("doUnStore", stateMachine.doUnStore());
            telemetry.addData("doHighBasket", stateMachine.doHighBasket());
            telemetry.addData("doLowBasket", stateMachine.doLowBasket());
            telemetry.addData("doHighSpecimen", stateMachine.doHighSpecimen());
            telemetry.addData("doWall", stateMachine.doWall());
            telemetry.addData("hasInIntake", stateMachine.debug()[0]);
            telemetry.addData("transferred", stateMachine.debug()[1]);
            telemetry.addData("atStore", stateMachine.debug()[2]);
            telemetry.addLine();
        }
        if (debugPos || debugAll) {
            telemetry.addLine("POSITION");
            telemetry.addData("V Slide Tar", outtakeSystem.getVSlideTargetPos());
            telemetry.addData("V Slide Pos", outtakeSystem.getVSlidePos());
            telemetry.addData("H Slide Tar", intakeSystem.getHSlideTargetPos());
            telemetry.addData("H Slide Pos", intakeSystem.getHSlidePos());
            telemetry.addData("Arm Pos", outtakeSystem.getArmPos());
            telemetry.addLine();
        }
        if (debugClimber || debugAll) {
            telemetry.addLine("CLIMBER");
            telemetry.addData("climberStage", climberStage);
            telemetry.addData("PTO Tar", driveTrain.getPTOTPos());
            telemetry.addData("PTO Pos", driveTrain.getPTOPos());
            telemetry.addData("PTO Pow", Math.max(Math.max(driveTrain.getDrivePowers()[0], driveTrain.getDrivePowers()[1]), Math.max(driveTrain.getDrivePowers()[2], driveTrain.getDrivePowers()[3])));
            telemetry.addData("Climber Tar", climber.getTargetPos());
            telemetry.addData("Climber Pos", climber.getPos());
            telemetry.addLine();
        }
        if (debugMisc || debugAll) {
            telemetry.addLine("MISCELLANEOUS");
            telemetry.addData("onceTime", onceTime);
            telemetry.addData("transferTime", transferTime);
            //TODO add more Things here
            telemetry.addLine();
        }

        telemetry.update();
    }

    public void stop() {
        driveTrain.stopDrive();
    }
}
