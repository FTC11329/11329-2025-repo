package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.PowerTakeOff;
import org.firstinspires.ftc.teamcode.utility.DriveSpeedEnum;
import org.firstinspires.ftc.teamcode.utility.PlacePosEnum;
import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;
import org.firstinspires.ftc.teamcode.utility.StateMachine;

public class Teleop {
    //comment me out V
//    DcMotorEx motor1, motor2, motor3, motor4, motor5, motor6, motor7, motor8;
    Climber climber;
    Drivetrain driveTrain;
    PowerTakeOff powerTakeOff;
    IntakeSystem intakeSystem;
    OuttakeSystem outtakeSystem;

    StateMachine stateMachine;
    ElapsedTime elapsedTime = new ElapsedTime();

    FtcDashboard dashboard;

    //Debug Variables
    boolean debugAll = false;
    boolean debugState = false;
    boolean debugStateMachine = false;
    boolean debugPos = false;
    boolean debugColor = false;
    boolean debugClimber = false;
    boolean debugPower = false;
    //Requires ^ uncommenting things
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

    boolean gamepad1B;

    boolean sideDepo;

    boolean highSpecimen;
    boolean highBasket;
    boolean lowBasket;
    boolean frontBasket;
    boolean wallPreset;
    boolean storePos;
    boolean transfer;

    boolean clawToggleButton;

    boolean intake;
    boolean intakeColor;
    boolean unJam;

    //State Machine Variables
    boolean hasInIntake = false;;
    boolean atStorePos = false;;
    boolean hasInOuttake = false;
    boolean hasInTray = false;
    PlacePosEnum whereAmI = PlacePosEnum.wall;

    //Various Variables
    int climberStage = 0;
    Timer climberTimer = new Timer();
    int climberPos = 0;

    boolean climberActive = false;
    boolean climbPause = false;
    boolean climbDebounce = false;
    boolean lastCurrentTrip = false;
    double lastCurrentTripTime = 2000000000;
    boolean climbStopPause = false;
    boolean climbStopPauseOnce = true;

    boolean onceTime = true;
    boolean onceWall = true;
    boolean onceState = true;
    boolean wallOnce = true;
    double storeTime = 2000000000;
    double transferTime = 2000000000;
    double unStoringTime = 2000000000;
    double wallTime = 2000000000;
    double lastTime = 2000000000;

    boolean transferFirstTime = true;

    boolean clawToggle = true;
    boolean clawDebounce = false;

    boolean grabbingOffWall = false;
    double grabbingOffWallTime = 2000000000;
    boolean droppingBasket = false;
    double droppingBasketTime = 2000000000;
    double sideDepoTime = 2000000000;
    boolean sideDepoFirst = false;
    boolean sideDepoDebounce = false;

    boolean downOnce = true;
    boolean sideDepoing = false;

    boolean intakeingColor = false;
    boolean intakeing = false;
    boolean afterIntakeingColor = false;
    boolean afterIntakeing = false;
    double afterIntakeingColorTime = 2000000000;
    double afterIntakeingTime = 2000000000;
    boolean intakeingDebounce = false;
    double intakeTime = 2000000000;
    double intakeWristTime = 2000000000;
    int extendHSlide = Constants.Intake.intakeSlidePos;

    boolean unjamming = false;
    double unjammingTime = 2000000000;
    boolean unjamAfterIntake = false;
    double unjamAfterIntakeTime = 2000000000;

    double tempTime1 = 0;
    double tempTime2 = 0;
    double tempTime3 = 0;
    double tempTime4 = 0;
    double tempTime5 = 0;


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
        elapsedTime.reset();
        //comment me out
//        motor1 = hardwareMap.get(DcMotorEx.class, "leftFront");
//        motor2 = hardwareMap.get(DcMotorEx.class, "rightFront");
//        motor3 = hardwareMap.get(DcMotorEx.class, "rightBack");
//        motor4 = hardwareMap.get(DcMotorEx.class, "leftBack");
//        motor5 = hardwareMap.get(DcMotorEx.class, "hSlides");
//        motor6 = hardwareMap.get(DcMotorEx.class, "vSlides");
//        motor7 = hardwareMap.get(DcMotorEx.class, "climber");
//        motor8 = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        //uncomment if you want telemetry on dashboard
//        dashboard = FtcDashboard.getInstance();
//        telemetry = dashboard.getTelemetry();

        tempTime1 = elapsedTime.milliseconds();
        climber = new Climber(hardwareMap);
        tempTime2 = elapsedTime.milliseconds();
        driveTrain = new Drivetrain(hardwareMap);
        tempTime3 = elapsedTime.milliseconds();
        stateMachine = new StateMachine();
    }

    public void start() {
        tempTime4 = elapsedTime.milliseconds();
        intakeSystem = new IntakeSystem(hardwareMap, robotSide);
        powerTakeOff = new PowerTakeOff(hardwareMap);
        outtakeSystem = new OuttakeSystem(hardwareMap, robotSide, true);
        outtakeSystem.setArmPos(Constants.Outtake.upArm);
        tempTime5 = elapsedTime.milliseconds();
        elapsedTime.reset();
        telemetry.addData("1", tempTime1);
        telemetry.addData("2", tempTime2);
        telemetry.addData("3", tempTime3);
        telemetry.addData("4", tempTime4);
        telemetry.addData("5", tempTime5);
        telemetry.update();
    }


    public void loop() {
        // Inputs
        gamepad1B = gamepad1.b;

        debugAll = gamepad1.a;
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
        if (!climberActive) {
            manualHSlide = gamepad2.right_trigger + (0.6 *(-gamepad2.left_trigger + gamepad1.right_trigger - gamepad1.left_trigger));
            if (debugAll) {
                manualHSlide = manualHSlide * 2;
            }
            manualArm = gamepad2.left_stick_y;
            manualClimber = gamepad1.right_trigger - gamepad1.left_trigger;
        } else {
            manualHSlide = 0;
            manualArm = 0;
            manualClimber = 0;
        }

        sideDepo = (gamepad2.touchpad && gamepad2.touchpad_finger_1_x > 0) || (gamepad2.dpad_left && hasInIntake);

        highSpecimen = gamepad2.dpad_up;
        highBasket = gamepad2.dpad_right;
        lowBasket = gamepad2.dpad_down;
        frontBasket = gamepad2.touchpad && gamepad2.touchpad_finger_1_x < 0;
        wallPreset = gamepad2.dpad_left;
        storePos = gamepad2.y;//triangle
        transfer = gamepad2.left_bumper;

        clawToggleButton = gamepad1.left_bumper || gamepad2.right_bumper;

        intake = gamepad2.a;//cross
        intakeColor = gamepad2.x;//square
        unJam = gamepad2.b;//circle

        //Drivetrain *****************************************************************************~D
        if (!climberActive && !climbPause) {
            if (climbToggButton) {
                climberActive = true;
                climbDebounce = true;
            }
            //Drive time
            driveTrain.drive(driveForward, driveStrafe, driveRotation, driveSpeed);

            if (climberActive) {
                driveTrain.setRunToPos();
                climberTimer.resetTimer();
            }
        }
        //Auto Climb ****************************************************************************~Ci
        if (climberActive && !climbPause) {
            //Climb Pause
            if (climbToggButton && !climbDebounce) {
                climbPause = true;
                //puts arm to safe space
                if (outtakeSystem.getArmPos() > 0.6) {
                    outtakeSystem.setVSlidePos(Constants.Outtake.maxSlides);
                    outtakeSystem.setArmPos(Constants.Outtake.intakeArm);
                } else {
                    outtakeSystem.setVSlidePos(Constants.Outtake.maxSlides);
                    outtakeSystem.setArmPos(Constants.Outtake.intakeWallArm);
                }
                climber.setPos(climber.getPos());
                driveTrain.setPTOPos(driveTrain.getPTOPos());
            }
            if (!climbToggButton) {
                climbDebounce = false;
            }

            double current;
            switch (climberStage) {
                case 0:
                    intakeSystem.storePos();
                    intakeSystem.setIntakeServoPos(Constants.Intake.wristClimb);
                    outtakeSystem.setVSlidePos(450);

                    climberPos = Constants.Climber.outPos;

                    //Enable PTO
                    driveTrain.setRunToPos();
                    powerTakeOff.enable();

                    climberTimer.resetTimer();
                    climberStage = 1;
                    break;
                case 1:
                    driveTrain.moveBackWheels();
                    if (climberTimer.getElapsedTimeSeconds() > 0.5 && Math.abs(climber.getPos() - climberPos) < 500) {
                        driveTrain.setPTOPos(Constants.PTO.motorClimb);

                        climberTimer.resetTimer();
                        climberStage = 2;
                    }
                    break;
                case 2:
                    driveTrain.setPTOPower(0.9);
                    if (climber.getDistance() > 10.8) {
                        climberPos = Constants.Climber.hookPos;
                        outtakeSystem.setVSlidePos(Constants.Outtake.maxSlides);
                        //Prevent pto from drawing too much power
                        driveTrain.setPTOPos(driveTrain.getPTOPos());

                        climberTimer.resetTimer();
                        climberStage = 3;
                    }
                    break;
                case 3:
                    driveTrain.setPTOPower(0.35);
                    if (Math.abs(climber.getPos() - Constants.Climber.hookPos) < 100) {
                        //puts arm to safe space
                        if (outtakeSystem.getArmPos() > 0.6) {
                            outtakeSystem.setVSlidePos(Constants.Outtake.maxSlides);
                            outtakeSystem.setArmPos(Constants.Outtake.intakeArm);
                        } else {
                            outtakeSystem.setVSlidePos(Constants.Outtake.maxSlides);
                            outtakeSystem.setArmPos(Constants.Outtake.intakeWallArm);
                        }
                        //disable PTO to conserve power
                        driveTrain.setPTOPower(0);

                        climberPos = Constants.Climber.inPos;

                        climberTimer.resetTimer();
                        climberStage = 4;
                    }
                    break;
                case 4:
                    driveTrain.setPTOPower(0);
                    if (Math.abs(climber.getPos() - Constants.Climber.inPos) < 1000) {
                        outtakeSystem.disable();

                        climberTimer.resetTimer();
                        climberStage = 5;
                    }
                    break;
                case 5:
                    driveTrain.setPTOPower(-1);
                    //Does some things to make sure that the current has been tripped for more than 1 second after one one second
                    if (climberTimer.getElapsedTimeSeconds() > 0.1) {
                        current = Math.min(Math.max(driveTrain.getDriveCurrent()[0], driveTrain.getDriveCurrent()[1]), Math.max(driveTrain.getDriveCurrent()[2], driveTrain.getDriveCurrent()[3]));
                    } else {
                        current = 0;
                    }
                    if (current > 4) {
                        //Prevent pto from drawing too much power
                        driveTrain.setPTOPos(driveTrain.getPTOPos());

                        climberTimer.resetTimer();
                        climberStage = 6;
                    }
                    break;
                case 6:
                    driveTrain.setPTOPower(-0.2);
                    break;
            }

            //manual movement
            climberPos += (int) (20 * (manualClimber));
            climber.setPos(climberPos);
        }
        if (climbPause) {
            if (gamepad1B) {
                climbStopPause = true;
            }
            if (!climbStopPause) {
                climber.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
                driveTrain.setPTOPower(-gamepad1.left_stick_y);
            }

            if (climbStopPause && climbStopPauseOnce) {
                climbStopPauseOnce = false;
                climber.setPos(climber.getPos());
            }

            if (climbStopPause) {
                driveTrain.setPTOPower(-0.2);
            }
        }
        // Pre-Start Climb
        if (gamepad1.dpad_up || gamepad2.back) {
            climberPos = Constants.Climber.prePos;
            climber.setPos(climberPos);
        }

        //Manual Movements ***********************************************************************~M
        intakeSystem.manualHSlide(manualHSlide);
        outtakeSystem.manualVSlide(manualVSlide);
        outtakeSystem.manualArm(manualArm);


        intakeSystem.update();
        outtakeSystem.update(Math.abs(manualVSlide) > 0.05);

        //Presets
        //Button to State Machine class *********************************************************~BS
        if (highSpecimen) {
            if (!stateMachine.doTransfer()) {
                onceTime = true;
            }
            intakeSystem.storePos();
            stateMachine.goHighSpecimen(atStorePos);
        }
        if (lowBasket) {
            if (!stateMachine.doTransfer()) {
                onceTime = true;
            }
            intakeingColor = false;
            intakeing = false;
            stateMachine.goLowBasket(hasInIntake || hasInTray, hasInOuttake, atStorePos);
        }
        if (highBasket) {
            if (!stateMachine.doTransfer()) {
                onceTime = true;
            }
            intakeingColor = false;
            intakeing = false;
            stateMachine.goHighBasket(hasInIntake || hasInTray, hasInOuttake, atStorePos);
        }
        if (frontBasket) {
            if (!stateMachine.doTransfer()) {
                onceTime = true;
            }
            intakeingColor = false;
            intakeing = false;
            stateMachine.goFrontBasket(hasInIntake || hasInTray, hasInOuttake, atStorePos);
        }
        if (wallPreset) {
            if (!stateMachine.doTransfer()) {
                onceTime = true;
            }
            intakeingColor = false;
            intakeing = false;
            stateMachine.goWall(hasInIntake || hasInTray, hasInOuttake, atStorePos);
        }
        if (storePos) {
            onceTime = true;
            stateMachine.resetValues();
            intakeSystem.storePos();
            intakeingColor = false;
            intakeing = false;
            stateMachine.goStore();
            extendHSlide = Constants.Intake.intakeSlidePos;
        }
        if (transfer) {
            if (!stateMachine.doTransfer() && !stateMachine.doUnStore()) {
                onceTime = true;
            }
            intakeSystem.storeOutPos();
            intakeingColor = false;
            intakeing = false;
            stateMachine.goTransfer(atStorePos);
        }


        //State Machine class to movement *******************************************************~SM
        if (stateMachine.doGoToStore()) {
            if (onceTime) {
                outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                clawToggle = true;
                if (whereAmI == PlacePosEnum.highSpecimen) {
                    outtakeSystem.setVSlidePos(Constants.Outtake.safeFromSpecBar);
                } else {
                    outtakeSystem.setVSlidePos(Constants.Outtake.safeFromClimberBar);
                }
                intakeSystem.setHSlidePos(Constants.Intake.transferSlides);
                onceState = true;

                onceTime = false;
            }
            if (Math.abs(outtakeSystem.getVSlidePos() - outtakeSystem.getVSlideTargetPos()) < 200 && onceState) {
                storeTime = elapsedTime.milliseconds();
                outtakeSystem.setArmPos(Constants.Outtake.intakeArm);
                onceState = false;
            }
            if (elapsedTime.milliseconds() > storeTime + 500 && elapsedTime.milliseconds() < storeTime + 600) {
                outtakeSystem.setVSlidePos(Constants.Outtake.intakeSlides);
            }
            if (elapsedTime.milliseconds() > storeTime + 700 && elapsedTime.milliseconds() < storeTime + 800) {
                outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
            }
            if (elapsedTime.milliseconds() > storeTime + 800 && elapsedTime.milliseconds() < storeTime + 900) {
                clawToggle = false;
                atStorePos = true;
                whereAmI = PlacePosEnum.intake;
                onceState = false;
                onceTime = true;
                stateMachine.finishGoToStoreFromSpec();
            }
        }

        if (stateMachine.doTransfer()) {
            if (onceTime) {
                outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                transferTime = elapsedTime.milliseconds();
                onceTime = false;
                transferFirstTime = true;
                if (!intakeSystem.readyToTransfer()) {
                    transferTime = elapsedTime.milliseconds() + Constants.Intake.intakeServoSpeedTime;
                }
                outtakeSystem.placePos(PlacePosEnum.intake);
            }
            if (elapsedTime.milliseconds() > transferTime + 0 && elapsedTime.milliseconds() < transferTime + 100 && transferFirstTime) {
                intakeSystem.setIntakePower(Constants.Intake.transferSpeed);
                outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
            }
            if (outtakeSystem.readyToTransfer() && intakeSystem.readyToTransfer() && transferFirstTime && elapsedTime.milliseconds() > transferTime + 100) {
                // if we have a piece
                transferFirstTime = false;
                transferTime = elapsedTime.milliseconds();
            }
            if (elapsedTime.milliseconds() > transferTime + 1500 && transferFirstTime) {
                // if failed
                intakeSystem.setIntakePower(0);
                hasInIntake = false;
//                hasInTray = true;
                stateMachine.failTransfer();
            }
            if (elapsedTime.milliseconds() > transferTime + 50 && elapsedTime.milliseconds() < transferTime + 150 && !transferFirstTime) {
                intakeSystem.setIntakePower(0);
                outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                clawToggle = true;
            }
            if (elapsedTime.milliseconds() > transferTime + 290 && elapsedTime.milliseconds() < transferTime + 390 && !transferFirstTime) {
                hasInIntake = false;
                hasInTray = false;
                hasInOuttake = true;
                onceState = true;
                onceTime = true;
                transferFirstTime = true;
                stateMachine.finishTransfer();
            }
        }

        if (stateMachine.doUnStore()) {
            if (onceTime) {
                if (stateMachine.goingHighBasket()) {
                    outtakeSystem.setVSlidePos(Constants.Outtake.highBasketSlides);
                } else {
                    outtakeSystem.setVSlidePos(Constants.Outtake.safeFromClimberBar);
                }
                intakeSystem.setIntakeServoPos(Constants.Intake.wristClear);
                outtakeSystem.setArmPos(Constants.Outtake.downArm);

                onceState = true;

                onceTime = false;
            }
            if (Math.abs(outtakeSystem.getVSlidePos() - Constants.Outtake.safeFromClimberBar) < 100 && onceState) {
                atStorePos = false;
                outtakeSystem.setArmPos(Constants.Outtake.initTeleopArm);
                intakeSystem.setIntakeServoPos(Constants.Intake.wristStore);
                intakeSystem.setHSlidePos(0);
                unStoringTime = elapsedTime.milliseconds();
                onceState = false;
            }
            if (elapsedTime.milliseconds() > unStoringTime + 200 && elapsedTime.milliseconds() < unStoringTime + 400) {
                atStorePos = false;
                onceState = true;
                onceTime = true;
                stateMachine.finishUnStore();
            }
        }

        if (stateMachine.doHighSpecimen()) {
            clawToggle = true;
            outtakeSystem.placePos(PlacePosEnum.highSpecimen);
            whereAmI = PlacePosEnum.highSpecimen;
            stateMachine.finishHighSpecimen();
        }

        if (stateMachine.doLowBasket()) {
            if (onceTime) {
                outtakeSystem.setVSlidePos(Constants.Outtake.lowBasketSlides);
                outtakeSystem.setArmPos(Constants.Outtake.upArm);
                whereAmI = PlacePosEnum.lowBasket;
                onceTime = false;
            }
            if (outtakeSystem.getVSlidePos() > outtakeSystem.getVSlideTargetPos() - 50) {
                outtakeSystem.setArmPos(Constants.Outtake.basketArmHigh);
                stateMachine.finishLowBasket();
                onceTime = true;
            }
        }

        if (stateMachine.doHighBasket()) {
            if (onceTime) {
                outtakeSystem.setVSlidePos(Constants.Outtake.highBasketSlides);
                outtakeSystem.setArmPos(Constants.Outtake.upArm);
                whereAmI = PlacePosEnum.highBasket;
                onceTime = false;
            }
            if (outtakeSystem.getVSlidePos() > outtakeSystem.getVSlideTargetPos() - 150) {
                outtakeSystem.setArmPos(Constants.Outtake.basketArmHigh);
                stateMachine.finishHighBasket();
                onceTime = true;
            }
        }

        if (stateMachine.doFrontBasket()) {
            if (onceTime) {
                outtakeSystem.setVSlidePos(Constants.Outtake.highBasketSlides);
                outtakeSystem.setArmPos(Constants.Outtake.upArm);
                whereAmI = PlacePosEnum.highBasket;
                onceTime = false;
            }
            if (outtakeSystem.getVSlidePos() > outtakeSystem.getVSlideTargetPos() - 150) {
                outtakeSystem.setArmPos(Constants.Outtake.frontBasketArm);
                stateMachine.finishFrontBasket();
                onceTime = true;
            }
        }

        if (stateMachine.doWall()) {
            if (onceTime) {
                outtakeSystem.placePos(PlacePosEnum.wall);
                whereAmI = PlacePosEnum.wall;
                if (!hasInOuttake) {
                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    clawToggle = false;
                }

                intakeSystem.storePos();
                wallTime = elapsedTime.milliseconds();
                onceTime = false;
            }
            //transferring into tray
            if (elapsedTime.milliseconds() > wallTime + 0 && elapsedTime.milliseconds() < wallTime + 100 && hasInIntake) {
                intakeSystem.setIntakePower(Constants.Intake.transferSpeed);
            }
            if (elapsedTime.milliseconds() > wallTime + 800 && elapsedTime.milliseconds() < wallTime + 900 && hasInIntake) {
                intakeSystem.setIntakePower(0);
                hasInIntake = false;
                hasInTray = true;
                onceTime = true;
                stateMachine.finishWall();
            }

        }


        // Claw Controls ************************************************************************~Ca
        if (!clawToggle) {
            hasInOuttake = false;
        }
        if (clawToggleButton && !clawDebounce) {
            clawToggle = !clawToggle;
            clawDebounce = true;

            // Grab
            if (clawToggle && whereAmI == PlacePosEnum.wall) {
                grabbingOffWall = true;
                wallOnce = true;
            } else if (clawToggle) {
                outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
            }

            // Drop
            if (!clawToggle && (whereAmI == PlacePosEnum.lowBasket || whereAmI == PlacePosEnum.highBasket)) {
                hasInOuttake = false;
                outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                droppingBasket = true;
                droppingBasketTime = elapsedTime.milliseconds();
            } else if (!clawToggle && whereAmI == PlacePosEnum.highSpecimen) {
                hasInOuttake = false;
                outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
            } else if (!clawToggle) {
                hasInOuttake = false;
                outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
            }
        }
        if (!clawToggleButton) {
            clawDebounce = false;
        }

        if (grabbingOffWall) {
            if (highSpecimen || highBasket || lowBasket || frontBasket) {
                grabbingOffWall = false;
            }
            if ((outtakeSystem.seesWall() || !clawToggleButton) && wallOnce) {
                grabbingOffWallTime = elapsedTime.milliseconds();
                wallOnce = false;
            }
            if (!clawToggleButton || (elapsedTime.milliseconds() > grabbingOffWallTime + 10) && !wallOnce && !hasInOuttake) {
                if (!clawToggleButton || outtakeSystem.seesWall()) {
                    outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                    hasInOuttake = true;
                } else {
                    // Restart
                    grabbingOffWallTime = elapsedTime.milliseconds();
                    wallOnce = true;
                }
            }
            if (elapsedTime.milliseconds() > grabbingOffWallTime + 250 && !wallOnce) {
                outtakeSystem.setVSlidePos(Constants.Outtake.safeFromWallSlides);
                grabbingOffWall = false;
                onceWall = true;
                wallOnce = true;
            }
        }

        if (droppingBasket) {
            if (clawToggleButton) {
                droppingBasketTime = elapsedTime.milliseconds();
            }
            if (elapsedTime.milliseconds() > droppingBasketTime + 300 && elapsedTime.milliseconds() < droppingBasketTime + 400) {
//                whereAmI = PlacePosEnum.highSpecimen;
                outtakeSystem.setArmPos(Constants.Outtake.upArm);
            }
            if (elapsedTime.milliseconds() > droppingBasketTime + 500 && elapsedTime.milliseconds() < droppingBasketTime + 600) {
                stateMachine.goStore();
            }
            // Fixes a bug
            if (elapsedTime.milliseconds() > droppingBasketTime + 600 && elapsedTime.milliseconds() < droppingBasketTime + 700) {
                if (intakeing || intakeingColor) {
                    intakeSystem.setHSlidePos(Constants.Intake.intakeSlidePos);
                }
                droppingBasket = false;
            }
        }

        //Intakes ********************************************************************************~I
//        if (gamepad1.touchpad_finger_1) {
//            extendHSlide = (int) ((gamepad1.touchpad_finger_1_x + 1)/2.0 * Constants.Intake.maxSlidePos);
//            if (gamepad1.touchpad) {
//                intakeSystem.setHSlidePos(extendHSlide);
//            }
//        }
        if (intakeColor && !intakeingDebounce) {
            downOnce = true;
            if (!intakeingColor) {
                intakeSystem.setHSlidePos(extendHSlide);
            }
            intakeSystem.setIntakeServoPos(Constants.Intake.wristClear);
            intakeSystem.setIntakePower(0);
            intakeingColor = true;
            intakeing = false;
            //do we want this?
            hasInTray = false;
            intakeWristTime = elapsedTime.milliseconds();
        }
        if (intake && !intakeingDebounce) {
            downOnce = true;
            if (!intakeing) {
                intakeSystem.setHSlidePos(extendHSlide);
            }
            intakeSystem.setIntakeServoPos(Constants.Intake.wristClear);
            intakeSystem.setIntakePower(0);
            intakeingColor = false;
            intakeing = true;
            //do we want this?
            hasInTray = false;
            intakeWristTime = elapsedTime.milliseconds();
        }
        // For intaking after down
        if (intake || intakeColor || hasInOuttake) {
            intakeTime = elapsedTime.milliseconds();
        }

        if (intakeing) {
            if (elapsedTime.milliseconds() > intakeTime + 300 && !hasInOuttake && intakeSystem.intakeUntil()) {
                afterIntakeing = true;
                afterIntakeingTime = elapsedTime.milliseconds();

                intakeSystem.setIntakePower(0);
                intakeing = false;
            }
        }

        if (intakeingColor) {
            if (elapsedTime.milliseconds() > intakeTime + 300 && intakeSystem.intakeUntilColor()) {
                afterIntakeingColor = true;
                afterIntakeingColorTime = elapsedTime.milliseconds();

                intakeSystem.setIntakePower(0);
                intakeingColor = false;
            }
        }

        if (afterIntakeing) {
            if (elapsedTime.milliseconds() > afterIntakeingTime + 200) {
                if (atStorePos) {
                    stateMachine.goTransfer(true);
                    intakeSystem.storeOutPos();
                } else {
                    intakeSystem.storePos();
                }
                gamepad1.rumble(1,1,1000);
                gamepad2.rumble(1,1,1000);
                extendHSlide = Constants.Intake.intakeSlidePos;
                hasInIntake = true;
                afterIntakeing = false;
                unjamAfterIntakeTime = elapsedTime.milliseconds();
                unjamAfterIntake = true;
            }
        }

        if (afterIntakeingColor) {
            if (elapsedTime.milliseconds() > afterIntakeingColorTime + 200) {
                if (atStorePos) {
                    intakeSystem.storeOutPos();
                } else {
                    intakeSystem.storePos();
                }
                gamepad1.rumble(1, 1, 300);
                gamepad2.rumble(1, 1, 300);
                extendHSlide = Constants.Intake.intakeSlidePos;
                hasInIntake = true;
                afterIntakeingColor = false;
                unjamAfterIntakeTime = elapsedTime.milliseconds();
                unjamAfterIntake = true;
            }
        }
        //makes the intake wrist not hit the robot while coming out
        if (((intakeingColor && !intakeColor) || (intakeing && !intake)) && !hasInOuttake && downOnce) {
            intakeingDebounce = false;
            intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
            intakeWristTime = 2000000000;
            downOnce = false;
        }

        //SIDE DEPOSIT***************************************************************************~SD
        if (sideDepo && !sideDepoDebounce && !intakeing && !intakeingColor && !unjamAfterIntake && !unjamming && !intakeColor && !intake) {
            sideDepoing = true;
            sideDepoFirst = true;
            sideDepoDebounce = true;
            sideDepoTime = elapsedTime.milliseconds();
            if (whereAmI != PlacePosEnum.wall) {
                outtakeSystem.setVSlidePos(Constants.Outtake.safeFromClimberBar);
            }
            intakeSystem.setDepoServoPos(Constants.Intake.depoStore);
            intakeSystem.setIntakeServoPos(Constants.Intake.wristStore);
            intakeSystem.setIntakePower(0);
        }
        if (!sideDepo) {
            sideDepoDebounce = false;
        }

        if (sideDepoing) {
            intakeSystem.setHSlidePos(40);
            if (whereAmI != PlacePosEnum.wall) {
                outtakeSystem.setVSlidePos(Constants.Outtake.safeFromClimberBar);
            }
            if (elapsedTime.milliseconds() < sideDepoTime + 200 && sideDepoFirst) {
                intakeSystem.setIntakePower(Constants.Intake.transferSpeed);
            }
            if (elapsedTime.milliseconds() > sideDepoTime + 500 && !sideDepo && sideDepoFirst) {
                sideDepoTime = elapsedTime.milliseconds();
                intakeSystem.setIntakeServoPos(Constants.Intake.wristDepo);
                sideDepoFirst = false;
            }
            if (elapsedTime.milliseconds() > sideDepoTime + 150 && elapsedTime.milliseconds() < sideDepoTime + 250 && !sideDepoFirst) {
                intakeSystem.setDepoServoPos(Constants.Intake.depoDepo);
            }
            if (elapsedTime.milliseconds() > sideDepoTime + 850 && !sideDepoFirst) {
                intakeSystem.setIntakePower(0);
                intakeSystem.setDepoServoPos(Constants.Intake.depoStore);
                intakeSystem.setIntakeServoPos(Constants.Intake.wristStore);
                hasInIntake = false;
                sideDepoing = false;
            }
            // Cancels it
            if (intake || intakeColor) {
                sideDepoing = false;
            }
        }

        //Unjamming intake ***********************************************************************~U
        if (unjamAfterIntake) {
            if (elapsedTime.milliseconds() < unjamAfterIntakeTime + Constants.Intake.unjamTimeMillisTeleop) {
                unJam = true;
            }
            if (elapsedTime.milliseconds() > unjamAfterIntakeTime + Constants.Intake.unjamTimeMillisTeleop) {
                unjamAfterIntake = false;
            }
        }

        if (unJam) {
            if (!intakeing && !intakeingColor) {
                intakeSystem.setIntakeServoPos(Constants.Intake.wristClear);
            }
            unjamming = true;
            unjammingTime = elapsedTime.milliseconds();

            intakeSystem.setIntakePower(Constants.Intake.unjamSpeed);
        }

        if (unjamming && !unJam) {
            if (!(intakeing || intakeingColor)) {
                if (atStorePos) {
                    intakeSystem.storeOutPos();
                } else {
                    intakeSystem.storePos();
                }
                intakeSystem.setIntakeServoPos(Constants.Intake.wristStore);
            }
            if (intakeSystem.intakeUntil()) {
                intakeSystem.setIntakePower(0);
                hasInIntake = true;
                unjamming = false;
            }
            if (elapsedTime.milliseconds() > unjammingTime + 800) {
                hasInIntake = false;
                intakeSystem.setIntakePower(0);
                unjamming = false;
            }
        }

//        if (elapsedTime.seconds() > 90 && elapsedTime.seconds() < 91) {
//            gamepad1.rumble(1000);
//            gamepad2.rumble(1000);
//        }

        //DEBUG **********************************************************************************~D
        if (debugState || debugAll) {
            telemetry.addLine("STATE");
            telemetry.addData("hasInIntake", hasInIntake);
            telemetry.addData("hasInTray", hasInTray);
            telemetry.addData("hasInOuttake", hasInOuttake);
            telemetry.addData("atStore", atStorePos);
            telemetry.addData("onceTime", onceTime);
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
            telemetry.addData("onceTime", onceTime);
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
        if (debugColor || debugAll) {
            NormalizedRGBA color = intakeSystem.directColor();
            telemetry.addData("r", color.red);
            telemetry.addData("g", color.green);
            telemetry.addData("b", color.blue);
            telemetry.addData("a", color.alpha);
            telemetry.addData("dis", intakeSystem.distance());
        }
        if (debugClimber || debugAll) {
            telemetry.addLine("CLIMBER");
            telemetry.addData("climberActive", climberActive);
            telemetry.addData("climberStage", climberStage);
            telemetry.addData("climbPause", climbPause);
            telemetry.addData("climbDebounce", climbDebounce);
            telemetry.addData("climberPos", climberPos);

            telemetry.addData("PTO Tar", driveTrain.getPTOTPos());
            telemetry.addData("PTO Pos", driveTrain.getPTOPos());
            telemetry.addData("PTO Err", Math.abs(driveTrain.getPTOPos() - driveTrain.getPTOTPos()));
            telemetry.addData("PTO Pow", Math.max(Math.max(driveTrain.getDrivePowers()[0], driveTrain.getDrivePowers()[1]), Math.max(driveTrain.getDrivePowers()[2], driveTrain.getDrivePowers()[3])));
            telemetry.addData("Climber Tar", climber.getTargetPos());
            telemetry.addData("Climber Var", climberPos);
            telemetry.addData("Climber Pos", climber.getPos());
            telemetry.addData("Climber err", Math.abs(climber.getPos() - climber.getTargetPos()));
            telemetry.addData("Climber once", climbStopPauseOnce);
            telemetry.addData("Climber once", climbStopPause);
            telemetry.addData("Current", Math.max(Math.max(driveTrain.getDriveCurrent()[0], driveTrain.getDriveCurrent()[1]), Math.max(driveTrain.getDriveCurrent()[2], driveTrain.getDriveCurrent()[3])));
            telemetry.addData("climberDistance", climber.getDistance());
            telemetry.addLine();
        }
        if (debugPower || debugAll) {
//            telemetry.addData("leftFront", motor1.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("rightFront", motor2.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("rightBack", motor3.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("leftBack", motor4.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("hSlides", motor5.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("vSlides", motor6.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("climber", motor7.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("intakeMotor", motor8.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("max", motor1.getCurrent(CurrentUnit.AMPS) + motor2.getCurrent(CurrentUnit.AMPS) + motor3.getCurrent(CurrentUnit.AMPS) + motor4.getCurrent(CurrentUnit.AMPS) + motor5.getCurrent(CurrentUnit.AMPS) + motor6.getCurrent(CurrentUnit.AMPS) + motor7.getCurrent(CurrentUnit.AMPS) + motor8.getCurrent(CurrentUnit.AMPS));
        }
        if (debugMisc || debugAll) {
            telemetry.addLine("MISCELLANEOUS");
            telemetry.addData("onceTime", onceTime);
            telemetry.addData("transferTime", transferTime);
            telemetry.addData("droppingBasketTime", droppingBasketTime);
            telemetry.addData("storeTime", storeTime);
            telemetry.addData("walltime", wallTime - elapsedTime.milliseconds());
            telemetry.addData("extendHSlide", extendHSlide);
            telemetry.addData("unjam", unJam);
            telemetry.addData("unjamming", unjamming);
            telemetry.addData("unjamAfterIntake", unjamAfterIntake);
            telemetry.addData("outtake distance", outtakeSystem.getClawDistance());
            telemetry.addData("grabbing off wall", grabbingOffWall);
            telemetry.addData("power", outtakeSystem.getAmp());
            telemetry.addData("outtakeSystem.readyToTransfer", outtakeSystem.readyToTransfer());
            telemetry.addData("intakeSystem.readyToTransfer",  intakeSystem.readyToTransfer());
            telemetry.addData("wallonce",  wallOnce);
            telemetry.addData("thing",  elapsedTime.milliseconds() > grabbingOffWallTime + 250);
            telemetry.addData("grabbingOffWallTime",  grabbingOffWallTime - elapsedTime.milliseconds());
            telemetry.addData("grabbingOffWall",  grabbingOffWall);

            //TODO add more Things here
            telemetry.addLine();
        }
        if (debugAll || debugClimber || debugMisc || debugPos || debugStateMachine || debugState || debugPower || debugColor) {
            telemetry.update();
        }
        if (false) {
            telemetry.addData("Loop Times ms", elapsedTime.milliseconds() - lastTime);
            lastTime = elapsedTime.milliseconds();
        }
    }

    public void stop() {
        driveTrain.stopDrive();
    }
}