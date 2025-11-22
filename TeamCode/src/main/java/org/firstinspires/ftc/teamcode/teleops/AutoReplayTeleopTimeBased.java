package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.autos.AutoReplay;
import org.firstinspires.ftc.teamcode.autos.AutoReplayTime;
import org.firstinspires.ftc.teamcode.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.MathFunctions;
import org.firstinspires.ftc.teamcode.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.PowerTakeOff;
import org.firstinspires.ftc.teamcode.utility.DriveSpeedEnum;
import org.firstinspires.ftc.teamcode.utility.PlacePosEnum;
import org.firstinspires.ftc.teamcode.utility.Robot;
import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;
import org.firstinspires.ftc.teamcode.utility.RobotStateVariables;
import org.firstinspires.ftc.teamcode.utility.StateMachine;

public class AutoReplayTeleopTimeBased {
    //comment me out V
//    DcMotorEx motor1, motor2, motor3, motor4, motor5, motor6, motor7, motor8;
    Robot robot;
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
    boolean debugPos = true;
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
    boolean backFlapButton;
    boolean climbToggButton;

    double manualVSlide;
    double manualHSlide;
    double manualWrist;
    double manualArm;
    double manualClimber;

    boolean leftStickToggle = false;
    boolean rightStickToggle = false;

    boolean leftStickToggleDebounce = false;
    boolean rightStickToggleDebounce = false;

    boolean gamepad1B;

    boolean sideDepo;

    boolean highSpecimen;
    boolean lowSpecimen;
    boolean highBasket;
    boolean lowBasket;
    boolean frontBasket;
    boolean wallPreset;
    boolean storePos;
    boolean transfer;

    boolean clawToggleButton;
    boolean clawCancel;

    boolean intake;
    boolean intakeColor;
    boolean unJam;

    //State Machine Variables
    RobotStateVariables robotState;

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

    boolean onceWall = true;
    boolean onceState = true;
    boolean wallOnce = true;
    double storeTime = 2000000000;
    double transferTime = 2000000000;
    double unStoringTime = 2000000000;
    double wallTime = 2000000000;
    double lastTime = 2000000000;

    boolean transferFirstTime = true;

    boolean clawDebounce = false;

    boolean grabbingOffWall = false;
    double grabbingOffWallTime = 2000000000;
    boolean droppingBasket = false;
    boolean droppingSpec = false;
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

    double prevFlapPos = 0;
    boolean backFlapDebounce = false;

    Gamepad gamepad1;
    Gamepad gamepad2;

    //this is here because I have to have a teleop blue and teleop red
    HardwareMap hardwareMap;
    Follower follower;
    Telemetry telemetry;
    Gamepad gamepadInfo1;
    Gamepad gamepadInfo2;
    RobotSideEnum robotSide;
    AutoReplayTime autoReplay;

    public AutoReplayTeleopTimeBased(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, RobotSideEnum robotSide) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepadInfo1 = gamepad1;
        this.gamepadInfo2 = gamepad2;
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
        follower = new Follower(hardwareMap);

        climber = new Climber(hardwareMap);
        driveTrain = new Drivetrain(hardwareMap);
        stateMachine = new StateMachine();
        robotState = new RobotStateVariables(PlacePosEnum.clear, robotSide);
        autoReplay = new AutoReplayTime(follower, telemetry, gamepadInfo1, gamepadInfo2, driveTrain);
        follower.setStartingPose(new Pose(0, 0, 0));

        autoReplay.init();
    }

    public void start() {
        elapsedTime.reset();
        intakeSystem = new IntakeSystem(hardwareMap, robotSide);
        powerTakeOff = new PowerTakeOff(hardwareMap);
        outtakeSystem = new OuttakeSystem(hardwareMap, robotSide, true);
        outtakeSystem.setArmPos(Constants.Outtake.upArm);

        robot = new Robot(climber, telemetry, driveTrain, powerTakeOff, intakeSystem, stateMachine, outtakeSystem, robotState, false);
    }


    public void loop() {
        follower.update();

        // Inputs
        gamepad1B = gamepadInfo1.b;

        debugAll = gamepadInfo1.a;
        driveForward = -gamepadInfo1.left_stick_y;
        driveStrafe = -gamepadInfo1.left_stick_x;
        driveRotation = -gamepadInfo1.right_stick_x;

        autoReplay.update();

        if (autoReplay.IsReplayOn()){
            gamepad1 = autoReplay.getGamepad1();
            gamepad2 = autoReplay.getGamepad2();
        }
        else{
            gamepad1 = gamepadInfo1;
            gamepad2 = gamepadInfo2;
        }

        if (gamepad1.right_bumper) {
            driveSpeed = DriveSpeedEnum.Fast;
        } else {
            driveSpeed = DriveSpeedEnum.Slow;
        }

        backFlapButton = gamepad1.right_stick_button;

        climbToggButton = gamepad1.back;

        if (!climberActive) {
            if (gamepad2.left_stick_button && !leftStickToggleDebounce) {
                leftStickToggle = !leftStickToggle;
                rightStickToggle = false;
                leftStickToggleDebounce = true;
            }
            if (gamepad2.right_stick_button && !rightStickToggleDebounce) {
                rightStickToggle = !rightStickToggle;
                leftStickToggle = false;
                rightStickToggleDebounce = true;
            }
            if (!gamepad2.left_stick_button) {
                leftStickToggleDebounce = false;
            }
            if (!gamepad2.right_stick_button) {
                rightStickToggleDebounce = false;
            }

            if (robot.robotState.whereAmI == PlacePosEnum.highSpecimen || robot.robotState.whereAmI == PlacePosEnum.lowSpecimen) {
                manualHSlide = gamepad2.right_trigger + (0.6 * (-gamepad2.left_trigger + gamepad1.right_trigger));
            } else {
                manualHSlide = gamepad2.right_trigger + (0.6 * (-gamepad2.left_trigger + gamepad1.right_trigger - gamepad1.left_trigger));
            }
            if (debugAll) {
                manualHSlide = manualHSlide * 2;
            }

            if (leftStickToggle) {
                manualWrist = gamepad2.left_stick_y;
                manualArm = 0;
                manualVSlide = -gamepad2.right_stick_y;
            } else if (rightStickToggle) {
                manualWrist = gamepad2.right_stick_y;
                manualArm = gamepad2.left_stick_y;
                manualVSlide = 0;
            } else {
                manualWrist = 0;
                manualArm = gamepad2.left_stick_y;
                manualVSlide = -gamepad2.right_stick_y;
            }

            manualClimber = 0;
        } else {
            manualHSlide = 0;
            manualArm = 0;
            manualWrist = 0;
            manualVSlide = 0;
        }

        if (robotState.whereAmI == PlacePosEnum.highBasket || robotState.whereAmI == PlacePosEnum.lowBasket) {
            manualArm *= 0.5;
            manualVSlide *= 0.5;
            manualWrist *= 0.5;
        }

        sideDepo = (gamepad2.touchpad && gamepad2.touchpad_finger_1_x > 0) || (gamepad2.dpad_left && robotState.hasInIntake);

        highSpecimen = gamepad2.dpad_up;
        lowSpecimen = gamepad2.dpad_down && (robotState.whereAmI == PlacePosEnum.wall || robotState.whereAmI == PlacePosEnum.highSpecimen);
        highBasket = gamepad2.dpad_right;
        lowBasket = gamepad2.dpad_down && (robotState.whereAmI == PlacePosEnum.intake || robotState.whereAmI == PlacePosEnum.highBasket);
        frontBasket = gamepad2.touchpad && gamepad2.touchpad_finger_1_x < 0;
        wallPreset = gamepad2.dpad_left;
        storePos = gamepad2.y;//triangle
        transfer = gamepad2.left_bumper;

        clawToggleButton = gamepad1.left_bumper || gamepad2.right_bumper;
        clawCancel = gamepad1.left_trigger > 0.1;

        intake = gamepad2.a;//cross
        intakeColor = gamepad2.x;//square
        unJam = gamepad2.b;//circle

        //Drivetrain *****************************************************************************~D
        if (backFlapButton && !backFlapDebounce) {
            prevFlapPos = outtakeSystem.getBackFlapsPos();
            outtakeSystem.setFlapsSpikeClear();
            backFlapDebounce = true;
        }
        if (!backFlapButton && backFlapDebounce) {
            outtakeSystem.setBackFlaps(prevFlapPos);
            backFlapDebounce = false;
        }

        if (!climberActive && !climbPause && !autoReplay.IsReplayOn()) {
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
            if ((climbToggButton || gamepad1B)  && !climbDebounce && climberStage >= 2) {
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
                    if (current > 4.5) {
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
            manualClimber = gamepad1.right_trigger - gamepad1.left_trigger;
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
            if (gamepad1.y) {
                powerTakeOff.hold();
            }
            if (gamepad1.x) {
                powerTakeOff.release();
            }
        }
        // Pre-Start Climb
        if (gamepad2.back) {
            climberPos = Constants.Climber.prePos;
            climber.setPos(climberPos);
        }

        //Manual Movements ***********************************************************************~M
        intakeSystem.manualHSlide(manualHSlide);
        outtakeSystem.manualArm(manualArm);
        outtakeSystem.manualVSlide(manualVSlide);

        outtakeSystem.manualWrist(manualWrist);


        intakeSystem.update();
        outtakeSystem.update(gamepad2.ps);

        //Presets
        //Button to State Machine class *********************************************************~BS
        if (lowSpecimen) {
            outtakeSystem.setFlapsUp();
            intakeSystem.storePos();
            stateMachine.goLowSpecimen(robotState.whereAmI == PlacePosEnum.intake);
            stateMachine.setAutoPresets(true);
        }
        if (highSpecimen) {
            outtakeSystem.setFlapsUp();
            intakeSystem.storePos();
            stateMachine.goHighSpecimen(robotState.whereAmI == PlacePosEnum.lowSpecimen, robotState.whereAmI == PlacePosEnum.intake);
        }
        if (lowBasket) {
            outtakeSystem.setFlapsUp();
            intakeingColor = false;
            intakeing = false;
            stateMachine.goLowBasket(robotState.hasInIntake, robotState.hasInOutake, robotState.whereAmI == PlacePosEnum.lowSpecimen, robotState.whereAmI == PlacePosEnum.intake);
        }
        if (highBasket) {
            outtakeSystem.setFlapsUp();
            intakeingColor = false;
            intakeing = false;
            stateMachine.goHighBasket(robotState.hasInIntake, robotState.hasInOutake, robotState.whereAmI == PlacePosEnum.lowSpecimen, robotState.whereAmI == PlacePosEnum.intake);
        }
        if (frontBasket) {
            outtakeSystem.setFlapsUp();
            intakeingColor = false;
            intakeing = false;
            stateMachine.goFrontBasket(robotState.hasInIntake, robotState.hasInOutake, robotState.whereAmI == PlacePosEnum.lowSpecimen, robotState.whereAmI == PlacePosEnum.intake);
        }
        if (wallPreset) {
            outtakeSystem.setFlapsWall();
            intakeingColor = false;
            intakeing = false;  //Todo change v if you want to transfer
            stateMachine.goWall(robot.robotState.hasInIntake, robotState.whereAmI == PlacePosEnum.lowSpecimen, robotState.whereAmI == PlacePosEnum.intake);
        }
        if (storePos) {
            outtakeSystem.setFlapsUp();
            stateMachine.resetValues();
            intakeingColor = false;
            intakeing = false;
            stateMachine.goStore();
            extendHSlide = Constants.Intake.intakeSlidePos;
        }
        if (transfer) {
            outtakeSystem.setFlapsUp();
            intakeSystem.storeOutPos();
            intakeingColor = false;
            intakeing = false;
            stateMachine.goTransfer(robotState.whereAmI == PlacePosEnum.intake);
        }


        //State Machine class to movement *******************************************************~SM
        robot.loop();

        if (stateMachine.doWall()) {
            if (!intake && !intakeColor) {
                intakeSystem.storePos();
            }
        }
        // Claw Controls ************************************************************************~Ca
        if (!robotState.clawToggle) {
            robotState.hasInOutake = false;
        }
        if (clawToggleButton && !clawDebounce) {
            robotState.clawToggle = !robotState.clawToggle;
            clawDebounce = true;

            // Grab
            if (robotState.clawToggle && robotState.whereAmI == PlacePosEnum.wall) {
                grabbingOffWall = true;
                wallOnce = true;
            } else if (robotState.clawToggle) {
                outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
            }

            // Drop
            if (!robotState.clawToggle && (robotState.whereAmI == PlacePosEnum.lowBasket || robotState.whereAmI == PlacePosEnum.highBasket)) {
                robotState.hasInOutake = false;
                outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                droppingBasket = true;
                droppingBasketTime = elapsedTime.milliseconds();
            } else if (!robotState.clawToggle && (robotState.whereAmI == PlacePosEnum.highSpecimen || robotState.whereAmI == PlacePosEnum.lowSpecimen)) {
                droppingSpec = true;
            } else if (!robotState.clawToggle) {
                robotState.hasInOutake = false;
                outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
            }
        }
        if (!clawToggleButton) {
            clawDebounce = false;
        }

        if (droppingSpec) {
            if (robot.robotState.whereAmI == PlacePosEnum.highSpecimen) {
                robot.outtakeSystem.placePos(PlacePosEnum.postClipHighSpecimen);
            } else {
                robot.outtakeSystem.placePos(PlacePosEnum.postClipLowSpecimenAuto);
            }
            if (!clawToggleButton) {
                outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                droppingSpec = false;
            }
            if (clawCancel) {
                if (robot.robotState.whereAmI == PlacePosEnum.highSpecimen) {
                    robot.outtakeSystem.placePos(PlacePosEnum.highSpecimen);
                } else {
                    robot.outtakeSystem.placePos(PlacePosEnum.preClipLowSpecimenAuto);
                }
                droppingSpec = false;
                robotState.clawToggle = true;
            }
        }

        if (grabbingOffWall) {
            if (highSpecimen || lowSpecimen || highBasket || lowBasket || frontBasket) {
                grabbingOffWall = false;
            }
            if ((outtakeSystem.seesWall() || !clawToggleButton) && wallOnce) {
                grabbingOffWallTime = elapsedTime.milliseconds();
                wallOnce = false;
            }
            if (!clawToggleButton || (elapsedTime.milliseconds() > grabbingOffWallTime + 10) && !wallOnce && !robotState.hasInOutake) {
                if (!clawToggleButton || outtakeSystem.seesWall()) {
                    outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                    robotState.hasInOutake = true;
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
//                robotState.whereAmI = PlacePosEnum.highSpecimen;
                outtakeSystem.setArmPos(Constants.Outtake.intakeArm);
            }
            if (elapsedTime.milliseconds() > droppingBasketTime + 500 && elapsedTime.milliseconds() < droppingBasketTime + 600) {
                stateMachine.goStore();
                if (intakeing || intakeingColor) {
                    stateMachine.setBringSlidesIn(false);
                }
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
            if (robot.stateMachine.doGoToStore()) {
                robot.stateMachine.setBringSlidesIn(false);
            }
            outtakeSystem.setFlapsUp();
            downOnce = true;
            if (!intakeingColor) {
                intakeSystem.setHSlidePos(extendHSlide);
            }
            intakeSystem.setIntakeServoPos(Constants.Intake.wristClear);
            intakeSystem.setIntakePower(0);
            intakeingColor = true;
            intakeing = false;
            intakeWristTime = elapsedTime.milliseconds();
        }
        if (intake && !intakeingDebounce) {
            if (robot.stateMachine.doGoToStore()) {
                robot.stateMachine.setBringSlidesIn(false);
            }
            outtakeSystem.setFlapsUp();
            downOnce = true;
            if (!intakeing) {
                intakeSystem.setHSlidePos(extendHSlide);
            }
            intakeSystem.setIntakeServoPos(Constants.Intake.wristClear);
            intakeSystem.setIntakePower(0);
            intakeingColor = false;
            intakeing = true;
            //do we want this?
            intakeWristTime = elapsedTime.milliseconds();
        }
        // For intaking after down
        if (intake || intakeColor || robotState.hasInOutake) {
            intakeTime = elapsedTime.milliseconds();
        }

        if (intakeing) {
            if (elapsedTime.milliseconds() > intakeTime + 300 && !robotState.hasInOutake && intakeSystem.intakeUntil()) {
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
                if (robotState.whereAmI == PlacePosEnum.intake) {
                    stateMachine.goTransfer(true);
                    intakeSystem.storeOutPos();
                } else {
                    intakeSystem.storePos();
                }
                gamepad1.rumble(1,1,1000);
                gamepad2.rumble(1,1,1000);
                extendHSlide = Constants.Intake.intakeSlidePos;
                robotState.hasInIntake = true;
                afterIntakeing = false;
                unjamAfterIntakeTime = elapsedTime.milliseconds();
                unjamAfterIntake = true;
            }
        }

        if (afterIntakeingColor) {
            if (elapsedTime.milliseconds() > afterIntakeingColorTime + 200) {
                if (robotState.whereAmI == PlacePosEnum.intake) {
                    stateMachine.goWall(true, false, true);
                    intakeSystem.storeOutPos();
                } else {
                    intakeSystem.storePos();
                }
                gamepad1.rumble(1, 1, 300);
                gamepad2.rumble(1, 1, 300);
                extendHSlide = Constants.Intake.intakeSlidePos;
                robotState.hasInIntake = true;
                afterIntakeingColor = false;
                unjamAfterIntakeTime = elapsedTime.milliseconds();
                unjamAfterIntake = true;
            }
        }
        //makes the intake wrist not hit the robot while coming out
        if (((intakeingColor && !intakeColor) || (intakeing && !intake)) && !robotState.hasInOutake && downOnce) {
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
            if (robotState.whereAmI != PlacePosEnum.wall) {
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
            // Cancels it
            if (intake || intakeColor) {
                sideDepoing = false;
            }
            intakeSystem.setHSlidePos(40);
            if (robotState.whereAmI != PlacePosEnum.wall) {
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
                robotState.hasInIntake = false;
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
                if (robotState.whereAmI == PlacePosEnum.intake) {
                    intakeSystem.storeOutPos();
                } else {
                    intakeSystem.storePos();
                }
            }
            if (intakeSystem.intakeUntil()) {
                intakeSystem.setIntakePower(0);
                robotState.hasInIntake = true;
                unjamming = false;
            }
            if (elapsedTime.milliseconds() > unjammingTime + 800) {
                robotState.hasInIntake = false;
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
            telemetry.addData("robotState.hasInIntake", robotState.hasInIntake);
            telemetry.addData("robotState.hasInOutake", robotState.hasInOutake);
            telemetry.addData("atStore", robotState.whereAmI == PlacePosEnum.intake);
            telemetry.addData("where am I", robotState.whereAmI);
            telemetry.addLine();
        }
        if (debugStateMachine || debugAll) {
            telemetry.addLine("STATE MACHINE");
            telemetry.addData("doGoToStore", stateMachine.doGoToStore());
            telemetry.addData("doTransfer", stateMachine.doTransfer());
            telemetry.addData("doUnStoreFromIntake", stateMachine.doUnStoreFromIntake());
            telemetry.addData("doUnStoreFromLowBar", stateMachine.doUnStoreFromLowBar());
            telemetry.addData("doHighBasket", stateMachine.doHighBasket());
            telemetry.addData("doLowBasket", stateMachine.doLowBasket());
            telemetry.addData("doHighSpecimen", stateMachine.doHighSpecimen());
            telemetry.addData("doWall", stateMachine.doWall());
            telemetry.addData("robotState.hasInIntake", stateMachine.debug()[0]);
            telemetry.addData("transferred", stateMachine.debug()[1]);
            telemetry.addData("atStore", stateMachine.debug()[2]);
            telemetry.addData("lowSpec", stateMachine.debug()[3]);
            telemetry.addData("highBasket", stateMachine.debug()[4]);
            telemetry.addLine();
        }
        if (debugPos || debugAll) {
            telemetry.addLine("POSITION");
            telemetry.addData("V Slide Tar", outtakeSystem.getVSlideTargetPos());
            telemetry.addData("V Slide Pos", outtakeSystem.getVSlidePos());
            telemetry.addData("H Slide Tar", intakeSystem.getHSlideTargetPos());
            telemetry.addData("H Slide Pos", intakeSystem.getHSlidePos());
            telemetry.addData("Arm Pos", outtakeSystem.getArmPos());
            telemetry.addData("Wrist Pos", outtakeSystem.getWristPos());
            telemetry.addLine();
        }
        if (debugColor || debugAll) {
            NormalizedRGBA color = intakeSystem.directColor();
            telemetry.addData("r", color.red);
            telemetry.addData("g", color.green);
            telemetry.addData("b", color.blue);
            telemetry.addData("a", color.alpha);
            telemetry.addData("dis", intakeSystem.distance());
            telemetry.addLine();
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
            telemetry.addData("left ",  leftStickToggle);
            telemetry.addData("right",  rightStickToggle);

            telemetry.addData("claw toggle", robot.robotState.clawToggle);
            telemetry.addData("claw debounce", clawDebounce);
            telemetry.addData("clawToggleButton", clawToggleButton);
            telemetry.addData("clawCancel", clawCancel);
            telemetry.addData("droppingSpec", droppingSpec);

            //TODO add more Things here
            telemetry.addLine();
        }
        if (debugAll || debugClimber || debugMisc || debugPos || debugStateMachine || debugState || debugPower || debugColor) {
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