package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Attempt89;
import org.firstinspires.ftc.teamcode.utility.SampleAutoEnum;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.BezierCurve;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Path;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Point;
import org.firstinspires.ftc.teamcode.pedropathing.util.Drawing;
import org.firstinspires.ftc.teamcode.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.PowerTakeOff;
import org.firstinspires.ftc.teamcode.utility.DriveSpeedEnum;
import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

public class SampleAuto {
    Telemetry telemetry;
    HardwareMap hardwareMap;
    RobotSideEnum robotSide;

    Climber climber;
    Follower follower;
    Drivetrain driveTrain;
    PowerTakeOff powerTakeOff;
    IntakeSystem intakeSystem;
    OuttakeSystem outtakeSystem;
    Attempt89 attempt89;

    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private SampleAutoEnum pathState;
    private int transferState = -1;

    // this variable will tell us if we failed to intake a sample
    private boolean intakeFail = false;

    /** Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left. But we don't want do so we don't, 0,0 is center off the field
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     **/

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(-40, -63, Math.toRadians(90));

    /** Scoring Poses of our robot. */
    private final Pose preloadPlace = new Pose(-56.5, -55.5, Math.toRadians(54));
    private final Pose intakeSpike1 = new Pose(-53.5, -51, Math.toRadians(81));
    private final Pose placeSpike1 = new Pose(-61.7, -53, Math.toRadians(80));

    private final Pose intakeSpike2 = new Pose(-59, -51.2, Math.toRadians(90));
    private final Pose placeSpike2 = new Pose(-62.3, -53, Math.toRadians(80));

    private final Pose intakeSpike3 = new Pose(-59, -49, Math.toRadians(117));
    private final Pose placeSpike3 = new Pose(-54.5, -56, Math.toRadians(45));

    private final Pose subIntake = new Pose(-23, -7.5, Math.toRadians(0));
    private final Pose subControlPointTo = new Pose(-57, -14, Math.toRadians(0));
    private final Pose subControlPointFrom = new Pose(-57, -14, Math.toRadians(0));
//    private final Pose subControlPointFrom = new Pose(-53, -14, Math.toRadians(0));

    private final Pose afterSubPlace = new Pose(-55, -56.5, Math.toRadians(45));

    private final Pose spikeSearch = new Pose(-59, -50.3, Math.toRadians(90));

    private Pose target = new Pose();
    private boolean driveShake = false;
    private boolean driveSweep = false;
    private boolean driveSee = false;
    private boolean transferSample = false;
    private boolean doIntakeWhilePark = false;


    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload;

    private Path intakeSpike1Path;
    private Path placeSpike1Path;

    private Path intakeSpike2Path;
    private Path placeSpike2Path;

    private Path intakeSpike3Path;
    private Path placeSpike3Path;

    private Path intakeSubPath;
    private Path placeSubPath;


    private Path failSpike1Path;
    private Path failSpike2Path;
    private Path failSpike3Path;

    public SampleAuto(HardwareMap hardwareMap, Telemetry telemetry, RobotSideEnum robotSide) {
        this.robotSide = robotSide;
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        climber = new Climber(hardwareMap);
        follower = new Follower(hardwareMap);
        driveTrain = new Drivetrain(hardwareMap);
        powerTakeOff = new PowerTakeOff(hardwareMap);
        intakeSystem = new IntakeSystem(hardwareMap, robotSide);
        outtakeSystem = new OuttakeSystem(hardwareMap, true);
        attempt89 = new Attempt89(hardwareMap, robotSide);
        attempt89.switchPipeline(0);

        outtakeSystem.setArmPos(Constants.Outtake.initAutoArm);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();

        opmodeTimer.resetTimer();

        follower.setStartingPose(startPose);

        buildPaths();
    }

    public void init_loop() {
        Pose2D sample = attempt89.getBestSample(robotSide);
        telemetry.addData("Test Block X", sample.getX(DistanceUnit.INCH));
        telemetry.addData("Test Block Y", sample.getY(DistanceUnit.INCH));
        telemetry.addData("Test Block H", sample.getHeading(AngleUnit.DEGREES));
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(SampleAutoEnum.scorePreload);
        attempt89.switchPipeline(0);
    }

    public void buildPaths() {
        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
//        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(placeSub1)));
//        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), placeSub1.getHeading());
        scorePreload = follower.linearPathBuilder(startPose, preloadPlace);

        intakeSpike1Path = follower.linearPathBuilder(preloadPlace, intakeSpike1);
        placeSpike1Path = follower.linearPathBuilder(intakeSpike1, placeSpike1);

        intakeSpike2Path = follower.linearPathBuilder(placeSpike1, intakeSpike2);
        placeSpike2Path = follower.linearPathBuilder(intakeSpike2, placeSpike2);

        intakeSpike3Path = follower.linearPathBuilder(placeSpike2, intakeSpike3);
        placeSpike3Path = follower.linearPathBuilder(intakeSpike3, placeSpike3);

        failSpike1Path = follower.linearPathBuilder(intakeSpike1, intakeSpike2);
        failSpike2Path = follower.linearPathBuilder(intakeSpike2, intakeSpike3);
        failSpike3Path = follower.linearPathBuilder(intakeSpike3, spikeSearch);

        intakeSubPath = new Path(new BezierCurve(new Point(placeSpike3), new Point(subControlPointTo), new Point(subIntake)));
        intakeSubPath.setLinearHeadingInterpolation(placeSpike3.getHeading(), subIntake.getHeading());
        intakeSubPath.setZeroPowerAccelerationMultiplier(7);

        placeSubPath = new Path(new BezierCurve(new Point(subIntake), new Point(subControlPointFrom), new Point(afterSubPlace)));
        placeSubPath.setLinearHeadingInterpolation(subIntake.getHeading(), afterSubPlace.getHeading());
    }
    public void autonomousPathUpdate() {
        //Driving and everything else
        switch (pathState) {
            case scorePreload:
                follower.setMaxPower(1);
                follower.followPath(scorePreload);
                outtakeSystem.setVSlidePos(Constants.Outtake.highBasketSlides);
                outtakeSystem.setArmPos(Constants.Outtake.intakeArm);
                setPathState(SampleAutoEnum.armClearing0);
                break;
            case armClearing0:
                if (outtakeSystem.getVSlidePos() > Constants.Outtake.safeFromClimberBar) {
                    outtakeSystem.setArmPos(Constants.Outtake.upArm);
                    setPathState(SampleAutoEnum.placePreload);
                }
            case placePreload:
                if (outtakeSystem.getVSlidePos() > outtakeSystem.getVSlideTargetPos() - 50) {
                    intakeSystem.setHSlidePos(Constants.Intake.autoPreExtendSlides);
                    outtakeSystem.setArmPos(Constants.Outtake.basketArm);
                    setPathState(SampleAutoEnum.dropClaw0);
                }
                break;
            case dropClaw0:
                if (pathTimer.getElapsedTimeSeconds() > 0.4) {
                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    setPathState(SampleAutoEnum.intakeSpike1);
                }
                break;
            case intakeSpike1:
                if (pathTimer.getElapsedTimeSeconds() > .3){
                    follower.followPath(intakeSpike1Path);

                    outtakeSystem.setArmPos(Constants.Outtake.intakeArm);
                    outtakeSystem.setVSlidePos(Constants.Outtake.intakeSlides);

                    intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
                    driveShake = true;
                    setPathState(SampleAutoEnum.armClearing1);
                }
                break;
            case armClearing1:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    intakeSystem.setHSlidePos(Constants.Intake.maxSlidePos);
                    setPathState(SampleAutoEnum.spike1Transfer);
                }
                break;
            case spike1Transfer:
                intakeSystem.update();
                if (intakeSystem.intakeUntil()){
                    driveShake = false;
                    follower.followPath(placeSpike1Path);
                    intakeSystem.storeOutPos();
                    intakeSystem.setIntakePower(Constants.Intake.unjamSpeed);
                    transferSample = true;
                    setPathState(SampleAutoEnum.placeSample);
                } else if (pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(failSpike1Path);
                    intakeSystem.setHSlidePos(Constants.Intake.minWhileDownPos);
                    setPathState(SampleAutoEnum.armClearing2);
                }
                break;
            case placeSample:
                if (!transferSample) {
                    intakeSystem.setHSlidePos(Constants.Intake.autoPreExtendSlides - 200);
                    setPathState(SampleAutoEnum.dropClaw1);
                }
                break;
            case dropClaw1:
                if (pathTimer.getElapsedTimeSeconds() > 0.4) {
                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
                    setPathState(SampleAutoEnum.intakeSpike2);
                }
                break;
            case intakeSpike2:
                if (pathTimer.getElapsedTimeSeconds() > .3) {
                    follower.followPath(intakeSpike2Path);
                    outtakeSystem.setArmPos(Constants.Outtake.intakeArm);
                    outtakeSystem.setVSlidePos(Constants.Outtake.intakeSlides);
                    driveShake = true;
                    setPathState(SampleAutoEnum.armClearing2);
                }
                break;
            case armClearing2:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    intakeSystem.setHSlidePos(Constants.Intake.maxSlidePos);
                    setPathState(SampleAutoEnum.spike2Transfer);
                }
                break;
            case spike2Transfer:
                intakeSystem.update();
                if (intakeSystem.intakeUntil()){
                    driveShake = false;
                    follower.followPath(placeSpike2Path);
                    intakeSystem.storeOutPos();
                    intakeSystem.setIntakePower(Constants.Intake.unjamSpeed);
                    transferSample = true;
                    setPathState(SampleAutoEnum.placeSample2);
                } else if (pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(failSpike2Path);
                    intakeSystem.setHSlidePos(Constants.Intake.minWhileDownPos);
                    setPathState(SampleAutoEnum.armClearing3);
                }
                break;
            case placeSample2:
                if (!transferSample) {
                    intakeSystem.setHSlidePos(Constants.Intake.autoPreExtendSlides - 100);
                    setPathState(SampleAutoEnum.dropClaw2);
                }
                break;
            case dropClaw2:
                if (pathTimer.getElapsedTimeSeconds() > .4) {
                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
                    setPathState(SampleAutoEnum.intakeSpike3);
                }
                break;
            case intakeSpike3:
                if (pathTimer.getElapsedTimeSeconds() > .3) {
                    follower.followPath(intakeSpike3Path);
                    outtakeSystem.setArmPos(Constants.Outtake.intakeArm);
                    outtakeSystem.setVSlidePos(Constants.Outtake.intakeSlides);

                    driveShake = true;
                    setPathState(SampleAutoEnum.armClearing3);
                }
                break;
            case armClearing3:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    intakeSystem.setHSlidePos(Constants.Intake.maxSlidePos);
                    setPathState(SampleAutoEnum.spike3Transfer);
                }
                break;
            case spike3Transfer:
                intakeSystem.update();
                if (intakeSystem.intakeUntil()){
                    driveShake = false;
                    follower.followPath(placeSpike3Path);
                    intakeSystem.storeOutPos();
                    intakeSystem.setIntakePower(Constants.Intake.unjamSpeed);
                    transferSample = true;
                    setPathState(SampleAutoEnum.placeSample3);
                } else if (pathTimer.getElapsedTimeSeconds() > 2) {
                    intakeSystem.storeOutPos();
                    driveShake = false;
                    setPathState(SampleAutoEnum.goSub1);
                }
                break;
            case placeSample3:
                if (!transferSample) {
                    driveShake = false;
                    setPathState(SampleAutoEnum.dropClaw3);
                }
                break;
            case dropClaw3:
                if (pathTimer.getElapsedTimeSeconds() > .4) {
                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    setPathState(SampleAutoEnum.intakeArm);
                }
                break;
            case intakeArm:
                if (pathTimer.getElapsedTimeSeconds() > .2) {
                    outtakeSystem.setArmPos(Constants.Outtake.intakeArm);
                    setPathState(SampleAutoEnum.goSub1);
                }
                break;
            //Loop Starts here
            case goSub1:
                if (pathTimer.getElapsedTimeSeconds() > .3) {
                    follower.followPath(intakeSubPath);
                    outtakeSystem.setVSlidePos(Constants.Outtake.intakeSlides);
                    setPathState(SampleAutoEnum.visionSearch1);
                }
                break;
            case visionSearch1:
                if (follower.getVelocityMagnitude() < 1 && follower.getVelocity().getYComponent() < 1) {
                    Pose2D target2D = attempt89.getBlockPosition(true);
                    telemetry.addData("X", target2D.getX(DistanceUnit.INCH));
                    telemetry.addData("Y", target2D.getY(DistanceUnit.INCH));
                    if (target2D.getHeading(AngleUnit.DEGREES) != -1) {
                        intakeSystem.setHSlidesInches(follower.followYourHead(target2D));
                        setPathState(SampleAutoEnum.subIntake1);
                        driveSweep = true;
                    }
                }
                break;
            case subIntake1:
                if (pathTimer.getElapsedTimeSeconds() > .35) {
                    intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.9) {
                    setPathState(SampleAutoEnum.transferSample4);
                }
                break;
            case transferSample4:
                intakeSystem.update();
                if (opmodeTimer.getElapsedTimeSeconds() > 28) {
                    //Break loop
                    setPathState(SampleAutoEnum.park);
                }
                if (pathTimer.getElapsedTimeSeconds() > 1.8) {
                    follower.breakFollowing();
                }
                if (intakeSystem.intakeUntil()) {
                    driveSweep = false;
                    follower.followPath(placeSubPath);
                    intakeSystem.storeOutPos();
                    intakeSystem.setIntakePower(Constants.Intake.unjamSpeed);
                    transferSample = true;
                    setPathState(SampleAutoEnum.placeSample4);
                }
                break;
            case placeSample4:
                if (!transferSample && follower.getError(afterSubPlace).getX() < 1 && follower.getError(afterSubPlace).getY() < 1 || pathTimer.getElapsedTimeSeconds() > 2) {
                    setPathState(SampleAutoEnum.dropClaw4);
                }
                break;
            case dropClaw4:
                if (pathTimer.getElapsedTimeSeconds() > 0.45) {
                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    if (opmodeTimer.getElapsedTimeSeconds() > 25) {
                        //Break loop
                        setPathState(SampleAutoEnum.park);
                    } else {
                        setPathState(SampleAutoEnum.intakeArm2);
                    }
                }
                break;
            case intakeArm2:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(intakeSubPath, false);
                    outtakeSystem.setArmPos(Constants.Outtake.intakeArm);
                    outtakeSystem.setVSlidePos(Constants.Outtake.intakeSlides);

                    setPathState(SampleAutoEnum.visionSearch1);
                    //Go loop
                }
                break;

            case park:
                if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                    doIntakeWhilePark = true;
                    follower.followPath(intakeSubPath);
                    outtakeSystem.setVSlidePos(Constants.Outtake.safeFromClimberBar);
                    setPathState(SampleAutoEnum.parkArm);
                }
                break;
            case parkArm:
                if (outtakeSystem.getVSlidePos() > Constants.Outtake.safeFromClimberBar - 50) {
                    outtakeSystem.setArmPos(Constants.Outtake.parkArm);

                    setPathState(SampleAutoEnum.parkSlides1);
                }
                break;
            case parkSlides1:
                if (pathTimer.getElapsedTimeSeconds() > 0.05) {
                    outtakeSystem.setVSlidePos(Constants.Outtake.safeFromHSlides + 200);

                    setPathState(SampleAutoEnum.parkSlides2);
                }
                break;
            case parkSlides2:
                if (pathTimer.getElapsedTimeSeconds() > 0.1 && follower.getError(subIntake).getX() < 2) {
                    outtakeSystem.setVSlidePos(0);

                    setPathState(SampleAutoEnum.end);
                }
                break;

            //when you fail a spike at the end you go here
            /*
            case failSpikeSequence:
                if (intakeFail) {
                    intakeSystem.storePos();
                    follower.followPath(failSpike3Path);
                    setPathState(SampleAutoEnum.failSpikeSearch);
                } else {
                    setPathState(SampleAutoEnum.goSub1);
                }
                break;
            case failSpikeSearch:
                if (follower.getError(spikeSearch).getX() < 1 && follower.getError(spikeSearch).getY() < 1) {
                    Pose2D target2D = attempt89.getBestSample();
                    driveSee = true;
                    if (target2D.getHeading(AngleUnit.DEGREES) != -1) {
                        driveSee = false;
                        follower.followYourHeart(target2D.getX(DistanceUnit.INCH));
                        intakeSystem.setHSlidesInches(target2D.getY(DistanceUnit.INCH));
                        setPathState(SampleAutoEnum.spikeSearchWrist);
                    } else if (pathTimer.getElapsedTimeSeconds() > 1) {
                        setPathState(SampleAutoEnum.goSub1);
                    }
                }
                break;
            case spikeSearchWrist:
                if (pathTimer.getElapsedTimeSeconds() > .35) {
                    intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
                    setPathState(SampleAutoEnum.spikeSearchIntake);
                }
                break;
            case spikeSearchIntake:
                if (intakeSystem.intakeUntil()) {
                    driveSweep = false;
                    follower.followPath(placeSpike2Path);
                    intakeSystem.storeOutPos();
                    intakeSystem.setIntakePower(Constants.Intake.unjamSpeed);
                    transferSample = true;
                    setPathState(SampleAutoEnum.placeSample3);
                } else if (pathTimer.getElapsedTimeSeconds() > 0.75) {
                    driveSweep = false;
                    intakeSystem.storePos();
                    follower.followPath(intakeSubPath);
                    intakeSystem.setIntakePower(Constants.Intake.unjamSpeed);
                    setPathState(SampleAutoEnum.visionSearch1);
                }
                break;
             */
        }
        //while parking we want to be able to intake
        if (doIntakeWhilePark) {
            if (intakeSystem.intakeUntil()) {
                intakeSystem.storePos();
                driveSweep = false;
                doIntakeWhilePark = false;
            }
        }
        //Transfering
        if (transferSample) {
            switch (transferState) {
                case -1:
                    actionTimer.resetTimer();
                    transferState = 0;
                    break;
                case 0:
                    if (actionTimer.getElapsedTimeSeconds() > Constants.Intake.unjamTimeMillisAuto / 1000.0) {
                        intakeSystem.setIntakePower(Constants.Intake.intakeSpeed);
                        setTransferState(1);
                    }
                    break;
                case 1:
                    if (intakeSystem.intakeUntil() || actionTimer.getElapsedTimeSeconds() > 0.5) {
                        intakeSystem.setIntakePower(0);
                        outtakeSystem.setClawPos(Constants.Outtake.dropClaw);

                        setTransferState(2);
                    }
                    break;
                case 2:
                    if (actionTimer.getElapsedTimeSeconds() > .3){
                        intakeSystem.setIntakePower(Constants.Intake.transferSpeed);

                        setTransferState(4);
                    }
                    break;
                case 4:
                    if ((outtakeSystem.readyToTransfer() && intakeSystem.readyToTransfer()) || actionTimer.getElapsedTimeSeconds() > 2){
                        intakeSystem.setIntakePower(0);
                        intakeSystem.setIntakeServoPos(Constants.Intake.wristClear);
                        outtakeSystem.setClawPos(Constants.Outtake.grabClaw);

                        setTransferState(5);
                    }
                    break;
                case 5:
                    if (actionTimer.getElapsedTimeSeconds() > .3) {
                        outtakeSystem.setVSlidePos(Constants.Outtake.highBasketSlides);

                        setTransferState(6);
                    }
                    break;
                case 6:
                    if (outtakeSystem.getVSlidePos() > Constants.Outtake.safeFromClimberBar) {
                        outtakeSystem.setArmPos(Constants.Outtake.upArm);

                        intakeSystem.setIntakeServoPos(Constants.Intake.wristStore);
                        setTransferState(7);
                    }
                    break;
                case 7:
                    if (outtakeSystem.getVSlidePos() > outtakeSystem.getVSlideTargetPos() - 200) {
                        outtakeSystem.setArmPos(Constants.Outtake.basketArm);

                        transferSample = false;
                        setTransferState(-1);
                    }
            }
        }
    }

    public void setPathState(SampleAutoEnum pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    public void setTransferState(int tState) {
        transferState = tState;
        actionTimer.resetTimer();
    }

    public void loop() {
        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();
        if (driveShake && pathTimer.getElapsedTimeSeconds() > 1.1) {
            if (!follower.isBusy()) {
                follower.breakFollowing();
            }
            if (Math.round((pathTimer.getElapsedTimeSeconds() - 1.1) * 2.5) % 2 == 0 ){
                driveTrain.drive(0,0, 0.8, DriveSpeedEnum.Auto);
            } else {
                driveTrain.drive(0,0, -0.8, DriveSpeedEnum.Auto);
            }
        }

        if (driveSweep && pathTimer.getElapsedTimeSeconds() > 1.5) {
            if (!follower.isBusy()) {
                follower.breakFollowing();
            }
            if (Math.round((pathTimer.getElapsedTimeSeconds() - 0.9) * .5) % 2 == 0 ){
                driveTrain.drive(0,0.5, 0, DriveSpeedEnum.Auto);
            } else {
                driveTrain.drive(0,-0.5, 0, DriveSpeedEnum.Auto);
            }
        }

        if (driveSee && pathTimer.getElapsedTimeSeconds() > 1) {
            if (!follower.isBusy()) {
                follower.breakFollowing();
            }
            if (Math.round((pathTimer.getElapsedTimeSeconds() - 0.9) * .75) % 2 == 0 ){
                driveTrain.drive(0.05,0, 0.2, DriveSpeedEnum.Auto);
            } else {
                driveTrain.drive(0.05,0, -0.2, DriveSpeedEnum.Auto);
            }
        }

        // Feedback to FTC Dashboard
        Drawing.drawDebug(follower);

        // Feedback to Driver Hub
        if (true) {
            telemetry.addData("state", pathState);
            telemetry.addData("target", target);
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            follower.telemetryDebug(telemetry);
            telemetry.update();
        }
    }
}
