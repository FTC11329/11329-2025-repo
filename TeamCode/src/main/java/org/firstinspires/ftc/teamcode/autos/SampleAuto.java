package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Attempt89;
import org.firstinspires.ftc.teamcode.utility.PlacePosEnum;
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
import org.firstinspires.ftc.teamcode.utility.Specimen6AutoEnum;

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
    Attempt89 blockVision;

    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private SampleAutoEnum pathState;
    private int transferState = -1;

    // this variable will tell us if we failed to intake a sample
    private boolean missedSpike = false;

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
    private final Pose preloadPlace = new Pose(-55.6, -54.6, Math.toRadians(54));
    private final Pose intakeSpike1 = new Pose(-52, -51, Math.toRadians(81));
    private final Pose placeSpike1 = new Pose(-60.5, -52.7, Math.toRadians(80));

    private final Pose intakeSpike2 = new Pose(-57.9, -51.2, Math.toRadians(90));
    private final Pose placeSpike2 = new Pose(-60, -52.7, Math.toRadians(80));

    private final Pose intakeSpike3 = new Pose(-58, -49, Math.toRadians(115));
    private final Pose placeSpike3 = new Pose(-54.2, -55.7, Math.toRadians(45));

    private final Pose subIntake = new Pose(-23, -6, Math.toRadians(0));
    private final Pose subControlPoint = new Pose(-48.4, -10.3, Math.toRadians(0));

    private final Pose afterSubPlace = new Pose(-53.25, -54.75, Math.toRadians(45));

    private final Pose spikeSearch = new Pose(-59, -40, Math.toRadians(40));

    private Pose2D target2D;
    private boolean driveShake = false;
    private boolean driveSee = false;
    private boolean transferSample = false;
    private boolean doIntakeWhilePark = false;
    private int parkState = 0;


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
    private Path parkPath; // IS THAT A BTD 6 REFERANCE?!?!?!?


    private Path failSpike1Path;
    private Path failSpike2Path;
    private Path toSearchSpikes;

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
        outtakeSystem = new OuttakeSystem(hardwareMap, robotSide, true);
        blockVision = new Attempt89(hardwareMap, robotSide);
        blockVision.switchPipeline(0);

        outtakeSystem.setArmPos(Constants.Outtake.initAutoSampArm);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();

        opmodeTimer.resetTimer();

        follower.setStartingPose(startPose);

        buildPaths();
    }

    public void init_loop() {
        Pose2D sample = blockVision.getBestSample(robotSide);
        telemetry.addData("Test Block X", sample.getX(DistanceUnit.INCH));
        telemetry.addData("Test Block Y", sample.getY(DistanceUnit.INCH));
        telemetry.addData("Test Block H", sample.getHeading(AngleUnit.DEGREES));
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(SampleAutoEnum.scorePreload);
        blockVision.switchPipeline(0);
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

        toSearchSpikes = follower.linearPathBuilder(placeSpike3, spikeSearch);

        intakeSubPath = new Path(new BezierCurve(new Point(placeSpike3), new Point(subControlPoint), new Point(subIntake)));
        intakeSubPath.setLinearHeadingInterpolation(placeSpike3.getHeading(), subIntake.getHeading());
        intakeSubPath.setZeroPowerAccelerationMultiplier(13);

        placeSubPath = new Path(new BezierCurve(new Point(subIntake), new Point(subControlPoint), new Point(afterSubPlace)));
        placeSubPath.setLinearHeadingInterpolation(subIntake.getHeading(), afterSubPlace.getHeading());

        parkPath = new Path(new BezierCurve(new Point(subIntake), new Point(subControlPoint), new Point(afterSubPlace)));
        parkPath.setLinearHeadingInterpolation(subIntake.getHeading(), afterSubPlace.getHeading());
        parkPath.setZeroPowerAccelerationMultiplier(4);
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
                    intakeSystem.setHSlidePos(Constants.Intake.autoPreExtendSlides - 225);
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
                    follower.followPath(intakeSpike1Path, false);

                    outtakeSystem.setArmPos(Constants.Outtake.intakeArm);

                    intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
                    driveShake = true;
                    setPathState(SampleAutoEnum.armClearing1);
                }
                break;
            case armClearing1:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    outtakeSystem.setVSlidePos(Constants.Outtake.intakeSlides);
                    outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                    intakeSystem.setHSlidePos(Constants.Intake.maxSlidePos);
                    setPathState(SampleAutoEnum.spike1Transfer);
                    driveShake = true;
                }
                break;
            case spike1Transfer:
                intakeSystem.update();
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.startTeleopDrive();
                }
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
                    missedSpike = true;
                }
                break;
            case placeSample:
                if (!transferSample) {
                    intakeSystem.setHSlidePos(Constants.Intake.autoPreExtendSlides - 100);
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
                    follower.followPath(intakeSpike2Path, false);
                    outtakeSystem.setArmPos(Constants.Outtake.intakeArm);
                    driveShake = true;
                    setPathState(SampleAutoEnum.armClearing2);
                }
                break;
            case armClearing2:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    outtakeSystem.setVSlidePos(Constants.Outtake.intakeSlides);
                    outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                    intakeSystem.setHSlidePos(Constants.Intake.maxSlidePos);
                    driveShake = true;
                    setPathState(SampleAutoEnum.spike2Transfer);
                }
                break;
            case spike2Transfer:
                intakeSystem.update();
                driveShake = true;
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.startTeleopDrive();
                }
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
                    missedSpike = true;
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
                    follower.followPath(intakeSpike3Path, false);
                    outtakeSystem.setArmPos(Constants.Outtake.intakeArm);

                    setPathState(SampleAutoEnum.armClearing3);
                }
                break;
            case armClearing3:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    outtakeSystem.setVSlidePos(Constants.Outtake.intakeSlides);
                    outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                    intakeSystem.setHSlidePos(Constants.Intake.maxSlidePos);
                    driveShake = true;
                    setPathState(SampleAutoEnum.spike3Transfer);
                }
                break;
            case spike3Transfer:
                intakeSystem.update();
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.startTeleopDrive();
                }
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
                    setPathState(SampleAutoEnum.toFailSpike);
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
                    if (missedSpike) {
                        setPathState(SampleAutoEnum.toFailSpike);
                    } else {
                        setPathState(SampleAutoEnum.goSub1);
                    }
                }
                break;

            //If failed any spike
            case toFailSpike:
                follower.followPath(toSearchSpikes);
                intakeSystem.storeOutPos();
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                    outtakeSystem.setVSlidePos(Constants.Outtake.intakeSlides);
                    setPathState(SampleAutoEnum.startLookFailSpike);
                }
                break;
            case startLookFailSpike:
                if (follower.getError(spikeSearch).getX() < 0.5 && follower.getError(spikeSearch).getY() < 0.5 && follower.getError(spikeSearch).getHeading() < Math.toRadians(4)) {
                    setPathState(SampleAutoEnum.lookFailSpike);
                }
            case lookFailSpike:
                if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                    follower.startTeleopDrive();
                    follower.setTeleOpMovementVectors(0,0,0.4);
                }
                target2D = blockVision.getBlockPosition(true);
                if (target2D.getHeading(AngleUnit.DEGREES) != -1) {
                    intakeSystem.setHSlidesInches(follower.followYourHead(target2D));
                    setPathState(SampleAutoEnum.drivingVisionFail);
                } else if (pathTimer.getElapsedTimeSeconds() > 2.25) {
                    intakeSystem.storeOutPos();
                    setPathState(SampleAutoEnum.goSub1);
                }
                break;
            case drivingVisionFail:
                if ((pathTimer.getElapsedTimeSeconds() > 0.2 && Math.abs(intakeSystem.getHSlideTargetPos() - intakeSystem.getHSlidePos()) < 250 && follower.getHeadingError() < Math.toRadians(2)) || pathTimer.getElapsedTimeSeconds() > 0.5) {
                    intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
                    setPathState(SampleAutoEnum.startIntake);
                }
                break;
            case startIntakeFail:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    driveShake = true;
                    intakeSystem.intakeUntil();
                    setPathState(SampleAutoEnum.intakingWithVisionFail);
                }
                break;
            case intakingWithVisionFail:
                intakeSystem.update();
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.startTeleopDrive();
                }
                if (intakeSystem.intakeUntil()) {
                    driveShake = false;
                    follower.followPath(placeSpike3Path);
                    intakeSystem.storeOutPos();
                    intakeSystem.setIntakePower(Constants.Intake.unjamSpeed);
                    transferSample = true;
                    setPathState(SampleAutoEnum.placeSample3);

                } else if (pathTimer.getElapsedTimeSeconds() > 2) {
                    intakeSystem.storeOutPos();
                    intakeSystem.setIntakePower(Constants.Intake.spitSpeed);
                    follower.breakFollowing();
                    driveShake = false;

                    setPathState(SampleAutoEnum.goSub1);
                }
                break;

            //Loop Starts here
            case goSub1:
                if (pathTimer.getElapsedTimeSeconds() > .3) {
                    outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                    follower.followPath(intakeSubPath);
                    outtakeSystem.setVSlidePos(Constants.Outtake.intakeSlides);
                    setPathState(SampleAutoEnum.waitSearch);
                }
                break;
            case waitSearch:
                if (follower.getErrorDistance(subIntake) < 3) {
                    outtakeSystem.setVSlidePos(Constants.Outtake.intakeSlides);
                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    setPathState(SampleAutoEnum.visionSearch1);
                }
                break;
            case visionSearch1:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    target2D = blockVision.getBlockPosition(true);
                    if (target2D.getHeading(AngleUnit.DEGREES) != -1) {
                        intakeSystem.setHSlidesInches(follower.followYourHead(target2D));
                        setPathState(SampleAutoEnum.drivingVision);
                    }
                    if (opmodeTimer.getElapsedTimeSeconds() > 28) {
                        //Break loop
                        setPathState(SampleAutoEnum.park);
                    }
                }
                break;
            case drivingVision:
                if ((pathTimer.getElapsedTimeSeconds() > 0.2 && Math.abs(intakeSystem.getHSlideTargetPos() - intakeSystem.getHSlidePos()) < 250 && follower.getHeadingError() < Math.toRadians(2)) || pathTimer.getElapsedTimeSeconds() > 0.5) {
                    //I don't exist shh
                    intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
                    setPathState(SampleAutoEnum.startIntake);
                }
                break;
            case startIntake:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    driveShake = true;
                    intakeSystem.intakeUntilColor();
                    setPathState(SampleAutoEnum.transferSample4);
                }
                break;
            case transferSample4:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.startTeleopDrive();
                }
                intakeSystem.update();
                if (opmodeTimer.getElapsedTimeSeconds() > 28) {
                    //Break loop
                    setPathState(SampleAutoEnum.park);
                }
                if (intakeSystem.intakeUntil()) {
                    driveShake = false;
                    follower.followPath(placeSubPath);
                    intakeSystem.storeOutPos();
                    intakeSystem.setIntakePower(Constants.Intake.unjamSpeed);
                    transferSample = true;
                    setPathState(SampleAutoEnum.placeSample4);
                }
                break;
            case placeSample4:
                if (!transferSample && follower.getError(afterSubPlace).getX() < 1 && follower.getError(afterSubPlace).getY() < 1 || pathTimer.getElapsedTimeSeconds() > 5) {
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
                    follower.followPath(intakeSubPath);
                    outtakeSystem.setArmPos(Constants.Outtake.intakeArm);

                    setPathState(SampleAutoEnum.waitSearch);
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
                    outtakeSystem.setVSlidePos(100);

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
                    Pose2D target2D = blockVision.getBestSample();
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
            switch (parkState) {
                case 0:
                    if (intakeSystem.intakeUntil()) {
                        intakeSystem.storePos();
                        intakeSystem.setIntakePower(Constants.Intake.unjamSpeed);
                        driveShake = false;
                        parkState = 1;
                        actionTimer.resetTimer();
                    }
                    break;
                case 1:
                    if (actionTimer.getElapsedTimeSeconds() > Constants.Intake.unjamTimeMillisAuto / 1000.0) {
                        intakeSystem.setIntakePower(Constants.Intake.intakeSpeed);
                        parkState = 2;
                        actionTimer.resetTimer();
                    }
                case 2:
                    if (intakeSystem.intakeUntil()) {
                        intakeSystem.setIntakePower(0);
                        driveShake = false;
                        parkState = 3;
                        actionTimer.resetTimer();
                    }
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
                        outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                        setTransferState(1);
                    }
                    break;
                case 1:
                    if ((intakeSystem.intakeUntil() && actionTimer.getElapsedTimeSeconds() > 0.1) || actionTimer.getElapsedTimeSeconds() > 0.5) {
                        intakeSystem.setIntakePower(0);

                        setTransferState(2);
                    }
                    break;
                case 2:
                    if (actionTimer.getElapsedTimeSeconds() > .5){
                        intakeSystem.setIntakePower(Constants.Intake.transferSpeed);

                        setTransferState(4);
                    }
                    break;
                case 4:
                    if ((outtakeSystem.readyToTransfer() && intakeSystem.readyToTransfer()) || actionTimer.getElapsedTimeSeconds() > 0.75){
                        setTransferState(5);
                    }
                    break;
                case 5:
                    if (actionTimer.getElapsedTimeSeconds() > 0.05) {
                        intakeSystem.setIntakePower(0);
                        intakeSystem.setIntakeServoPos(Constants.Intake.wristClear);
                        outtakeSystem.setClawPos(Constants.Outtake.grabClaw);

                        setTransferState(6);
                    }
                    break;
                case 6:
                    if (actionTimer.getElapsedTimeSeconds() > .3) {
                        outtakeSystem.setVSlidePos(Constants.Outtake.highBasketSlides);

                        setTransferState(7);
                    }
                    break;
                case 7:
                    if (outtakeSystem.getVSlidePos() > Constants.Outtake.safeFromClimberBar) {
                        outtakeSystem.setArmPos(Constants.Outtake.upArm);

                        intakeSystem.setIntakeServoPos(Constants.Intake.wristStore);
                        setTransferState(8);
                    }
                    break;
                case 8:
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
        if (driveShake && !doIntakeWhilePark && pathTimer.getElapsedTimeSeconds() > 1.1) {
            if (Math.round((pathTimer.getElapsedTimeSeconds() - 1.1) * 2.3) % 2 == 0 ){
                follower.setTeleOpMovementVectors(0,0, 1);
            } else {
                follower.setTeleOpMovementVectors(0,0, -1);
            }
        }

        if (driveSee && pathTimer.getElapsedTimeSeconds() > 1) {
            if (Math.round((pathTimer.getElapsedTimeSeconds() - 0.9) * .75) % 2 == 0 ){
                driveTrain.drive(0.05,0, 0.2, DriveSpeedEnum.Auto);
            } else {
                driveTrain.drive(0.05,0, -0.2, DriveSpeedEnum.Auto);
            }
        }
        if (pathTimer.getElapsedTimeSeconds() > 6.5 && !doIntakeWhilePark) {
            setPathState(SampleAutoEnum.goSub1);
        }

        // Feedback to FTC Dashboard
        Drawing.drawDebug(follower);

        // Feedback to Driver Hub
        if (true) {
            telemetry.addData("state", pathState);
//            telemetry.addData("x", follower.getPose().getX());
//            telemetry.addData("y", follower.getPose().getY());
//            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }
}
