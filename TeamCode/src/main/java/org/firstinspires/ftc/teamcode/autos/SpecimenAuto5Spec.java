package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.BezierCurve;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Path;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.PathChain;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Point;
import org.firstinspires.ftc.teamcode.pedropathing.util.Drawing;
import org.firstinspires.ftc.teamcode.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.Attempt89;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.PowerTakeOff;
import org.firstinspires.ftc.teamcode.utility.DriveSpeedEnum;
import org.firstinspires.ftc.teamcode.utility.PlacePosEnum;
import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;
import org.firstinspires.ftc.teamcode.utility.Specimen5AutoEnum;

//Match 1 transfer too long, down with servo then out with intake for 3rd          miss 2
//Match 2 Wiring                                                                   miss 4
//match 3 down with servo then out with intake for 3rd, grabed too early x2        miss 2
//match 4 Bad clip                                                                 miss 0
//match 5 down with servo then out with intake for 3rd, grabbed to early x2        miss 2
//Tournament miss one from arm not coming down early                               miss 1

public class SpecimenAuto5Spec {
    Climber climber;
    Follower follower;
    Attempt89 blockVision;
    Drivetrain driveTrain;
    PowerTakeOff powerTakeOff;
    IntakeSystem intakeSystem;
    OuttakeSystem outtakeSystem;

    FtcDashboard dashboard;
    private Timer pathTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private Specimen5AutoEnum pathState;

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(9, -65.3, Math.toRadians(180));

    /** Scoring Poses of our robot. */
    private final Pose preloadPlace = new Pose(6, -32.5, Math.toRadians(90));
    private final Pose placeSub1 = new Pose(15, -32.5, Math.toRadians(90));
    private final Pose placeSub2 = new Pose(12, -33, Math.toRadians(90));
    private final Pose placeSub3 = new Pose(9.5, -33.5, Math.toRadians(90));
    private final Pose placeSub4 = new Pose(6.5, -33.5, Math.toRadians(90));

    private final Pose spike1ControlPoint1 = new Pose(65, -62, 0);
    private final Pose spike1ControlPoint2 = new Pose(22.5, -11, 0);
    private final Pose backSpike1 = new Pose(48, -17, Math.toRadians(90));

    private final Pose pushedSpike1 = new Pose(47, -50, Math.toRadians(90));

    private final Pose spike2ControlPoint1 = new Pose(41, 0, 0);
    private final Pose spike2ControlPoint2 = new Pose(68, 0, 0);
    private final Pose pushedSpike2 = new Pose(58, -50, Math.toRadians(90));

    private final Pose spike3ControlPoint1 = new Pose(53, -5, 0);
    private final Pose backSpike3 = new Pose(63.1, -15, Math.toRadians(90));

    private final Pose pushedSpike3 = new Pose(63.1, -48, Math.toRadians(90));

    private final Pose frontWall  = new Pose(40, -57, Math.toRadians(90));
    private final Pose pickupWall = new Pose(40, -62.75, Math.toRadians(90));
    private final Pose pickupWallRightSide = new Pose(64, -62.5, Math.toRadians(90));
    private final Pose frontSubOffset = new Pose(0, -15, Math.toRadians(0));

    private final Pose parkSpit = new Pose(36, -52, Math.toRadians(-45));
    private final Pose park = new Pose(58, -58, Math.toRadians(90));
    private final Pose parkControlPoint = new Pose(12, -48, Math.toRadians(90));

    //Various Variables
    Pose2D visionResult = new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.DEGREES,-1);
    private double loopTime = 0;
    private double slamSpeed = 0.8;
    private double humanWait = 0.675;
    private boolean driveShake = false;
    private boolean recoveryDebounce = false;
    private double recoveryTime = 0;
    private boolean recovering = false;
    private boolean recoverOnce = false;
    private Specimen5AutoEnum recoveryEnum;
    private Path recoveryPath;

    private Path scorePreload;

    private Path pickupSpike1Path;
    private Path pushSpike1Path;

    private Path pickupSpike2Path;

    private Path pickupSpike3Path;
    private Path pushSpike3Path;

    private Path frontWallToWall;


    private Path toWall1;
    private Path placeSub1Path;

    private Path toFrontWall2;
    private Path placeSub2Path;

    private Path toFrontWall3;
    private Path placeSub3Path;

    private Path toFrontWall4;
    private Path placeSub4Path;

    private Path parkPath; //IS THAT A BTD REFERENCE?
    private Path parkPathSpit;


    RobotSideEnum robotSide;
    Telemetry telemetry;
    HardwareMap hardwareMap;

    public SpecimenAuto5Spec(HardwareMap hardwareMap, Telemetry telemetry, RobotSideEnum robotSide) {
        this.robotSide = robotSide;
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }
    /** This method is called once at the init of the OpMode. **/
    public void init() {
        climber = new Climber(hardwareMap);
        follower = new Follower(hardwareMap);
        blockVision = new Attempt89(hardwareMap, robotSide);
        driveTrain = new Drivetrain(hardwareMap);
        powerTakeOff = new PowerTakeOff(hardwareMap);
        intakeSystem = new IntakeSystem(hardwareMap, robotSide);

        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        pathTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        follower.setStartingPose(startPose);

        buildPaths();
        blockVision.switchPipeline(robotSide);
    }

    /** This method is called continuously after Init while waiting for "play". **/
    public void init_loop() {
        Pose2D sample = blockVision.getBlockPosition();
        telemetry.addData("Test Block X", sample.getX(DistanceUnit.INCH));
        telemetry.addData("Test Block Y", sample.getY(DistanceUnit.INCH));
        telemetry.addData("Test Block H", sample.getHeading(AngleUnit.DEGREES));
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    public void start() {
        opmodeTimer.resetTimer();

        outtakeSystem = new OuttakeSystem(hardwareMap, robotSide, true);

        outtakeSystem.setArmPos(Constants.Outtake.upArm);

        blockVision.switchPipeline(robotSide);

        setPathState(Specimen5AutoEnum.driveGoScorePreload);
    }

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        scorePreload     = follower.linearPathBuilder(startPose, preloadPlace);
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), preloadPlace.getHeading(), 0.25);

        double zeroMultFast = 1;
        double zeroMultSlow = 0.01;

        pickupSpike1Path = new Path(new BezierCurve(new Point(preloadPlace), new Point(spike1ControlPoint1), new Point(spike1ControlPoint2), new Point(backSpike1)));
        pickupSpike1Path.setConstantHeadingInterpolation(Math.toRadians(90));
//        pickupSpike1Path.setZeroPowerAccelerationMultiplier(zeroMultFast);

        pushSpike1Path   = follower.linearPathBuilder(backSpike1, pushedSpike1);
//        pushSpike1Path.setZeroPowerAccelerationMultiplier(zeroMultFast);

        pickupSpike2Path = new Path(new BezierCurve(new Point(pushedSpike1), new Point(spike2ControlPoint1), new Point(spike2ControlPoint2), new Point(pushedSpike2)));
        pickupSpike2Path.setConstantHeadingInterpolation(Math.toRadians(90));
//        pickupSpike2Path.setZeroPowerAccelerationMultiplier(zeroMultFast);

        pickupSpike3Path = new Path(new BezierCurve(new Point(pushedSpike2), new Point(spike3ControlPoint1), new Point(backSpike3)));
        pickupSpike3Path.setConstantHeadingInterpolation(Math.toRadians(90));
//        pickupSpike2Path.setZeroPowerAccelerationMultiplier(zeroMultFast);

        pushSpike3Path   = follower.linearPathBuilder(backSpike3, pushedSpike3);
//        pushSpike3Path.setZeroPowerAccelerationMultiplier(zeroMultSlow);


        frontWallToWall  = follower.linearPathBuilder(frontWall, pickupWall);
//        frontWallToWall.setZeroPowerAccelerationMultiplier(3.5);

        toWall1 = follower.linearPathBuilder(pushedSpike3, pickupWallRightSide);
        toWall1.setConstantHeadingInterpolation(pushedSpike3.getHeading());
//        toWall1.setZeroPowerAccelerationMultiplier(zeroMultSlow);

        //frontWallToWall
        placeSub1Path = follower.linearPathBuilder(pickupWallRightSide, placeSub1.addReturn(frontSubOffset));
        placeSub1Path.setConstantHeadingInterpolation(placeSub1.getHeading());
//        placeSub1Path.setZeroPowerAccelerationMultiplier(zeroMultFast);


        toFrontWall2 = follower.linearPathBuilder(placeSub1.addReturn(new Pose(0,-3,0)), frontWall);
        toFrontWall2.setConstantHeadingInterpolation(placeSub1.getHeading());
//        toFrontWall2.setZeroPowerAccelerationMultiplier(4);
        //frontWallToWall
        placeSub2Path = follower.linearPathBuilder(pickupWall, placeSub2.addReturn(frontSubOffset));
        placeSub2Path.setConstantHeadingInterpolation(placeSub2.getHeading());
//        placeSub2Path.setZeroPowerAccelerationMultiplier(zeroMultFast);


        toFrontWall3 = follower.linearPathBuilder(placeSub2.addReturn(new Pose(0,-3,0)), frontWall);
        toFrontWall3.setConstantHeadingInterpolation(placeSub2.getHeading());
//        toFrontWall3.setZeroPowerAccelerationMultiplier(zeroMultSlow);

        //frontWallToWall
        placeSub3Path = follower.linearPathBuilder(pickupWall, placeSub3.addReturn(frontSubOffset));
        placeSub3Path.setConstantHeadingInterpolation(placeSub3.getHeading());
//        placeSub3Path.setZeroPowerAccelerationMultiplier(zeroMultFast);


        toFrontWall4 = follower.linearPathBuilder(placeSub3.addReturn(new Pose(0,-3,0)), frontWall);
        toFrontWall4.setConstantHeadingInterpolation(placeSub3.getHeading());
//        toFrontWall4.setZeroPowerAccelerationMultiplier(zeroMultSlow);
        //frontWallToWall
        placeSub4Path = follower.linearPathBuilder(pickupWall, placeSub4.addReturn(frontSubOffset));
        placeSub4Path.setConstantHeadingInterpolation(placeSub4.getHeading());
//        placeSub4Path.setZeroPowerAccelerationMultiplier(zeroMultFast);

        parkPathSpit = new Path(new BezierCurve(new Point(placeSub4), new Point(parkControlPoint), new Point(parkSpit)));
        parkPathSpit.setLinearHeadingInterpolation(placeSub4.getHeading(), parkSpit.getHeading());
        parkPath = new Path(new BezierCurve(new Point(placeSub4), new Point(parkControlPoint), new Point(park)));
        parkPath.setConstantHeadingInterpolation(placeSub4.getHeading());

    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState(Specimen5AutoEnum.) method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            //go to score preload
            case driveGoScorePreload:
                follower.followPath(scorePreload);

                outtakeSystem.setVSlidePos(Constants.Outtake.highSpecimenSlides);
                outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                outtakeSystem.setArmPos(Constants.Outtake.specimenArm);

                setPathState(Specimen5AutoEnum.placePreload);
                break;
            //go to pickup spike1
            case placePreload:
                //Checks if you are less than 1 inch away from target pos
                if (follower.getError(preloadPlace).getX() < 1.5 && follower.getError(preloadPlace).getY() < 1.5 && pathTimer.getElapsedTimeSeconds() > .17) {
                    follower.followPath(pickupSpike1Path);
                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    setPathState(Specimen5AutoEnum.droppedClaw0);
                }
                break;
            case droppedClaw0:
                if (pathTimer.getElapsedTimeSeconds() > 0.17) {
                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    outtakeSystem.setArmPos(Constants.Outtake.upArm);
                    outtakeSystem.setVSlidePos(0);
                    setPathState(Specimen5AutoEnum.pushSpike1);
                }
                break;
            case pushSpike1:
                if (follower.getError(backSpike1).getX() < 4 && follower.getError(backSpike1).getY() < 4) {
                    follower.followPath(pushSpike1Path);
                    setPathState(Specimen5AutoEnum.pushSpike2);
                }
                break;
            case pushSpike2:
                if (follower.getError(pushedSpike1).getX() < 2 && follower.getError(pushedSpike1).getY() < 2) {
                    follower.followPath(pickupSpike2Path);
                    setPathState(Specimen5AutoEnum.goBackSpike3);
                }
                break;
            //go to pickup spike3
            case goBackSpike3:
                if (follower.getError(pushedSpike2).getX() < 4 && follower.getError(pushedSpike2).getY() < 4) {
                    follower.followPath(pickupSpike3Path);
                    setPathState(Specimen5AutoEnum.pushSpike3);
                }
                break;
            //go to front pickup wall
            case pushSpike3:
                if (follower.getError(backSpike3).getX() < 4 && follower.getError(backSpike3).getY() < 4) {
                    follower.followPath(pushSpike3Path);
                    outtakeSystem.placePos(PlacePosEnum.wallAuto);
                    setPathState(Specimen5AutoEnum.waitFrontWall1);
                }
                break;
            case waitFrontWall1:
                if (follower.getError(pushedSpike3).getY() < 2) {
                    setPathState(Specimen5AutoEnum.pickupWall1);
                }
                break;
            case pickupWall1:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.setMaxPower(0.4);
                    follower.followPath(toWall1);
                    setPathState(Specimen5AutoEnum.grabWall1);
                }
                break;
            //grab off wall and go to sub
            case grabWall1:
                if (follower.getError(pickupWallRightSide).getY() < 2.5 || pathTimer.getElapsedTimeSeconds() > 1) {
                    outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                    setPathState(Specimen5AutoEnum.goPlaceSub1);
                }
                break;
            //outtake to specimen place pos
            case goPlaceSub1:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.setMaxPower(1);
                    follower.followPath(placeSub1Path);
                    outtakeSystem.setVSlidePos(Constants.Outtake.safeFromWallSlides);
                    setPathState(Specimen5AutoEnum.specPreset1);
                }
                break;
            case specPreset1:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    outtakeSystem.placePos(PlacePosEnum.highSpecimen);
                    setPathState(Specimen5AutoEnum.drivePlace1);
                }
                break;
            case drivePlace1:
                if (follower.getErrorDistance(placeSub1.addReturn(frontSubOffset)) < 1.5) {
                    follower.breakFollowing();
                    follower.driveSlam(true);

                    driveTrain.drive(slamSpeed,0,0, DriveSpeedEnum.Auto);
                    setPathState(Specimen5AutoEnum.dropClaw1);
                }
                break;
            //go Front Wall 2
            case dropClaw1:
                driveTrain.drive(slamSpeed,0,0, DriveSpeedEnum.Auto);
                if (follower.getError(placeSub1).getY() < 2 || pathTimer.getElapsedTimeSeconds() > 0.75) {
                    follower.driveSlam(false);

                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    follower.followPath(toFrontWall2);
                    setPathState(Specimen5AutoEnum.wallPreset2);
                }
                break;
            case wallPreset2:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    outtakeSystem.placePos(PlacePosEnum.wallAuto);
                    setPathState(Specimen5AutoEnum.frontWallToWall2);
                }
                break;
            //part 5 placing specimen 3 ***********************************************************~
            //front Wall To Wall
            case frontWallToWall2:
                if (follower.getErrorDistance(frontWall) < 1) {
                    setPathState(Specimen5AutoEnum.waitFrontWall2);
                }
                break;
            case waitFrontWall2:
                if (pathTimer.getElapsedTimeSeconds() > humanWait) {
                    follower.setMaxPower(0.4);
                    follower.followPath(frontWallToWall);
                    setPathState(Specimen5AutoEnum.grabWall2);
                }
                break;
            //grab off wall and go to sub
            case grabWall2:
                if (follower.getErrorDistance(pickupWall) < 1.5 || pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.setMaxPower(1);
                    outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                    setPathState(Specimen5AutoEnum.goPlaceSub2);
                }
                break;
            //outtake to specimen place pos
            case goPlaceSub2:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(placeSub2Path);
                    outtakeSystem.setVSlidePos(Constants.Outtake.safeFromWallSlides);
                    setPathState(Specimen5AutoEnum.specPreset2);
                }
                break;
            case specPreset2:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    outtakeSystem.placePos(PlacePosEnum.highSpecimen);
                    setPathState(Specimen5AutoEnum.drivePlace2);
                }
                break;
            case drivePlace2:
                if (follower.getErrorDistance(placeSub2.addReturn(frontSubOffset)) < 1.5) {
                    follower.breakFollowing();
                    follower.driveSlam(true);

                    driveTrain.drive(slamSpeed,0,0, DriveSpeedEnum.Auto);
                    setPathState(Specimen5AutoEnum.dropClaw2);
                }
                break;
            //go Front Wall 2
            case dropClaw2:
                driveTrain.drive(slamSpeed,0,0, DriveSpeedEnum.Auto);
                if (follower.getError(placeSub2).getY() < 2 || pathTimer.getElapsedTimeSeconds() > 0.75) {
                    follower.driveSlam(false);

                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    follower.followPath(toFrontWall3);
                    setPathState(Specimen5AutoEnum.wallPreset3);
                }
                break;
            case wallPreset3:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    outtakeSystem.placePos(PlacePosEnum.wallAuto);
                    setPathState(Specimen5AutoEnum.frontWallToWall3);
                }
                break;
            //part 6 placing specimen 4 ***********************************************************~
            //front Wall To Wall
            case frontWallToWall3:
                if (follower.getErrorDistance(frontWall) < 1) {
                    setPathState(Specimen5AutoEnum.waitFrontWall3);
                }
                break;
            case waitFrontWall3:
                if (pathTimer.getElapsedTimeSeconds() > humanWait) {
                    follower.setMaxPower(0.4);
                    follower.followPath(frontWallToWall);
                    setPathState(Specimen5AutoEnum.grabWall3);
                }
                break;
            //grab off wall and go to sub
            case grabWall3:
                if (follower.getErrorDistance(pickupWall) < 1.5 || pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.setMaxPower(1);
                    outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                    setPathState(Specimen5AutoEnum.goPlaceSub3);
                }
                break;
            //outtake to specimen place pos
            case goPlaceSub3:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(placeSub3Path);
                    outtakeSystem.setVSlidePos(Constants.Outtake.safeFromWallSlides);
                    setPathState(Specimen5AutoEnum.specPreset3);
                }
                break;
            case specPreset3:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    outtakeSystem.placePos(PlacePosEnum.highSpecimen);
                    setPathState(Specimen5AutoEnum.drivePlace3);
                }
                break;
            case drivePlace3:
                if (follower.getErrorDistance(placeSub3.addReturn(frontSubOffset)) < 1.5) {
                    follower.breakFollowing();
                    follower.driveSlam(true);

                    driveTrain.drive(slamSpeed,0,0, DriveSpeedEnum.Auto);
                    setPathState(Specimen5AutoEnum.dropClaw3);
                }
                break;
            //go Front Wall 2
            case dropClaw3:
                driveTrain.drive(slamSpeed,0,0, DriveSpeedEnum.Auto);
                if (follower.getError(placeSub3).getY() < 2 || pathTimer.getElapsedTimeSeconds() > 0.75) {
                    follower.driveSlam(false);

                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);

                    follower.followPath(toFrontWall4);
                    setPathState(Specimen5AutoEnum.wallPreset4);

                }
                break;
            case wallPreset4:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    outtakeSystem.placePos(PlacePosEnum.wallAuto);
                    setPathState(Specimen5AutoEnum.frontWallToWall4);
                }
                break;
            //part 7 placing specimen 5 ***********************************************************~
            //front Wall To Wall
            case frontWallToWall4:
                if (follower.getErrorDistance(frontWall) < 1) {
                    setPathState(Specimen5AutoEnum.waitFrontWall4);
                }
                break;
            case waitFrontWall4:
                if (pathTimer.getElapsedTimeSeconds() > humanWait) {
                    follower.setMaxPower(0.4);
                    follower.followPath(frontWallToWall);
                    setPathState(Specimen5AutoEnum.grabWall4);
                }
                break;
            //grab off wall and go to sub
            case grabWall4:
                if (follower.getErrorDistance(pickupWall) < 1.5 || pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.setMaxPower(1);
                    outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                    setPathState(Specimen5AutoEnum.goPlaceSub4);
                }
                break;
            //outtake to specimen place pos
            case goPlaceSub4:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(placeSub4Path);
                    outtakeSystem.setVSlidePos(Constants.Outtake.safeFromWallSlides);
                    setPathState(Specimen5AutoEnum.specPreset4);
                }
                break;
            case specPreset4:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    outtakeSystem.placePos(PlacePosEnum.highSpecimen);
                    setPathState(Specimen5AutoEnum.drivePlace4);
                }
                break;
            case drivePlace4:
                if (follower.getErrorDistance(placeSub4.addReturn(frontSubOffset)) < 1.5) {
                    follower.breakFollowing();
                    follower.driveSlam(true);

                    driveTrain.drive(slamSpeed,0,0, DriveSpeedEnum.Auto);
                    setPathState(Specimen5AutoEnum.dropClaw4);
                }
                break;
            //go Front Wall 2
            case dropClaw4:
                driveTrain.drive(slamSpeed,0,0, DriveSpeedEnum.Auto);
                if (follower.getError(placeSub4).getY() < 3 || pathTimer.getElapsedTimeSeconds() > 0.75) {
                    follower.driveSlam(false);
                    driveTrain.drive(0,0,0, DriveSpeedEnum.Auto);
                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);

                    setPathState(Specimen5AutoEnum.visionLook);
                }
                break;
            case visionLook:
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
//                    visionResult = blockVision.getBlockPosition();
//                    if (visionResult.getHeading(AngleUnit.DEGREES) != -1) {
//                        telemetry.addData("targetx", visionResult.getX(DistanceUnit.INCH));
//                        telemetry.addData("targety", visionResult.getY(DistanceUnit.INCH));
//                        telemetry.addData("target", visionResult.getHeading(AngleUnit.DEGREES));
//                        telemetry.update();
//
//                        follower.followYourHeart(visionResult.getX(DistanceUnit.INCH));
//                        intakeSystem.setHSlidesInches(visionResult.getY(DistanceUnit.INCH));
//                        setPathState(Specimen5AutoEnum.visionDropIntake);
//                    }
                } else if (opmodeTimer.getElapsedTimeSeconds() > 27) {
                    outtakeSystem.setArmPos(Constants.Outtake.upArm);
                    outtakeSystem.setVSlidePos(0);
                    setPathState(Specimen5AutoEnum.goParkNoSpit);
                }
                break;
            case visionDropIntake:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
                    setPathState(Specimen5AutoEnum.visionIntake);
                }
                break;
            case visionIntake:
                if (intakeSystem.intakeUntilColor()) {
                    intakeSystem.storePos();
                    outtakeSystem.setArmPos(Constants.Outtake.upArm);
                    outtakeSystem.setVSlidePos(0);
                    intakeSystem.setIntakePower(Constants.Intake.unjamSpeed);
                    setPathState(Specimen5AutoEnum.intakeUnjam);

                } else if (opmodeTimer.getElapsedTimeSeconds() > 27) {
                    intakeSystem.storePos();
                    intakeSystem.setIntakePower(Constants.Intake.spitSpeed);
                    setPathState(Specimen5AutoEnum.goParkNoSpit);
                }
                break;
            case intakeUnjam:
                if (pathTimer.getElapsedTime() > Constants.Intake.unjamTimeMillisAuto) {
                    intakeSystem.setIntakePower(0);
                    setPathState(Specimen5AutoEnum.goParkSpit);
                }
                break;
                //Park path
            case goParkSpit:
                follower.followPath(parkPathSpit);
                setPathState(Specimen5AutoEnum.teleopPresetSpit);
                break;
            case teleopPresetSpit:
                if (follower.getError(parkSpit).getHeading() < Math.toRadians(100)) {
                    outtakeSystem.setArmPos(Constants.Outtake.upArm);
                    outtakeSystem.setVSlidePos(0);
                    intakeSystem.setHSlidePos(Constants.Intake.intakeSlidePos);
                    setPathState(Specimen5AutoEnum.spit);
                }
                break;
            case spit:
                if (follower.getError(parkSpit).getHeading() < Math.toRadians(7) && Math.abs(intakeSystem.getHSlidePos() - intakeSystem.getHSlideTargetPos()) < 300) {
                    intakeSystem.setIntakeServoPos(Constants.Intake.wristClear);
                    intakeSystem.setIntakePower(Constants.Intake.spitSpeed);
                    setPathState(Specimen5AutoEnum.stopSpit);
                }
                break;
            case stopSpit:
                if (pathTimer.getElapsedTimeSeconds() > 0.5 && opmodeTimer.getElapsedTimeSeconds() < 29.25) {
                    intakeSystem.storePos();
                    follower.followPath(parkPath);
                    setPathState(Specimen5AutoEnum.end);
                }
                break;

            case goParkNoSpit:
                follower.followPath(parkPath);
                intakeSystem.storePos();
                setPathState(Specimen5AutoEnum.teleopPreset);
                break;
            case teleopPreset:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    outtakeSystem.setArmPos(Constants.Outtake.upArm);
                    outtakeSystem.setVSlidePos(0);
                    setPathState(Specimen5AutoEnum.end);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(Specimen5AutoEnum pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    public void loop() {
        // These loop the movements of the robot
        follower.update();

        autonomousPathUpdate();
        if (driveShake && pathTimer.getElapsedTimeSeconds() > 1.2) {
            if (Math.round((pathTimer.getElapsedTimeSeconds() - 1.2) * 2.5) % 2 == 0 ){
                driveTrain.drive(0,0, -0.5, DriveSpeedEnum.Auto);
            } else {
                driveTrain.drive(0,0, 0.5, DriveSpeedEnum.Auto);
            }
        }
        //Dumb path recovery
        if (pathTimer.getElapsedTimeSeconds() > 1 && driveTrain.isStalled(3) && !recoveryDebounce) {
            recoveryTime = opmodeTimer.getElapsedTimeSeconds();
            recoveryDebounce = true;
            recoverOnce = false;
        } else if (!driveTrain.isStalled(3)) {
            recoveryDebounce = false;
        }

        if (recoveryDebounce && driveTrain.isStalled(3) && opmodeTimer.getElapsedTimeSeconds() > recoveryTime + 2) {
            recoveryTime = opmodeTimer.getElapsedTimeSeconds();
            recoveryPath = follower.getCurrentPath();
            recoveryEnum = pathState;
            setPathState(Specimen5AutoEnum.end);
            recovering = true;
        }

        if (recovering) {
            if (!recoverOnce) {
                follower.followPath(parkPath);
                recoverOnce = true;
            }
            if (follower.getErrorDistance(park) < 3 && pathTimer.getElapsedTimeSeconds() > 1) {
                setPathState(recoveryEnum);
                follower.followPath(recoveryPath);
                recovering = false;
            }
        }

        // Feedback to FTC Dashboard
        Drawing.drawDebug(follower);

        // Feedback to Driver Hub
        if (false) {
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addData("loopTime", loopTime - opmodeTimer.getElapsedTime());
            telemetry.addData("tripped", driveTrain.isStalled(3));
            telemetry.addData("recoveryDebounce", recoveryDebounce);
            telemetry.addData("recovering", recovering);
            telemetry.addData("recoverOnce", recoverOnce);
            loopTime = opmodeTimer.getElapsedTime();
            telemetry.update();
        }

        telemetry.addData("loopTime", loopTime - opmodeTimer.getElapsedTime());
        loopTime = opmodeTimer.getElapsedTime();
        telemetry.update();
    }
    /** We do not use this because everything should automatically disable **/
    public void stop() {
    }
}