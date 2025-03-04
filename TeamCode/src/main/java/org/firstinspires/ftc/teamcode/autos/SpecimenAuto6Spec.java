package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Drawing;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.Attempt89;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.PowerTakeOff;
import org.firstinspires.ftc.teamcode.utility.DriveSpeedEnum;
import org.firstinspires.ftc.teamcode.utility.PlacePosEnum;
import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;
import org.firstinspires.ftc.teamcode.utility.SampleAutoEnum;
import org.firstinspires.ftc.teamcode.utility.Specimen6AutoEnum;

//Match 1 transfer too long, down with servo then out with intake for 3rd          miss 2
//Match 2 Wiring                                                                   miss 4
//match 3 down with servo then out with intake for 3rd, grabed too early x2        miss 2
//match 4 Bad clip                                                                 miss 0
//match 5 down with servo then out with intake for 3rd, grabbed to early x2        miss 2
//Tournament miss one from arm not coming down early                               miss 1


public class SpecimenAuto6Spec {
    Climber climber;
    Follower follower;
    Attempt89 blockVision;
    Drivetrain driveTrain;
    PowerTakeOff powerTakeOff;
    IntakeSystem intakeSystem;
    OuttakeSystem outtakeSystem;

    private Timer pathTimer, storeTimer, transferTimer, wallTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private Specimen6AutoEnum pathState;

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(7.25, -65.75, Math.toRadians(180));

    /** Scoring Poses of our robot. */
    private final Pose preloadPlace = new Pose(-4, -31.5, Math.toRadians(90));
    private final Pose placeSub1 = new Pose(6, -31.5, Math.toRadians(90));
    private final Pose placeSub2 = new Pose(7.25, -31.5, Math.toRadians(90));
    private final Pose placeSub3 = new Pose(8.5, -31.5, Math.toRadians(90));
    private final Pose placeSub4 = new Pose(9.75, -31.5, Math.toRadians(90));
    private final Pose placeSub5 = new Pose(10, -31.5, Math.toRadians(90));

    private final Pose spike1ControlPoint1 = new Pose(55, -62, 0);
    private final Pose spike1ControlPoint2 = new Pose(22.5, -11, 0);
    private final Pose backSpike1 = new Pose(48, -17, Math.toRadians(90));

    private final Pose pushedSpike1 = new Pose(48.5, -50, Math.toRadians(90));

    private final Pose spike2ControlPoint1 = new Pose(41, -6, 0);
    private final Pose spike2ControlPoint2 = new Pose(68, -6, 0);
    private final Pose pushedSpike2 = new Pose(57, -53, Math.toRadians(90));

    private final Pose spike3ControlPoint1 = new Pose(53, -5, 0);
    private final Pose backSpike3 = new Pose(63, -15, Math.toRadians(90));

    private final Pose pushedSpike3 = new Pose(63, -50, Math.toRadians(90));

    private final Pose frontWall  = new Pose(45, -53, Math.toRadians(90));
    private final Pose pickupWall = new Pose(45, -62, Math.toRadians(90));
    private final Pose pickupWallRightSide = new Pose(64, -61.5, Math.toRadians(90));
    private final Pose frontSubOffset = new Pose(0, -5, Math.toRadians(0));

    private final Pose park = new Pose(58, -58, Math.toRadians(90));

    //Various Variables
    private  Pose2D target = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0);

    Pose2D visionResult = new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.DEGREES,-1);
    private double loopTime = 0;
    private double slamSpeed = 0.8;
    private boolean driveShake = false;
    private boolean drivePlace = false;

    private boolean doStore = false;
    private boolean doTransfer = false;
    private boolean doGoWall = false;
    private boolean hasOne = false;
    private int storeState = 0;
    private int transferState = 0;
    private int wallState = 0;

    private boolean recoveryDebounce = false;
    private double recoveryTime = 0;
    private boolean recovering = false;
    private boolean recoverOnce = false;
    private Specimen6AutoEnum recoveryEnum;
    private Path recoveryPath;

    // These are our Paths and PathChains that we will define in buildPaths()
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

    private Path toFrontWall5;
    private Path placeSub5Path;

    private Path parkPath; //IS THAT A BTD REFERENCE?

    RobotSideEnum robotSide;
    Telemetry telemetry;
    HardwareMap hardwareMap;

    public SpecimenAuto6Spec(HardwareMap hardwareMap, Telemetry telemetry, RobotSideEnum robotSide) {
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
        outtakeSystem = new OuttakeSystem(hardwareMap);

//        outtakeSystem.setArmPos(Constants.Outtake.initAutoArm);

        pathTimer = new Timer();
        opmodeTimer = new Timer();

        storeTimer = new Timer();
        transferTimer = new Timer();
        wallTimer = new Timer();

        opmodeTimer.resetTimer();

        follower.setStartingPose(startPose);

        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    public void init_loop() {
        Pose2D sample = blockVision.getBestSpecimen();
        telemetry.addData("Test Block X", sample.getX(DistanceUnit.INCH));
        telemetry.addData("Test Block Y", sample.getY(DistanceUnit.INCH));
        telemetry.addData("Test Block H", sample.getHeading(AngleUnit.DEGREES));
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(Specimen6AutoEnum.armClearing);
    }

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        scorePreload     = follower.linearPathBuilder(startPose, preloadPlace);

        double zeroMultFast = 10;
        double zeroMultSlow = 0.4;

        pickupSpike1Path = new Path(new BezierCurve(new Point(preloadPlace), new Point(spike1ControlPoint1), new Point(spike1ControlPoint2), new Point(backSpike1)));
        pickupSpike1Path.setConstantHeadingInterpolation(Math.toRadians(90));
        pickupSpike1Path.setZeroPowerAccelerationMultiplier(zeroMultFast);

        pushSpike1Path   = follower.linearPathBuilder(backSpike1, pushedSpike1);
        pushSpike1Path.setZeroPowerAccelerationMultiplier(zeroMultFast);

        pickupSpike2Path = new Path(new BezierCurve(new Point(pushedSpike1), new Point(spike2ControlPoint1), new Point(spike2ControlPoint2), new Point(pushedSpike2)));
        pickupSpike2Path.setConstantHeadingInterpolation(Math.toRadians(90));
        pickupSpike2Path.setZeroPowerAccelerationMultiplier(zeroMultFast);

        pickupSpike3Path = new Path(new BezierCurve(new Point(pushedSpike2), new Point(spike3ControlPoint1), new Point(backSpike3)));
        pickupSpike3Path.setConstantHeadingInterpolation(Math.toRadians(90));
        pickupSpike2Path.setZeroPowerAccelerationMultiplier(zeroMultFast);

        pushSpike3Path   = follower.linearPathBuilder(backSpike3, pushedSpike3);
        pickupSpike2Path.setZeroPowerAccelerationMultiplier(zeroMultFast);


        frontWallToWall  = follower.linearPathBuilder(frontWall, pickupWall);

        toWall1 = follower.linearPathBuilder(pushedSpike3, pickupWallRightSide);
        //frontWallToWall
        placeSub1Path = follower.linearPathBuilder(pickupWallRightSide, placeSub1.addReturn(frontSubOffset));
        placeSub1Path.setConstantHeadingInterpolation(placeSub1.getHeading());
        placeSub1Path.setZeroPowerAccelerationMultiplier(zeroMultFast);


        toFrontWall2 = follower.linearPathBuilder(placeSub1, frontWall);
        toFrontWall2.setConstantHeadingInterpolation(placeSub1.getHeading());
        toFrontWall2.setZeroPowerAccelerationMultiplier(zeroMultSlow);
        //frontWallToWall
        placeSub2Path = follower.linearPathBuilder(pickupWall, placeSub2.addReturn(frontSubOffset));
        placeSub2Path.setConstantHeadingInterpolation(placeSub2.getHeading());
        placeSub2Path.setZeroPowerAccelerationMultiplier(zeroMultFast);


        toFrontWall3 = follower.linearPathBuilder(placeSub2, frontWall);
        toFrontWall3.setConstantHeadingInterpolation(placeSub2.getHeading());
        toFrontWall3.setZeroPowerAccelerationMultiplier(zeroMultSlow);
        //frontWallToWall
        placeSub3Path = follower.linearPathBuilder(pickupWall, placeSub3.addReturn(frontSubOffset));
        placeSub3Path.setConstantHeadingInterpolation(placeSub3.getHeading());
        placeSub3Path.setZeroPowerAccelerationMultiplier(zeroMultFast);


        toFrontWall4 = follower.linearPathBuilder(placeSub3, frontWall);
        toFrontWall4.setConstantHeadingInterpolation(placeSub3.getHeading());
        toFrontWall4.setZeroPowerAccelerationMultiplier(zeroMultSlow);
        //frontWallToWall
        placeSub4Path = follower.linearPathBuilder(pickupWall, placeSub4.addReturn(frontSubOffset));
        placeSub4Path.setConstantHeadingInterpolation(placeSub4.getHeading());
        placeSub4Path.setZeroPowerAccelerationMultiplier(zeroMultFast);

        toFrontWall5 = follower.linearPathBuilder(placeSub4, frontWall);
        toFrontWall5.setConstantHeadingInterpolation(placeSub4.getHeading());
        toFrontWall5.setZeroPowerAccelerationMultiplier(zeroMultSlow);
        //frontWallToWall
        placeSub5Path = follower.linearPathBuilder(pickupWall, placeSub5.addReturn(frontSubOffset));
        placeSub5Path.setConstantHeadingInterpolation(placeSub5.getHeading());
        placeSub5Path.setZeroPowerAccelerationMultiplier(zeroMultFast);

        parkPath = follower.linearPathBuilder(placeSub5, park);

    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState(Specimen6AutoEnum.) method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() { // todo: deal with all the doTransfer/hasOne/doStore code
        switch (pathState) {
            //go to score preload
            case armClearing:
                blockVision.switchPipeline(robotSide);
                outtakeSystem.setVSlidePos(Constants.Outtake.highSpecimenSlides);
                outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                setPathState(Specimen6AutoEnum.armClearing2);
                break;
            case armClearing2:
                if (pathTimer.getElapsedTimeSeconds() > .2) {
                    outtakeSystem.setArmPos(Constants.Outtake.specimenArm);
                    setPathState(Specimen6AutoEnum.driveGoScorePreload);
                }
                break;
            case driveGoScorePreload:
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    follower.followPath(scorePreload);
                    setPathState(Specimen6AutoEnum.armGoScorePreload);
                }
                break;
            case armGoScorePreload:
                if (Math.abs(outtakeSystem.getVSlidePos() - outtakeSystem.getVSlideTargetPos()) < 100) {
                    outtakeSystem.setArmPos(Constants.Outtake.specimenArm);
                    setPathState(Specimen6AutoEnum.slideGoScorePreload);
                }
                break;
            case slideGoScorePreload:
                if (pathTimer.getElapsedTimeSeconds() > 0.05) {
                    outtakeSystem.setVSlidePos(Constants.Outtake.highSpecimenSlides - 100);
                    setPathState(Specimen6AutoEnum.placePreload);
                }
                break;
            //go to pickup spike1
            case placePreload:
                //Checks if you are less than 1 inch away from target pos
                if (follower.getError(preloadPlace).getX() < 1 && follower.getError(preloadPlace).getY() < 1 && pathTimer.getElapsedTimeSeconds() > .17) {
                    follower.followPath(pickupSpike1Path);
                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    setPathState(Specimen6AutoEnum.visionLook);
                }
                break;
            //go to pickup wall preset
            case visionLook:
                if (pathTimer.getElapsedTimeSeconds() > .15) {
                    //wait for robot to fall & stop bouncing
                    visionResult = blockVision.getBestSpecimen();
                    if (visionResult.getHeading(AngleUnit.DEGREES) != -1) {
                        doStore = true;
                        intakeSystem.setHSlidesInches(follower.followYourHead(visionResult));
                        setPathState(Specimen6AutoEnum.drivingVision);

                    } else if (pathTimer.getElapsedTimeSeconds() > 1) {
                        outtakeSystem.setArmPos(Constants.Outtake.upArm);
                        outtakeSystem.setVSlidePos(0);
                        setPathState(Specimen6AutoEnum.pushSpike1);
                    }
                }
                break;
            case droppedClaw0:
                if (pathTimer.getElapsedTimeSeconds() > 0.17) {
                    outtakeSystem.setArmPos(Constants.Outtake.upArm);
                    outtakeSystem.setVSlidePos(0);
                    setPathState(Specimen6AutoEnum.drivingVision);
                }
                break;
            //go to pickup spike2
            case drivingVision:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
                    setPathState(Specimen6AutoEnum.intakingVision);
                }
                break;
            case intakingVision:
                if (intakeSystem.intakeUntilColor()) {
                    doTransfer = true; //todo: swap to drive + spit
                    intakeSystem.storePos();
                    hasOne = true;

                    follower.followPath(toFrontWall3);
                    setPathState(Specimen6AutoEnum.pushSpike1);

                } else if (pathTimer.getElapsedTimeSeconds() > 2) {
                    doGoWall = true;
                    intakeSystem.storePos();
                    hasOne = false;

                    setPathState(Specimen6AutoEnum.pushSpike1);
                }
                break;
            case pushSpike1:
                if (follower.getError(backSpike1).getX() < 4 && follower.getError(backSpike1).getY() < 4) {
                    follower.followPath(pushSpike1Path);
                    setPathState(Specimen6AutoEnum.pushSpike2);
                }
                break;
            case pushSpike2:
                if (follower.getError(pushedSpike1).getX() < 2 && follower.getError(pushedSpike1).getY() < 2) {
                    follower.followPath(pickupSpike2Path);
                    setPathState(Specimen6AutoEnum.goBackSpike3);
                }
                break;
            //go to pickup spike3
            case goBackSpike3:
                if (follower.getError(pushedSpike2).getX() < 2 && follower.getError(pushedSpike2).getY() < 2) {
                    follower.followPath(pickupSpike3Path);
                    setPathState(Specimen6AutoEnum.pushSpike3);
                }
                break;
            //go to front pickup wall
            case pushSpike3:
                if (follower.getError(backSpike3).getX() < 4 && follower.getError(backSpike3).getY() < 4) {
                    follower.followPath(pushSpike3Path);
                    outtakeSystem.placePos(PlacePosEnum.wallAuto);
                    setPathState(Specimen6AutoEnum.waitFrontWall1);
                }
                break;
            case waitFrontWall1:
                if (follower.getError(pushedSpike3).getY() < 1.5) {
                    setPathState(Specimen6AutoEnum.pickupWall1);
                }
                break;
            case pickupWall1:
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    follower.followPath(toWall1);
                    setPathState(Specimen6AutoEnum.grabWall1);
                }
                break;
            //grab off wall and go to sub
            case grabWall1:
                if (follower.getError(pickupWallRightSide).getY() < 1.5) {
                    outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                    setPathState(Specimen6AutoEnum.goPlaceSub1);
                }
                break;
            //outtake to specimen place pos
            case goPlaceSub1:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(placeSub1Path);
                    outtakeSystem.setVSlidePos(Constants.Outtake.safeFromWallSlides);
                    setPathState(Specimen6AutoEnum.specPreset1);
                }
                break;
            case specPreset1:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    outtakeSystem.placePos(PlacePosEnum.highSpecimen);
                    setPathState(Specimen6AutoEnum.drivePlace1);
                }
                break;
            case drivePlace1:
                if (follower.getErrorDistance(placeSub1.addReturn(frontSubOffset)) < 1.5) {
                    follower.breakFollowing();
                    follower.driveSlam(true);

                    driveTrain.drive(slamSpeed,0,0, DriveSpeedEnum.Auto);
                    setPathState(Specimen6AutoEnum.dropClaw1);
                }
                break;
            //go Front Wall 2
            case dropClaw1:
                driveTrain.drive(slamSpeed,0,0, DriveSpeedEnum.Auto);
                if (follower.getError(placeSub1).getY() < 1) {
                    follower.driveSlam(false);
                    setPathState(Specimen6AutoEnum.wallPreset2);
                }
                break;
            case wallPreset2:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    outtakeSystem.placePos(PlacePosEnum.wallAuto);
                    setPathState(Specimen6AutoEnum.frontWallToWall2);
                }
                break;
            //part 5 placing specimen 3 ***********************************************************~
            //front Wall To Wall
            case frontWallToWall2:
                if (follower.getErrorDistance(frontWall) < 1.5 && !doTransfer) {
                    follower.followPath(frontWallToWall);
                    setPathState(Specimen6AutoEnum.grabWall2);
                }
                break;
            //grab off wall and go to sub
            case grabWall2:
                if (follower.getErrorDistance(pickupWall) < 1.5) {
                    outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                    setPathState(Specimen6AutoEnum.goPlaceSub2);
                }
                break;
            //outtake to specimen place pos
            case goPlaceSub2:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(placeSub2Path);
                    outtakeSystem.setVSlidePos(Constants.Outtake.safeFromWallSlides);
                    setPathState(Specimen6AutoEnum.specPreset2);
                }
                break;
            case specPreset2:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    outtakeSystem.placePos(PlacePosEnum.highSpecimen);
                    setPathState(Specimen6AutoEnum.drivePlace2);
                }
                break;
            case drivePlace2:
                if (follower.getErrorDistance(placeSub2.addReturn(frontSubOffset)) < 1.5) {
                    follower.breakFollowing();
                    follower.driveSlam(true);

                    driveTrain.drive(slamSpeed,0,0, DriveSpeedEnum.Auto);
                    setPathState(Specimen6AutoEnum.dropClaw2);
                }
                break;
            //go Front Wall 2
            case dropClaw2:
                driveTrain.drive(slamSpeed,0,0, DriveSpeedEnum.Auto);
                if (follower.getError(placeSub2).getY() < 1.5) {
                    follower.driveSlam(false);

                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    follower.followPath(toFrontWall3);
                    setPathState(Specimen6AutoEnum.wallPreset3);
                }
                break;
            case wallPreset3:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    outtakeSystem.placePos(PlacePosEnum.wallAuto);
                    setPathState(Specimen6AutoEnum.frontWallToWall3);
                }
                break;
            //part 6 placing specimen 4 ***********************************************************~
            //front Wall To Wall
            case frontWallToWall3:
                if (follower.getErrorDistance(frontWall) < 1.5 && !doGoWall && !doTransfer) {
                    follower.followPath(frontWallToWall);
                    setPathState(Specimen6AutoEnum.grabWall3);
                }
                break;
            //grab off wall and go to sub
            case grabWall3:
                if (follower.getErrorDistance(pickupWall) < 1.5) {
                    outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                    setPathState(Specimen6AutoEnum.goPlaceSub3);
                }
                break;
            //outtake to specimen place pos
            case goPlaceSub3:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(placeSub3Path);
                    outtakeSystem.setVSlidePos(Constants.Outtake.safeFromWallSlides);
                    setPathState(Specimen6AutoEnum.specPreset3);
                }
                break;
            case specPreset3:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    outtakeSystem.placePos(PlacePosEnum.highSpecimen);
                    setPathState(Specimen6AutoEnum.drivePlace3);
                }
                break;
            case drivePlace3:
                if (follower.getErrorDistance(placeSub3.addReturn(frontSubOffset)) < 1.5) {
                    follower.breakFollowing();
                    follower.driveSlam(true);

                    driveTrain.drive(slamSpeed,0,0, DriveSpeedEnum.Auto);
                    setPathState(Specimen6AutoEnum.dropClaw3);
                }
                break;
            //go Front Wall 2
            case dropClaw3:
                driveTrain.drive(slamSpeed,0,0, DriveSpeedEnum.Auto);
                if (follower.getError(placeSub3).getY() < 1.5) {
                    follower.driveSlam(false);

                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);

                    follower.followPath(toFrontWall4);
                    setPathState(Specimen6AutoEnum.wallPreset4);

                }
                break;
            case wallPreset4:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    outtakeSystem.placePos(PlacePosEnum.wallAuto);
                    setPathState(Specimen6AutoEnum.frontWallToWall4);
                }
                break;
            //part 7 placing specimen 5 ***********************************************************~
            //front Wall To Wall
            case frontWallToWall4:
                if (follower.getErrorDistance(frontWall) < 1.5 && !doGoWall && !doTransfer) {
                    follower.followPath(frontWallToWall);
                    setPathState(Specimen6AutoEnum.grabWall4);
                }
                break;
            //grab off wall and go to sub
            case grabWall4:
                if (follower.getErrorDistance(pickupWall) < 1.5) {
                    outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                    setPathState(Specimen6AutoEnum.goPlaceSub4);
                }
                break;
            //outtake to specimen place pos
            case goPlaceSub4:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(placeSub4Path);
                    outtakeSystem.setVSlidePos(Constants.Outtake.safeFromWallSlides);
                    setPathState(Specimen6AutoEnum.specPreset4);
                }
                break;
            case specPreset4:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    outtakeSystem.placePos(PlacePosEnum.highSpecimen);
                    setPathState(Specimen6AutoEnum.drivePlace4);
                }
                break;
            case drivePlace4:
                if (follower.getErrorDistance(placeSub4.addReturn(frontSubOffset)) < 1.5) {
                    follower.breakFollowing();
                    follower.driveSlam(true);

                    driveTrain.drive(slamSpeed,0,0, DriveSpeedEnum.Auto);
                    setPathState(Specimen6AutoEnum.dropClaw4);
                }
                break;
            //go Front Wall 2
            case dropClaw4:
                driveTrain.drive(slamSpeed,0,0, DriveSpeedEnum.Auto);
                if (follower.getError(placeSub4).getY() < 1.5) {
                    follower.driveSlam(false);
                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    follower.followPath(toFrontWall5);
                    if (hasOne) {
                        setPathState(Specimen6AutoEnum.wallPreset5);
                    } else {
                        setPathState(Specimen6AutoEnum.goPark);
                    }
                }
                break;

            case wallPreset5:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    outtakeSystem.placePos(PlacePosEnum.wallAuto);
                    setPathState(Specimen6AutoEnum.frontWallToWall5);
                }
                break;
            //part 8 placing specimen 6?!?!?!?!? **************************************************~
            //front Wall To Wall
            case frontWallToWall5:
                if (follower.getErrorDistance(frontWall) < 1.5 && !doGoWall && !doTransfer) {
                    follower.followPath(frontWallToWall);
                    setPathState(Specimen6AutoEnum.grabWall5);
                }
                break;
            //grab off wall and go to sub
            case grabWall5:
                if (follower.getErrorDistance(pickupWall) < 1.5) {
                    outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                    setPathState(Specimen6AutoEnum.goPlaceSub5);
                }
                break;
            //outtake to specimen place pos
            case goPlaceSub5:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(placeSub5Path);
                    outtakeSystem.setVSlidePos(Constants.Outtake.safeFromWallSlides);
                    setPathState(Specimen6AutoEnum.specPreset5);
                }
                break;
            case specPreset5:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    outtakeSystem.placePos(PlacePosEnum.highSpecimen);
                    setPathState(Specimen6AutoEnum.drivePlace5);
                }
                break;
            case drivePlace5:
                if (follower.getErrorDistance(placeSub5.addReturn(frontSubOffset)) < 1.5) {
                    follower.breakFollowing();
                    follower.driveSlam(true);

                    driveTrain.drive(slamSpeed,0,0, DriveSpeedEnum.Auto);
                    setPathState(Specimen6AutoEnum.dropClaw5);
                }
                break;
            //go Front Wall 2
            case dropClaw5:
                driveTrain.drive(slamSpeed,0,0, DriveSpeedEnum.Auto);
                if (follower.getError(placeSub5).getY() < 1.5) {
                    follower.driveSlam(false);
                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    setPathState(Specimen6AutoEnum.goPark);
                }
                break;
            //Park path
            case goPark:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(parkPath);
                    setPathState(Specimen6AutoEnum.teleopPreset);
                }
                break;
            case teleopPreset:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    outtakeSystem.setArmPos(Constants.Outtake.upArm);
                    outtakeSystem.setVSlidePos(0);
                    setPathState(Specimen6AutoEnum.end);
                }
                break;
        }
        // State Machine **************************************************************************~
        if (doStore) {
            switch (storeState) {
                case 0:
                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    outtakeSystem.setVSlidePos(Constants.Outtake.safeFromSpecBar);
                    setStoreState(1);
                    break;
                case 1:
                    if (Math.abs(outtakeSystem.getVSlidePos() - outtakeSystem.getVSlideTargetPos()) < 300) {
                        outtakeSystem.setArmPos(Constants.Outtake.intakeArm);
                        setStoreState(2);
                    }
                    break;
                case 2:
                    if (storeTimer.getElapsedTimeSeconds() > 0.3) {
                        outtakeSystem.setVSlidePos(Constants.Outtake.intakeWaitSlides);
                        setStoreState(3);
                    }
                    break;
                case 3:
                    if (intakeSystem.readyToTranfer() && Math.abs(outtakeSystem.getVSlidePos() - outtakeSystem.getVSlideTargetPos()) < 500) {
                        setStoreState(0);
                        doStore = false;
                    }
                    break;
            }
        }

        if (doTransfer && !doStore) {
            switch (transferState) {
                case 0:
                    if (transferTimer.getElapsedTimeSeconds() > 0.45) {
                        intakeSystem.setIntakePower(Constants.Intake.transferSpeed);
                        setTransferState(1);
                    }
                    break;
                case 1:
                    if (transferTimer.getElapsedTimeSeconds() > 0.2) {
                        outtakeSystem.setVSlidePos(Constants.Outtake.intakeSlides);
                        setTransferState(2);
                    }
                    break;
                case 2:
                    if (transferTimer.getElapsedTimeSeconds() > 0.35) {
                        intakeSystem.setIntakeServoPos(Constants.Intake.wristClear);
                        intakeSystem.setIntakePower(0);
                        outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                        setTransferState(3);
                    }
                    break;
                case 3:
                    if (transferTimer.getElapsedTimeSeconds() > 0.3) {
                        outtakeSystem.setVSlidePos(Constants.Outtake.safeFromClimberBar);
                        setTransferState(4);
                    }
                    break;
                case 4:
                    if (Math.abs(outtakeSystem.getVSlidePos() - outtakeSystem.getVSlideTargetPos()) < 300) {
                        outtakeSystem.setArmPos(Constants.Outtake.intakeWallArm);
                        setTransferState(5);
                    }
                    break;
                case 5:
                    if (transferTimer.getElapsedTimeSeconds() > 0.15) {
                        outtakeSystem.setVSlidePos(Constants.Outtake.intakeWallAutoSlides);
                        setTransferState(6);
                    }
                    break;
                case 6:
                    if (Math.abs(outtakeSystem.getVSlidePos() - outtakeSystem.getVSlideTargetPos()) < 600) {
                        outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                        setTransferState(7);
                    }
                    break;
                case 7:
                    if (transferTimer.getElapsedTimeSeconds() > 0.2) {
                        setTransferState(0);
                        doTransfer = false;
                    }
                    break;
            }
        }

        if (doGoWall) {
            switch (wallState) {
                case 0:
                    doStore = false;
                    doTransfer = false;
                    outtakeSystem.setVSlidePos(Constants.Outtake.safeFromClimberBar);
                    setWallState(1);
                    break;
                case 1:
                    if (Math.abs(outtakeSystem.getVSlidePos() - outtakeSystem.getVSlideTargetPos()) < 100) {
                        outtakeSystem.setArmPos(Constants.Outtake.intakeWallArm);
                        setWallState(2);
                    }
                    break;
                case 2:
                    if (transferTimer.getElapsedTimeSeconds() > 0.2) {
                        outtakeSystem.setVSlidePos(Constants.Outtake.intakeWallAutoSlides);
                        setWallState(3);
                    }
                    break;
                case 3:
                    if (Math.abs(outtakeSystem.getVSlidePos() - outtakeSystem.getVSlideTargetPos()) < 300) {
                        doGoWall = false;
                        setWallState(0);
                    }
            }
        }




    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(Specimen6AutoEnum pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void setStoreState(int sState) {
        storeState = sState;
        storeTimer.resetTimer();
    }

    public void setTransferState(int tState) {
        transferState = tState;
        transferTimer.resetTimer();
    }

    public void setWallState(int wState) {
        wallState = wState;
        wallTimer.resetTimer();
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
        if (pathTimer.getElapsedTimeSeconds() > 1 && driveTrain.isStalled(4) && !recoveryDebounce) {
            recoveryTime = opmodeTimer.getElapsedTimeSeconds();
            recoveryDebounce = true;
            recoverOnce = false;
        } else if (!driveTrain.isStalled(3)) {
            recoveryDebounce = false;
        }

        if (recoveryDebounce && driveTrain.isStalled(4) && opmodeTimer.getElapsedTimeSeconds() > recoveryTime + 3) {
            recoveryTime = opmodeTimer.getElapsedTimeSeconds();
            recoveryPath = follower.getCurrentPath();
            recoveryEnum = pathState;
            setPathState(Specimen6AutoEnum.end);
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
        if (true) {
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


    }

    /** We do not use this because everything should automatically disable **/
    public void stop() {
    }
}