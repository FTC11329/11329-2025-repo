package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedropathing.follower.FollowerConstants;
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
import org.firstinspires.ftc.teamcode.utility.autoEnums.Specimen6SuperAutoEnum;

//Match 1 transfer too long, down with servo then out with intake for 3rd          miss 2
//Match 2 Wiring                                                                   miss 4
//match 3 down with servo then out with intake for 3rd, grabed too early x2        miss 2
//match 4 Bad clip                                                                 miss 0
//match 5 down with servo then out with intake for 3rd, grabbed to early x2        miss 2
//Tournament miss one from arm not coming down early                               miss 1


public class SpecimenAutoSuper6Spec {
    Climber climber;
    Follower follower;
    Attempt89 blockVision;
    Drivetrain driveTrain;
    PowerTakeOff powerTakeOff;
    IntakeSystem intakeSystem;
    OuttakeSystem outtakeSystem;

    private Timer pathTimer, actionTimer, opmodeTimer;

    private Specimen6SuperAutoEnum pathState;

    private final Pose startPose = new Pose(9, -65.3, Math.toRadians(180));


    // Scoring Poses of our robot.
    private final Pose preloadPlace = new Pose(6.25, -33.25, Math.toRadians(90));
    private final Pose placeSub1 = new Pose (8.25, -32, Math.toRadians(90));
    private final Pose placeSub2 = new Pose(10.25, -31.5, Math.toRadians(90));
    private final Pose placeSub3 = new Pose(6.5, -31.25, Math.toRadians(90));
    private final Pose placeSub4 = new Pose(3.5, -30.5, Math.toRadians(90));
    private final Pose placeSub5 = new Pose(1.5, -30, Math.toRadians(90));

    private final Pose pickupWallControlPointFirst = new Pose(38, -49.2, Math.toRadians(90));
    private final Pose pickupWallFirst = new Pose(38, -61.2, Math.toRadians(90));

    private final Pose spike1ControlPoint1 = new Pose(65, -62, 0);
    private final Pose spike1ControlPoint2 = new Pose(22.5, -11, 0);
    private final Pose backSpike1 = new Pose(47.5, -15, Math.toRadians(90));

    private final Pose pushedSpike1 = new Pose(47, -49, Math.toRadians(90));

    private final Pose spike2ControlPoint1 = new Pose(41, -3, 0);
    private final Pose spike2ControlPoint2 = new Pose(68, -3, 0);
    private final Pose pushedSpike2 = new Pose(58, -49, Math.toRadians(90));

    private final Pose spike3ControlPoint1 = new Pose(53, -5, 0);
    private final Pose backSpike3 = new Pose(62.7, -15, Math.toRadians(90));
    private final Pose pickupWallRightSide = new Pose(62.5, -60.5, Math.toRadians(90));

    private final Pose pickupWallControlPoint = new Pose(39, -48, Math.toRadians(90));
    private final Pose pickupWall = new Pose(41, -60, Math.toRadians(90));

    private final Pose frontSubPlaceOffset = new Pose(0, -6, Math.toRadians(0));
    private final Pose frontSubToWallOffset = new Pose(0, -3, Math.toRadians(0));

    private final Pose halfWayFirstWallPose = new Pose((preloadPlace.getX() + preloadPlace.addReturn(frontSubToWallOffset).getX()) / 2, (preloadPlace.getY() + preloadPlace.addReturn(frontSubToWallOffset).getY()) / 2, (preloadPlace.getHeading() + preloadPlace.addReturn(frontSubToWallOffset).getHeading()) / 2);

    private final Pose park = new Pose(58, -58, Math.toRadians(90));


    //Various Variables

    Pose2D visionResult = new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.DEGREES,-1);
    private double loopTime = 0;
    private double slamSpeed = 1;
    private double firstWallWait = 0.05;
    private double secondWallWait = 0.10;
    private double wallWait = 0.05;
    private double visionSlidePos = Constants.Intake.intakeSlidePos;
    private boolean driveShake = false;
    private boolean driveSlam = false;
    private double lastPoseTime = 0;
    private Pose lastPose = new Pose();

    private boolean hasOne = false;
    private boolean firstTimeThrough = true;
    private boolean hasSuper = false;

    private boolean recoveryDebounce = false;
    private double recoveryTime = 0;
    private boolean recovering = false;
    private boolean recoverOnce = false;
    private Specimen6SuperAutoEnum recoveryEnum;
    private Path recoveryPath;

    // These are our Paths and PathChains that we will define in buildPaths()
    private Path scorePreload;

    private Path pickupSpike1Path;
    private Path pushSpike1Path;

    private Path pickupSpike2Path;

    private Path pickupSpike3Path;

    private PathChain toWall1;
    private Path placeSub1Path;

    private PathChain toWall2Super;
    private Path placeSub2SuperPath;

    private Path toWall2;
    private Path placeSub2Path;

    private Path toWall3;
    private Path placeSub3Path;

    private Path toWall4;
    private Path placeSub4Path;

    private Path toWall5;
    private Path placeSub5Path;

    private Path parkPath; //IS THAT A BTD REFERENCE?

    RobotSideEnum robotSide;
    Telemetry telemetry;
    HardwareMap hardwareMap;

    public SpecimenAutoSuper6Spec(HardwareMap hardwareMap, Telemetry telemetry, RobotSideEnum robotSide) {
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
        outtakeSystem = new OuttakeSystem(hardwareMap, robotSide, true);


        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        follower.setStartingPose(startPose);
        blockVision.switchPipeline(robotSide);

        outtakeSystem.setArmPos(Constants.Outtake.initAutoNearWallArm);
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    public void init_loop() {
        Pose2D sample = blockVision.getBlockPosition(true);
        telemetry.addData("Test Block X", sample.getX(DistanceUnit.INCH));
        telemetry.addData("Test Block Y", sample.getY(DistanceUnit.INCH));
        telemetry.addData("Test Block H", sample.getHeading(AngleUnit.DEGREES));
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    public void start() {
        opmodeTimer.resetTimer();

        outtakeSystem.initArm();
        outtakeSystem.setArmPos(Constants.Outtake.upArm);

        setPathState(Specimen6SuperAutoEnum.driveGoScorePreload);
    }

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        double pushingZPAM = 31;
        double firstPlaceZPAM = 50;
        double toPlaceZPAM = 19;
        double firstWallZPAM = 12.5;
        double toWallZPAM = 3;

        scorePreload     = follower.linearPathBuilder(startPose, preloadPlace);
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), preloadPlace.getHeading(), 0.25);
        scorePreload.setZeroPowerAccelerationMultiplier(firstPlaceZPAM);

        toWall1 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(preloadPlace.addReturn(new Pose(0,-5,0))), new Point(halfWayFirstWallPose.addReturn(new Pose(0,-5, 0))))))
                .setConstantHeadingInterpolation(Math.toRadians(135))
                .setZeroPowerAccelerationMultiplier(toWallZPAM)
                .addPath(new BezierCurve(new Point(halfWayFirstWallPose.addReturn(new Pose(0,-5, 0))), new Point(pickupWallControlPointFirst), new Point(pickupWallFirst)))
                .setLinearHeadingInterpolation(Math.toRadians(135), pickupWallFirst.getHeading(), 0.6)
                .setZeroPowerAccelerationMultiplier(toWallZPAM)
                .build();
        //frontWallToWall
        placeSub1Path = follower.linearPathBuilder(pickupWallFirst, placeSub1.addReturn(frontSubPlaceOffset));
        placeSub1Path.setConstantHeadingInterpolation(placeSub1.getHeading());
        placeSub1Path.setZeroPowerAccelerationMultiplier(toPlaceZPAM);

        //super cycle
        toWall2Super = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(placeSub1.addReturn(new Pose(0,-5,0))), new Point(halfWayFirstWallPose.addReturn(new Pose(0,-5, 0))))))
                .setConstantHeadingInterpolation(Math.toRadians(135))
                .setZeroPowerAccelerationMultiplier(toWallZPAM)
                .addPath(new BezierCurve(new Point(halfWayFirstWallPose.addReturn(new Pose(0,-5, 0))), new Point(pickupWallControlPointFirst), new Point(pickupWallFirst)))
                .setLinearHeadingInterpolation(Math.toRadians(135), pickupWallFirst.getHeading(), 0.6)
                .setZeroPowerAccelerationMultiplier(toWallZPAM)
                .build();

        placeSub2SuperPath = follower.linearPathBuilder(pickupWallFirst, placeSub2.addReturn(frontSubPlaceOffset));
        placeSub2SuperPath.setConstantHeadingInterpolation(placeSub2.getHeading());
        placeSub2SuperPath.setZeroPowerAccelerationMultiplier(toPlaceZPAM);

        //regular

        pickupSpike1Path = new Path(new BezierCurve(new Point(preloadPlace), new Point(spike1ControlPoint1), new Point(spike1ControlPoint2), new Point(backSpike1)));
        pickupSpike1Path.setConstantHeadingInterpolation(Math.toRadians(90));

        pushSpike1Path   = follower.linearPathBuilder(backSpike1, pushedSpike1);
        pushSpike1Path.setZeroPowerAccelerationMultiplier(pushingZPAM);

        pickupSpike2Path = new Path(new BezierCurve(new Point(pushedSpike1), new Point(spike2ControlPoint1), new Point(spike2ControlPoint2), new Point(pushedSpike2)));
        pickupSpike2Path.setConstantHeadingInterpolation(Math.toRadians(90));
        pickupSpike2Path.setZeroPowerAccelerationMultiplier(pushingZPAM);

        pickupSpike3Path = new Path(new BezierCurve(new Point(pushedSpike2), new Point(spike3ControlPoint1), new Point(backSpike3)));
        pickupSpike3Path.setConstantHeadingInterpolation(Math.toRadians(90));
        pickupSpike3Path.setZeroPowerAccelerationMultiplier(pushingZPAM);


        toWall2 = follower.linearPathBuilder(backSpike3, pickupWallRightSide);
        toWall2.setConstantHeadingInterpolation(backSpike3.getHeading());
        toWall2.setZeroPowerAccelerationMultiplier(firstWallZPAM);
        //frontWallToWall
        placeSub2Path = follower.linearPathBuilder(pickupWallRightSide, placeSub2.addReturn(frontSubPlaceOffset));
        placeSub2Path.setConstantHeadingInterpolation(placeSub2.getHeading());
        placeSub2Path.setZeroPowerAccelerationMultiplier(toPlaceZPAM);


        toWall3 = new Path(new BezierCurve(new Point(placeSub2.addReturn(frontSubToWallOffset)), new Point(pickupWallControlPoint), new Point(pickupWall)));
        toWall3.setConstantHeadingInterpolation(placeSub2.getHeading());
        toWall3.setZeroPowerAccelerationMultiplier(toWallZPAM);
        //frontWallToWall
        placeSub3Path = follower.linearPathBuilder(pickupWall, placeSub3.addReturn(frontSubPlaceOffset));
        placeSub3Path.setConstantHeadingInterpolation(placeSub3.getHeading());
        placeSub3Path.setZeroPowerAccelerationMultiplier(toPlaceZPAM);


        toWall4 = new Path(new BezierCurve(new Point(placeSub3.addReturn(frontSubToWallOffset)), new Point(pickupWallControlPoint), new Point(pickupWall)));
        toWall4.setConstantHeadingInterpolation(placeSub3.getHeading());
        toWall4.setZeroPowerAccelerationMultiplier(toWallZPAM);
        //frontWallToWall
        placeSub4Path = follower.linearPathBuilder(pickupWall, placeSub4.addReturn(frontSubPlaceOffset));
        placeSub4Path.setConstantHeadingInterpolation(placeSub4.getHeading());
        placeSub4Path.setZeroPowerAccelerationMultiplier(toPlaceZPAM);

        toWall5 = new Path(new BezierCurve(new Point(placeSub4.addReturn(frontSubToWallOffset)), new Point(pickupWallControlPoint), new Point(pickupWall)));
        toWall5.setConstantHeadingInterpolation(placeSub4.getHeading());
        toWall5.setZeroPowerAccelerationMultiplier(toWallZPAM);
        //frontWallToWall
        placeSub5Path = follower.linearPathBuilder(pickupWall, placeSub5.addReturn(frontSubPlaceOffset));
        placeSub5Path.setConstantHeadingInterpolation(placeSub5.getHeading());
        placeSub5Path.setZeroPowerAccelerationMultiplier(toPlaceZPAM);

        parkPath = follower.linearPathBuilder(placeSub5, park);
        parkPath.setZeroPowerAccelerationMultiplier(pushingZPAM);

    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState(Specimen6SuperAutoEnum.) method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            //go to score preload
            case driveGoScorePreload:
                follower.followPath(scorePreload);
                blockVision.switchPipeline(robotSide);

                outtakeSystem.setVSlidePos(Constants.Outtake.highSpecimenSlides - 50);
                outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                outtakeSystem.setArmPos(Constants.Outtake.specimenArm);
                intakeSystem.setHSlidePos(75);

                setPathState(Specimen6SuperAutoEnum.placePreload);
                break;
            //go to pickup spike1
            case placePreload:
                //Checks if you are less than 1 inch away from target pos
                if (follower.getError(preloadPlace).getX() < 1.5 && follower.getError(preloadPlace).getY() < 1.5 || pathTimer.getElapsedTimeSeconds() > 3) {
                    setPathState(Specimen6SuperAutoEnum.dropClaw0);
                }
                break;
            case dropClaw0:
                if (pathTimer.getElapsedTimeSeconds() > 0.05) {
                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    setPathState(Specimen6SuperAutoEnum.visionLook);
                }
                break;
            //go to pickup wall preset
            case visionLook:
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    intakeSystem.setHSlidePos(0);
                    visionResult = blockVision.getBlockPosition(true);
                    if (visionResult.getHeading(AngleUnit.DEGREES) != -1) {
                        outtakeSystem.placePos(PlacePosEnum.wall);
                        outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                        intakeSystem.setHSlidesInches(follower.followYourHead(visionResult));
                        setPathState(Specimen6SuperAutoEnum.drivingVision);

                    } else if (pathTimer.getElapsedTimeSeconds() > 0.15) {
                        outtakeSystem.placePos(PlacePosEnum.wall);
                        outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                        intakeSystem.storePos();
                        hasOne = false;
                        follower.followPath(pickupSpike1Path);
                        if (firstTimeThrough) {
                            setPathState(Specimen6SuperAutoEnum.missFirstIntake);
                        } else {
                            setPathState(Specimen6SuperAutoEnum.dropClaw2);
                        }
                    }
                }
                break;
            case drivingVision:
                if ((pathTimer.getElapsedTimeSeconds() > 0.2 && Math.abs(intakeSystem.getHSlideTargetPos() - intakeSystem.getHSlidePos()) < 250 && follower.getHeadingError() < Math.toRadians(2)) || pathTimer.getElapsedTimeSeconds() > 0.5) {
                    intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
                    setPathState(Specimen6SuperAutoEnum.startIntake);
                }
                break;
            case startIntake:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    driveShake = true;
                    follower.startTeleopDrive();
                    intakeSystem.intakeUntilColor();
                    setPathState(Specimen6SuperAutoEnum.intakingWithVision);
                }
                break;
            case intakingWithVision:
                intakeSystem.update();
                if (intakeSystem.intakeUntilColor()) {
                    actionTimer.resetTimer();

                    intakeSystem.storePos();
                    intakeSystem.setIntakePower(Constants.Intake.unjamSpeed);
                    driveShake = false;
                    follower.breakFollowing();
                    hasOne = true;
                    hasSuper = true;

                    if (firstTimeThrough) {
                        follower.followPath(toWall1);
                    } else {
                        follower.followPath(toWall2Super);
                    }
                    setPathState(Specimen6SuperAutoEnum.unjam);

                } else if (pathTimer.getElapsedTimeSeconds() > 2) {
                    intakeSystem.storePos();
                    intakeSystem.setIntakePower(Constants.Intake.spitSpeed);
                    follower.breakFollowing();
                    driveShake = false;
                    hasOne = false;
                    if (firstTimeThrough) {
                        setPathState(Specimen6SuperAutoEnum.missFirstIntake);
                    } else {
                        setPathState(Specimen6SuperAutoEnum.dropClaw2);
                    }
                }
                break;
            case unjam:
                if (pathTimer.getElapsedTimeSeconds() > Constants.Intake.unjamTimeMillisAuto / 1000) {
                    intakeSystem.setIntakePower(Constants.Intake.intakeSpeed);
                    setPathState(Specimen6SuperAutoEnum.reIntake);
                }
                break;
            case reIntake:
                if (intakeSystem.intakeUntil() || pathTimer.getElapsedTimeSeconds() > Constants.Intake.unjamTimeMillisAuto / 1000){
                    intakeSystem.setIntakePower(0);
                    setPathState(Specimen6SuperAutoEnum.transferToTray);
                }
            case transferToTray:
                if (actionTimer.getElapsedTimeSeconds() > 0.65) {
                    intakeSystem.setIntakePower(Constants.Intake.transferSpeed);

                    setPathState(Specimen6SuperAutoEnum.wallPreset2);
                }
                break;
            case missFirstIntake:
                if (intakeSystem.getHSlideTargetPos() < 100 || pathTimer.getElapsedTimeSeconds() > 2) {
                    if (firstTimeThrough) {
                        follower.followPath(toWall1);
                    } else {
                        follower.followPath(toWall2Super);
                    }
                    setPathState(Specimen6SuperAutoEnum.wallPreset2);
                }

            case wallPreset2:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    outtakeSystem.placePos(PlacePosEnum.wallAuto);
                    intakeSystem.setIntakeServoPos(Constants.Intake.wristDepo);
                    setPathState(Specimen6SuperAutoEnum.depoThePickup);
                }
                break;
            //change to be transfer and dump
            case depoThePickup:
                if (pathTimer.getElapsedTimeSeconds() > 0.4) {
                    intakeSystem.setDepoServoPos(Constants.Intake.depoDepo);

                    setPathState(Specimen6SuperAutoEnum.grabWall2);
                }
                break;
            case grabWall2:
                if (!((follower.getErrorDistance(pickupWallFirst) < 2 && outtakeSystem.seesWall()) || pathTimer.getElapsedTimeSeconds() > 2)) {
                    actionTimer.resetTimer();
                }
                if (actionTimer.getElapsedTimeSeconds() > firstWallWait) {
                    intakeSystem.setIntakeServoPos(Constants.Intake.wristClear);
                    outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                    setPathState(Specimen6SuperAutoEnum.goPlaceSub2);
                }
                break;
            //outtake to specimen place pos
            case goPlaceSub2:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    intakeSystem.setIntakePower(Constants.Intake.spitSpeed);
                    if (firstTimeThrough) {
                        follower.followPath(placeSub1Path);
                    } else {
                        follower.followPath(placeSub2SuperPath);
                    }
                    outtakeSystem.setVSlidePos(Constants.Outtake.safeFromWallSlides);
                    setPathState(Specimen6SuperAutoEnum.specPreset2);
                }
                break;
            case specPreset2:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    intakeSystem.setIntakeServoPos(Constants.Intake.wristStore);
                    intakeSystem.setDepoServoPos(Constants.Intake.depoStore);
                    outtakeSystem.placePos(PlacePosEnum.highSpecimen);
                    setPathState(Specimen6SuperAutoEnum.drivePlace2);
                }
                break;
            case drivePlace2:
                Pose target;
                if (firstTimeThrough) {
                    target = placeSub1.addReturn(frontSubPlaceOffset);
                } else {
                    target = placeSub2.addReturn(frontSubPlaceOffset);
                }
                if (follower.getErrorDistance(target) < 1.5) {
                    intakeSystem.setIntakePower(0);

                    follower.startTeleopDrive();
                    driveSlam = true;

                    setPathState(Specimen6SuperAutoEnum.dropClaw2);
                }
                break;
            //go Front Wall 2
            case dropClaw2:
                driveTrain.drive(slamSpeed,0,0, DriveSpeedEnum.Auto);
                if (follower.getError(placeSub2).getY() < 1.5 || !firstTimeThrough) {
                    // teleop drive = false
                    follower.breakFollowing();
                    driveSlam = false;

                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    outtakeSystem.setArmPos(Constants.Outtake.upArm);
                    follower.followPath(pickupSpike1Path);

                    if (firstTimeThrough) {
                        setPathState(Specimen6SuperAutoEnum.visionLook);
                        firstTimeThrough = false;
                    } else {
                        outtakeSystem.setVSlidePos(0);
                        follower.setCentripetalScaling(0.0007);
                        setPathState(Specimen6SuperAutoEnum.pushSpike1);
                    }
                }
                break;

            case pushSpike1:
                if (follower.getError(backSpike1).getX() < 4 && follower.getError(backSpike1).getY() < 4) {
                    intakeSystem.setHSlidePos(Constants.Intake.intakeSlidePos);
                    follower.followPath(pushSpike1Path);
                    setPathState(Specimen6SuperAutoEnum.goBackSpike2);
                }
                break;
            case goBackSpike2:
                if (follower.getError(pushedSpike1).getX() < 2 && follower.getError(pushedSpike1).getY() < 2) {
                    intakeSystem.storePos();
                    follower.followPath(pickupSpike2Path);
                    setPathState(Specimen6SuperAutoEnum.backSpike2);
                }
                break;
            case backSpike2:
                if (follower.getPose().getY() > -15 || pathTimer.getElapsedTimeSeconds() > 2) {
                    setPathState(Specimen6SuperAutoEnum.pushingSpike2);
                }
                break;
            case pushingSpike2:
                if (follower.getPose().getY() < -14) {
                    intakeSystem.setHSlidePos(Constants.Intake.intakeSlidePos);
                    setPathState(Specimen6SuperAutoEnum.goBackSpike3);
                }
                break;
            //go to pickup spike3
            case goBackSpike3:
                if (follower.getError(pushedSpike2).getX() < 2 && follower.getError(pushedSpike2).getY() < 2) {
                    intakeSystem.setHSlidePos(0);
                    if (hasSuper) {
                        follower.followPath(toWall2);
                        setPathState(Specimen6SuperAutoEnum.grabWall1);
                    } else {
                        follower.followPath(pickupSpike3Path);
                        setPathState(Specimen6SuperAutoEnum.pushSpike3);
                    }
                }
                break;
            //go to front pickup wall
            case pushSpike3:
                if (follower.getError(backSpike3).getX() < 4 && follower.getError(backSpike3).getY() < 4) {
                    follower.followPath(toWall2);
                    outtakeSystem.placePos(PlacePosEnum.wallAuto);
                    setPathState(Specimen6SuperAutoEnum.grabWall1);
                }
                break;
            //grab off wall and go to sub
            case grabWall1:
                if ((outtakeSystem.seesWall()) || pathTimer.getElapsedTimeSeconds() > 2) {
                    intakeSystem.setDepoServoPos(Constants.Intake.depoStore);
                } else {
                    actionTimer.resetTimer();
                }
                if (actionTimer.getElapsedTimeSeconds() > secondWallWait) {
                    follower.setCentripetalScaling(FollowerConstants.centripetalScaling);
                    outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                    setPathState(Specimen6SuperAutoEnum.goPlaceSub1);
                }
                break;
            //outtake to specimen place pos
            case goPlaceSub1:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(placeSub2Path);
                    outtakeSystem.setVSlidePos(Constants.Outtake.safeFromWallSlides);
                    if (hasSuper) {
                        setPathState(Specimen6SuperAutoEnum.specPreset2);
                    } else {
                        setPathState(Specimen6SuperAutoEnum.specPreset1);
                    }
                }
                break;
            case specPreset1:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    outtakeSystem.placePos(PlacePosEnum.highSpecimen);
                    setPathState(Specimen6SuperAutoEnum.drivePlace1);
                }
                break;
            case drivePlace1:
                if (follower.getErrorDistance(placeSub2.addReturn(frontSubPlaceOffset)) < 1.5) {
                    follower.startTeleopDrive();
                    driveSlam = true;

                    setPathState(Specimen6SuperAutoEnum.dropClaw1);
                }
                break;
            //go Front Wall 2
            case dropClaw1:
                if (follower.getError(placeSub2).getY() < 1.5) {
                    // teleop drive = false
                    follower.breakFollowing();
                    driveSlam = false;

                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    follower.followPath(toWall3);
                    setPathState(Specimen6SuperAutoEnum.wallPreset3);
                }
                break;

            case wallPreset3:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    outtakeSystem.placePos(PlacePosEnum.wallAuto);
                    setPathState(Specimen6SuperAutoEnum.grabWall3);
                }
                break;
            //part 6 placing specimen 4 ***********************************************************~
            //grab off wall and go to sub
            case grabWall3:
                if ((follower.getErrorDistance(pickupWall) < 4 && outtakeSystem.seesWall()) || pathTimer.getElapsedTimeSeconds() > 2) {
                    intakeSystem.setDepoServoPos(Constants.Intake.depoStore);
                } else {
                    actionTimer.resetTimer();
                }
                if (actionTimer.getElapsedTimeSeconds() > wallWait) {
                    outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                    setPathState(Specimen6SuperAutoEnum.goPlaceSub3);
                }
                break;
            //outtake to specimen place pos
            case goPlaceSub3:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(placeSub3Path);
                    outtakeSystem.setVSlidePos(Constants.Outtake.safeFromWallSlides);
                    setPathState(Specimen6SuperAutoEnum.specPreset3);
                }
                break;
            case specPreset3:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    outtakeSystem.placePos(PlacePosEnum.highSpecimen);
                    setPathState(Specimen6SuperAutoEnum.drivePlace3);
                }
                break;
            case drivePlace3:
                if (follower.getErrorDistance(placeSub3.addReturn(frontSubPlaceOffset)) < 1.5) {
                    follower.startTeleopDrive();
                    driveSlam = true;

                    setPathState(Specimen6SuperAutoEnum.dropClaw3);
                }
                break;
            //go Front Wall 2
            case dropClaw3:
                if (follower.getError(placeSub3).getY() < 1.5) {
                    // teleop drive = false
                    follower.breakFollowing();
                    driveSlam = false;

                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);

                    follower.followPath(toWall4);
                    setPathState(Specimen6SuperAutoEnum.wallPreset4);

                }
                break;
            case wallPreset4:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    outtakeSystem.placePos(PlacePosEnum.wallAuto);
                    setPathState(Specimen6SuperAutoEnum.grabWall4);
                }
                break;
            //part 7 placing specimen 5 ***********************************************************~
            //grab off wall and go to sub
            case grabWall4:
                if ((follower.getErrorDistance(pickupWall) < 4 && outtakeSystem.seesWall()) || pathTimer.getElapsedTimeSeconds() > 2) {
                    intakeSystem.setDepoServoPos(Constants.Intake.depoStore);
                } else {
                    actionTimer.resetTimer();
                }
                if (actionTimer.getElapsedTimeSeconds() > wallWait) {
                    outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                    setPathState(Specimen6SuperAutoEnum.goPlaceSub4);
                }
                break;
            //outtake to specimen place pos
            case goPlaceSub4:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(placeSub4Path);
                    outtakeSystem.setVSlidePos(Constants.Outtake.safeFromWallSlides);
                    setPathState(Specimen6SuperAutoEnum.specPreset4);
                }
                break;
            case specPreset4:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    outtakeSystem.placePos(PlacePosEnum.highSpecimen);
                    setPathState(Specimen6SuperAutoEnum.drivePlace4);
                }
                break;
            case drivePlace4:
                if (follower.getErrorDistance(placeSub4.addReturn(frontSubPlaceOffset)) < 1.5) {
                    follower.startTeleopDrive();
                    driveSlam = true;

                    setPathState(Specimen6SuperAutoEnum.dropClaw4);
                }
                break;
            //go Front Wall 2
            case dropClaw4:
                if (follower.getError(placeSub4).getY() < 1.5) {
                    // teleop drive = false
                    follower.breakFollowing();
                    driveSlam = false;

                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    follower.followPath(toWall5);
//                    if (hasOne && opmodeTimer.getElapsedTimeSeconds() < 29) {
                        setPathState(Specimen6SuperAutoEnum.wallPreset5);
//                    } else {
//                        setPathState(Specimen6SuperAutoEnum.goPark);
//                    }
                }
                break;

            case wallPreset5:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    outtakeSystem.placePos(PlacePosEnum.wallAuto);
                    setPathState(Specimen6SuperAutoEnum.grabWall5);
                }
                break;
            //part 8 placing specimen 6?!?!?!?!? **************************************************~
            //grab off wall and go to sub
            case grabWall5:
                if ((follower.getErrorDistance(pickupWall) < 4 && outtakeSystem.seesWall()) || pathTimer.getElapsedTimeSeconds() > 2) {
                    intakeSystem.setDepoServoPos(Constants.Intake.depoStore);
                } else {
                    actionTimer.resetTimer();
                }
                if (actionTimer.getElapsedTimeSeconds() > wallWait) {
                    outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                    setPathState(Specimen6SuperAutoEnum.goPlaceSub5);
                }
                break;
            //outtake to specimen place pos
            case goPlaceSub5:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(placeSub5Path);
                    outtakeSystem.setVSlidePos(Constants.Outtake.safeFromWallSlides);
                    setPathState(Specimen6SuperAutoEnum.specPreset5);
                }
                break;
            case specPreset5:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    outtakeSystem.placePos(PlacePosEnum.highSpecimen);
                    setPathState(Specimen6SuperAutoEnum.drivePlace5);
                }
                break;
            case drivePlace5:
                if (follower.getErrorDistance(placeSub5.addReturn(frontSubPlaceOffset)) < 1.5) {
                    follower.startTeleopDrive();
                    driveSlam = true;

                    setPathState(Specimen6SuperAutoEnum.dropClaw5);
                }
                break;
            //go Front Wall 2
            case dropClaw5:
                if (follower.getError(placeSub5).getY() < 1.5) {
//                     teleop drive = false
                    follower.breakFollowing();
                    driveSlam = false;

                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    outtakeSystem.setArmPos(Constants.Outtake.upArm);
                    outtakeSystem.setVSlidePos(0);
                    setPathState(Specimen6SuperAutoEnum.goPark);
                }
                break;
            //Park path
            case goPark:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(parkPath);
                    setPathState(Specimen6SuperAutoEnum.teleopPreset);
                }
                break;
            case teleopPreset:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    setPathState(Specimen6SuperAutoEnum.end);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(Specimen6SuperAutoEnum pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    public void loop() {
        // These loop the movements of the robot
        if (driveSlam) {
            follower.setTeleOpMovementVectors(1, 0, 0);
        }
        follower.update();

        autonomousPathUpdate();
        if (driveShake && pathTimer.getElapsedTimeSeconds() > 0.5) {
            if (Math.round((pathTimer.getElapsedTimeSeconds() - 0.5) * 3) % 2 == 0 ){
                follower.setTeleOpMovementVectors(0,0, -0.3);
            } else {
                follower.setTeleOpMovementVectors(0,0, 0.3);
            }
        }
        //Dumb path recovery
        if (pathTimer.getElapsedTimeSeconds() > 1 && driveTrain.isStalled(5) && !recoveryDebounce) {
            recoveryTime = opmodeTimer.getElapsedTimeSeconds();
            recoveryDebounce = true;
            recoverOnce = false;
        } else if (!driveTrain.isStalled(5)) {
            recoveryDebounce = false;
        }

        if (recoveryDebounce && driveTrain.isStalled(5) && opmodeTimer.getElapsedTimeSeconds() > recoveryTime + 2.5) {
            recoveryTime = opmodeTimer.getElapsedTimeSeconds();
            recoveryPath = follower.getCurrentPath();
            recoveryEnum = pathState;
            setPathState(Specimen6SuperAutoEnum.end);
            recovering = true;
        }

        if (recovering) {
            if (!recoverOnce) {
                follower.followPath(parkPath);
                outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                recoverOnce = true;
            }
            if (follower.getErrorDistance(park) < 3 && pathTimer.getElapsedTimeSeconds() > 0.75) {
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
//            telemetry.addData("x", follower.getPose().getX());
//            telemetry.addData("y", follower.getPose().getY());
//            telemetry.addData("heading", follower.getPose().getHeading());
//            telemetry.addData("tripped", driveTrain.isStalled(3));
//            telemetry.addData("opmode", opmodeTimer.getElapsedTimeSeconds());
//            telemetry.addData("loopTime", loopTime - opmodeTimer.getElapsedTime());
//            telemetry.addData("fl", driveTrain.getDrivePowers()[0]);
//            telemetry.addData("bl", driveTrain.getDrivePowers()[1]);
//            telemetry.addData("fr", driveTrain.getDrivePowers()[2]);
//            telemetry.addData("br", driveTrain.getDrivePowers()[3]);
//            telemetry.addData("ma", Math.max(Math.max(driveTrain.getDriveCurrent()[0], driveTrain.getDriveCurrent()[1]), Math.max(driveTrain.getDriveCurrent()[2], driveTrain.getDriveCurrent()[3])));
            loopTime = opmodeTimer.getElapsedTime();
            telemetry.update();
        }
    }

    public void stop() {
        driveTrain.stopDrive();
    }
}