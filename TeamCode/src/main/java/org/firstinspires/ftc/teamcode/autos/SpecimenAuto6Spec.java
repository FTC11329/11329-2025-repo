package org.firstinspires.ftc.teamcode.autos;

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

    private Specimen6AutoEnum pathState;

    private final Pose startPose = new Pose(9, -65.3, Math.toRadians(180));


    // Scoring Poses of our robot. */
    private final Pose preloadPlace = new Pose(6, -32.5, Math.toRadians(90));
    private final Pose placeSub1 = new Pose(11, -32.5, Math.toRadians(90));
    private final Pose placeSub2 = new Pose(7, -33, Math.toRadians(90));
    private final Pose placeSub3 = new Pose(5.5, -33.5, Math.toRadians(90));
    private final Pose placeSub4 = new Pose(5, -33.5, Math.toRadians(90));
    private final Pose placeSub5 = new Pose(3, -34, Math.toRadians(90));

    private final Pose spike1AfterSpitControlPoint1 = new Pose(30, -12, 0);
    private final Pose spitFirstPose = new Pose(36, -43, Math.toRadians(290));
    private final Pose halfWaySpitFirstPose = new Pose((preloadPlace.getX() + spitFirstPose.getX()) / 2, (preloadPlace.getY() + spitFirstPose.getY()) / 2, (preloadPlace.getHeading() + spitFirstPose.getHeading()) / 2);

    private final Pose spike1ControlPoint1 = new Pose(65, -62, 0);
    private final Pose spike1ControlPoint2 = new Pose(22.5, -11, 0);
    private final Pose backSpike1 = new Pose(48, -17, Math.toRadians(90));

    private final Pose pushedSpike1 = new Pose(47, -50, Math.toRadians(90));

    private final Pose spike2ControlPoint1 = new Pose(41, -2, 0);
    private final Pose spike2ControlPoint2 = new Pose(68, -2, 0);
    private final Pose pushedSpike2 = new Pose(58, -50, Math.toRadians(90));

    private final Pose spike3ControlPoint1 = new Pose(53, -5, 0);
    private final Pose backSpike3 = new Pose(62.75, -15, Math.toRadians(90));

    private final Pose pickupWall = new Pose(39, -62.25, Math.toRadians(90));

    private final Pose pickupWallRightSide = new Pose(62.75, -63, Math.toRadians(90));
    private final Pose frontSubPlaceOffset = new Pose(0, -15, Math.toRadians(0));
    private final Pose frontSubToWallOffset = new Pose(0, -5, Math.toRadians(0));

    private final Pose park = new Pose(58, -58, Math.toRadians(90));


    //Various Variables

    Pose2D visionResult = new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.DEGREES,-1);
    private double loopTime = 0;
    private double slamSpeed = 0.8;
    private boolean driveShake = false;

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
    private PathChain spitFirstPath;

    private Path pickupSpike1PathAfterSpit;
    private Path pickupSpike1Path;
    private Path pushSpike1Path;

    private Path pickupSpike2Path;

    private Path pickupSpike3Path;

    private Path toWall1;
    private Path placeSub1Path;

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
        outtakeSystem = new OuttakeSystem(hardwareMap, false);


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

        outtakeSystem.initArm();
        outtakeSystem.setArmPos(Constants.Outtake.upArm);

        setPathState(Specimen6AutoEnum.driveGoScorePreload);
    }

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        double fastZeroPower = 25;
        double medFastZeroPower = 13;

        scorePreload     = follower.linearPathBuilder(startPose, preloadPlace);
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), preloadPlace.getHeading(), 0.25);
        scorePreload.setZeroPowerAccelerationMultiplier(medFastZeroPower);

        spitFirstPath = follower.pathBuilder().addPath(follower.linearPathBuilder(preloadPlace, halfWaySpitFirstPose))
                .setLinearHeadingInterpolation(preloadPlace.getHeading(), halfWaySpitFirstPose.getHeading())
                .addPath(follower.linearPathBuilder(halfWaySpitFirstPose, spitFirstPose))
                .setLinearHeadingInterpolation(halfWaySpitFirstPose.getHeading(), spitFirstPose.getHeading())
                .build();

        pickupSpike1PathAfterSpit = new Path(new BezierCurve(new Point(spitFirstPose), new Point(spike1AfterSpitControlPoint1), new Point(backSpike1)));
        pickupSpike1PathAfterSpit.setLinearHeadingInterpolation(spitFirstPose.getHeading(), backSpike1.getHeading(), 0.2);

        pickupSpike1Path = new Path(new BezierCurve(new Point(preloadPlace), new Point(spike1ControlPoint1), new Point(spike1ControlPoint2), new Point(backSpike1)));
        pickupSpike1Path.setConstantHeadingInterpolation(Math.toRadians(90));

        pushSpike1Path   = follower.linearPathBuilder(backSpike1, pushedSpike1);
        pushSpike1Path.setZeroPowerAccelerationMultiplier(fastZeroPower);

        pickupSpike2Path = new Path(new BezierCurve(new Point(pushedSpike1), new Point(spike2ControlPoint1), new Point(spike2ControlPoint2), new Point(pushedSpike2)));
        pickupSpike2Path.setConstantHeadingInterpolation(Math.toRadians(90));
        pickupSpike2Path.setZeroPowerAccelerationMultiplier(fastZeroPower);

        pickupSpike3Path = new Path(new BezierCurve(new Point(pushedSpike2), new Point(spike3ControlPoint1), new Point(backSpike3)));
        pickupSpike3Path.setConstantHeadingInterpolation(Math.toRadians(90));

        toWall1 = follower.linearPathBuilder(backSpike3, pickupWallRightSide);
        //frontWallToWall
        placeSub1Path = follower.linearPathBuilder(pickupWallRightSide, placeSub1.addReturn(frontSubPlaceOffset));
        placeSub1Path.setConstantHeadingInterpolation(placeSub1.getHeading());
        placeSub1Path.setZeroPowerAccelerationMultiplier(medFastZeroPower);


        toWall2 = follower.linearPathBuilder(placeSub1.addReturn(frontSubToWallOffset), pickupWall);
        toWall2.setConstantHeadingInterpolation(placeSub1.getHeading());
        //frontWallToWall
        placeSub2Path = follower.linearPathBuilder(pickupWall, placeSub2.addReturn(frontSubPlaceOffset));
        placeSub2Path.setConstantHeadingInterpolation(placeSub2.getHeading());
        placeSub2Path.setZeroPowerAccelerationMultiplier(medFastZeroPower);


        toWall3 = follower.linearPathBuilder(placeSub2.addReturn(frontSubToWallOffset), pickupWall);
        toWall3.setConstantHeadingInterpolation(placeSub2.getHeading());
        //frontWallToWall
        placeSub3Path = follower.linearPathBuilder(pickupWall, placeSub3.addReturn(frontSubPlaceOffset));
        placeSub3Path.setConstantHeadingInterpolation(placeSub3.getHeading());
        placeSub3Path.setZeroPowerAccelerationMultiplier(medFastZeroPower);


        toWall4 = follower.linearPathBuilder(placeSub3.addReturn(frontSubToWallOffset), pickupWall);
        toWall4.setConstantHeadingInterpolation(placeSub3.getHeading());
        //frontWallToWall
        placeSub4Path = follower.linearPathBuilder(pickupWall, placeSub4.addReturn(frontSubPlaceOffset));
        placeSub4Path.setConstantHeadingInterpolation(placeSub4.getHeading());
        placeSub4Path.setZeroPowerAccelerationMultiplier(medFastZeroPower);

        toWall5 = follower.linearPathBuilder(placeSub4.addReturn(frontSubToWallOffset), pickupWall);
        toWall5.setConstantHeadingInterpolation(placeSub4.getHeading());
        //frontWallToWall
        placeSub5Path = follower.linearPathBuilder(pickupWall, placeSub5.addReturn(frontSubPlaceOffset));
        placeSub5Path.setConstantHeadingInterpolation(placeSub5.getHeading());
        placeSub5Path.setZeroPowerAccelerationMultiplier(medFastZeroPower);

        parkPath = follower.linearPathBuilder(placeSub5.addReturn(frontSubPlaceOffset), park);

    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState(Specimen6AutoEnum.) method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            //go to score preload
            case driveGoScorePreload:
                follower.followPath(scorePreload);

                blockVision.switchPipeline(robotSide);
                outtakeSystem.setVSlidePos(Constants.Outtake.highSpecimenSlides);
                outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                outtakeSystem.setArmPos(Constants.Outtake.specimenArm);

                setPathState(Specimen6AutoEnum.placePreload);
                break;
            //go to pickup spike1
            case placePreload:
                //Checks if you are less than 1 inch away from target pos
                if (follower.getError(preloadPlace).getX() < 1 && follower.getError(preloadPlace).getY() < 1 && pathTimer.getElapsedTimeSeconds() > .17) {
                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    setPathState(Specimen6AutoEnum.visionLook);
                }
                break;
            //go to pickup wall preset
            case visionLook:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    //wait for robot to fall & stop moving
                    visionResult = blockVision.getBestSpecimen();
                    if (visionResult.getHeading(AngleUnit.DEGREES) != -1) {
                        outtakeSystem.setArmPos(Constants.Outtake.upArm);
                        outtakeSystem.setVSlidePos(0);

                        intakeSystem.setHSlidesInches(follower.followYourHead(visionResult));
                        setPathState(Specimen6AutoEnum.drivingVision);

                    } else if (pathTimer.getElapsedTimeSeconds() > 1) {
                        outtakeSystem.setArmPos(Constants.Outtake.upArm);
                        outtakeSystem.setVSlidePos(0);
                        hasOne = false;
                        follower.followPath(pickupSpike1Path);
                        setPathState(Specimen6AutoEnum.pushSpike1);
                    }
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
                    intakeSystem.storeOutPos();
                    hasOne = true;

                    follower.followPath(spitFirstPath);
                    setPathState(Specimen6AutoEnum.extendHSlidesSpit);

                } else if (pathTimer.getElapsedTimeSeconds() > 2) {
                    doGoWall = true;
                    intakeSystem.storePos();
                    hasOne = false;

                    follower.followPath(pickupSpike1Path);
                    setPathState(Specimen6AutoEnum.pushSpike1);
                }
                break;
            case extendHSlidesSpit:
                if (follower.getPose().getX() > halfWaySpitFirstPose.getX()) {
                    intakeSystem.setHSlidePos(Constants.Intake.intakeSlidePos);

                    setPathState(Specimen6AutoEnum.spitThePickup);
                }
                break;
            case spitThePickup:
                if (follower.getError(spitFirstPose).getHeading() < Math.toRadians(7)) {
                    intakeSystem.setIntakeServoPos(Constants.Intake.wristClear);
                    intakeSystem.setIntakePower(Constants.Intake.spitSpeed);

                    setPathState(Specimen6AutoEnum.stopSpit);
                }
                break;
            case stopSpit:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    intakeSystem.storePos();

                    follower.followPath(pickupSpike1PathAfterSpit);
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
                    follower.followPath(toWall1);
                    outtakeSystem.placePos(PlacePosEnum.wallAuto);
                    setPathState(Specimen6AutoEnum.grabWall1);
                }
                break;
//            case waitFrontWall1:
//                if (follower.getError(pushedSpike3).getY() < 2.5) {
//                    setPathState(Specimen6AutoEnum.pickupWall1);
//                }
//                break;
//            case pickupWall1:
//                if (pathTimer.getElapsedTimeSeconds() > 0.15) {
//                    follower.followPath(toWall1);
//                    setPathState(Specimen6AutoEnum.grabWall1);
//                }
//                break;
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
                if (follower.getErrorDistance(placeSub1.addReturn(frontSubPlaceOffset)) < 1.5) {
                    follower.breakFollowing();
                    follower.driveSlam(true);

                    driveTrain.drive(slamSpeed,0,0, DriveSpeedEnum.Auto);
                    setPathState(Specimen6AutoEnum.dropClaw1);
                }
                break;
            //go Front Wall 2
            case dropClaw1:
                driveTrain.drive(slamSpeed,0,0, DriveSpeedEnum.Auto);
                if (follower.getError(placeSub2).getY() < 1.5) {
                    follower.driveSlam(false);

                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    follower.followPath(toWall2);
                    setPathState(Specimen6AutoEnum.wallPreset2);
                }
                break;

            case wallPreset2:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    outtakeSystem.placePos(PlacePosEnum.wallAuto);
                    setPathState(Specimen6AutoEnum.grabWall2);
                }
                break;
            //part 5 placing specimen 3 ***********************************************************~
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
                if (follower.getErrorDistance(placeSub2.addReturn(frontSubPlaceOffset)) < 1.5) {
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
                    follower.followPath(toWall3);
                    setPathState(Specimen6AutoEnum.wallPreset3);
                }
                break;
            case wallPreset3:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    outtakeSystem.placePos(PlacePosEnum.wallAuto);
                    setPathState(Specimen6AutoEnum.grabWall3);
                }
                break;
            //part 6 placing specimen 4 ***********************************************************~
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
                if (follower.getErrorDistance(placeSub3.addReturn(frontSubPlaceOffset)) < 1.5) {
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

                    follower.followPath(toWall4);
                    setPathState(Specimen6AutoEnum.wallPreset4);

                }
                break;
            case wallPreset4:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    outtakeSystem.placePos(PlacePosEnum.wallAuto);
                    setPathState(Specimen6AutoEnum.grabWall4);
                }
                break;
            //part 7 placing specimen 5 ***********************************************************~
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
                if (follower.getErrorDistance(placeSub4.addReturn(frontSubPlaceOffset)) < 1.5) {
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
                    follower.followPath(toWall5);
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
                    setPathState(Specimen6AutoEnum.grabWall5);
                }
                break;
            //part 8 placing specimen 6?!?!?!?!? **************************************************~
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
                if (follower.getErrorDistance(placeSub5.addReturn(frontSubPlaceOffset)) < 1.5) {
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