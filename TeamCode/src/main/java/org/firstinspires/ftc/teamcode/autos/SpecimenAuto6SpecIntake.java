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
import org.firstinspires.ftc.teamcode.utility.autoEnums.Specimen6AutoIntakeEnum;

//Match 1 transfer too long, down with servo then out with intake for 3rd          miss 2
//Match 2 Wiring                                                                   miss 4
//match 3 down with servo then out with intake for 3rd, grabed too early x2        miss 2
//match 4 Bad clip                                                                 miss 0
//match 5 down with servo then out with intake for 3rd, grabbed to early x2        miss 2
//Tournament miss one from arm not coming down early                               miss 1


public class SpecimenAuto6SpecIntake {
    Climber climber;
    Follower follower;
    Attempt89 blockVision;
    Drivetrain driveTrain;
    PowerTakeOff powerTakeOff;
    IntakeSystem intakeSystem;
    OuttakeSystem outtakeSystem;

    private Timer pathTimer, actionTimer, opmodeTimer;

    private Specimen6AutoIntakeEnum pathState;

    private final Pose startPose = new Pose(9, -65.3, Math.toRadians(180));


    // Scoring Poses of our robot. */
    private final Pose preloadPlace = new Pose(6.5, -33.5, Math.toRadians(90));
    private final Pose placeSub1 = new Pose(11, -33, Math.toRadians(90));
    private final Pose placeSub2 = new Pose(8.5, -33, Math.toRadians(90));
    private final Pose placeSub3 = new Pose(7, -32.5, Math.toRadians(90));
    private final Pose placeSub4 = new Pose(5.5, -32.5, Math.toRadians(90));
    private final Pose placeSub5 = new Pose(4, -32.5, Math.toRadians(90));

    private final Pose pickupWallFirst = new Pose(37, -60.6, Math.toRadians(90));
    private final Pose pickupWallControlPointFirst = new Pose(40, -58, Math.toRadians(90));


    private final Pose pickupSpike1 = new Pose(35.5, -46, Math.toRadians(55.86));
    private final Pose pickupSpike2 = new Pose(40.87, -41, Math.toRadians(29));
    private final Pose pickupSpike3 = new Pose(49.32, -40.4, Math.toRadians(18.91));

    private final Pose pickupWall = new Pose(37, -60.6, Math.toRadians(90));
    private final Pose pickupWallControlPoint = new Pose(40, -58, Math.toRadians(90));

    private final Pose frontSubPlaceOffset = new Pose(0, -10, Math.toRadians(0));
    private final Pose frontSubToWallOffset = new Pose(0, -3, Math.toRadians(0));

    private final Pose halfWayFirstWallPose = new Pose((preloadPlace.getX() + preloadPlace.addReturn(frontSubToWallOffset).getX()) / 2, (preloadPlace.getY() + preloadPlace.addReturn(frontSubToWallOffset).getY()) / 2, (preloadPlace.getHeading() + preloadPlace.addReturn(frontSubToWallOffset).getHeading()) / 2);

    private final Pose park = new Pose(58, -58, Math.toRadians(90));


    //Various Variables

    Pose2D visionResult = new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.DEGREES,-1);
    private double loopTime = 0;
    private double slamSpeed = 0.8;
    private double visionSlidePos = Constants.Intake.intakeSlidePos;
    private boolean driveShake = false;
    private boolean driveSlam = false;

    private boolean hasOne = false;

    private boolean recoveryDebounce = false;
    private double recoveryTime = 0;
    private boolean recovering = false;
    private boolean recoverOnce = false;
    private Specimen6AutoIntakeEnum recoveryEnum;
    private Path recoveryPath;

    // These are our Paths and PathChains that we will define in buildPaths()
    private Path scorePreload;

    private Path pickupSpike1Path;

    private Path pickupSpike2Path;

    private Path pickupSpike3Path;

    private PathChain toWall1;
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

    public SpecimenAuto6SpecIntake(HardwareMap hardwareMap, Telemetry telemetry, RobotSideEnum robotSide) {
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
        outtakeSystem = new OuttakeSystem(hardwareMap, robotSide, false);


        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        follower.setStartingPose(startPose);
        blockVision.switchPipeline(robotSide);
        buildPaths();
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

        outtakeSystem.initArm();
        outtakeSystem.setArmPos(Constants.Outtake.upArm);

        setPathState(Specimen6AutoIntakeEnum.driveGoScorePreload);
    }

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        double pushingZPAM = 35;
        double firstPlaceZPAM = 69;
        double toPlaceZPAM = 13;
        double toPickup = 6;
        double toWallZPAM = 4.3;

        scorePreload     = follower.linearPathBuilder(startPose, preloadPlace);
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), preloadPlace.getHeading(), 0.25);
        scorePreload.setZeroPowerAccelerationMultiplier(firstPlaceZPAM);

        toWall1 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(preloadPlace.addReturn(frontSubToWallOffset)), new Point(halfWayFirstWallPose))))
                .setConstantHeadingInterpolation(Math.toRadians(135))
                .setZeroPowerAccelerationMultiplier(toWallZPAM)
                .addPath(new BezierCurve(new Point(halfWayFirstWallPose), new Point(pickupWallControlPointFirst), new Point(pickupWallFirst)))
                .setLinearHeadingInterpolation(Math.toRadians(135), pickupWallFirst.getHeading(), 0.6)
                .setZeroPowerAccelerationMultiplier(toWallZPAM)
                .build();
        //frontWallToWall
        placeSub1Path = follower.linearPathBuilder(pickupWallFirst, placeSub1.addReturn(frontSubPlaceOffset));
        placeSub1Path.setConstantHeadingInterpolation(placeSub1.getHeading());
        placeSub1Path.setZeroPowerAccelerationMultiplier(toPlaceZPAM);

        pickupSpike1Path = follower.linearPathBuilder(placeSub1.addReturn(frontSubToWallOffset), pickupSpike1);
        pickupSpike1Path.setLinearHeadingInterpolation(placeSub1.getHeading(), pickupSpike1.getHeading(), 0.8);
        pickupSpike1Path.setZeroPowerAccelerationMultiplier(toPickup);

        toWall2 = follower.linearPathBuilder(pickupSpike1, pickupWall);
        toWall2.setLinearHeadingInterpolation(pickupSpike1.getHeading(), pickupWall.getHeading(), 0.8);
        toWall2.setZeroPowerAccelerationMultiplier(toWallZPAM);
        //frontWallToWall
        placeSub2Path = follower.linearPathBuilder(pickupWall, placeSub2.addReturn(frontSubPlaceOffset));
        placeSub2Path.setConstantHeadingInterpolation(placeSub2.getHeading());
        placeSub2Path.setZeroPowerAccelerationMultiplier(toPlaceZPAM);

        pickupSpike2Path = follower.linearPathBuilder(placeSub2.addReturn(frontSubToWallOffset), pickupSpike2);
        pickupSpike2Path.setLinearHeadingInterpolation(placeSub2.getHeading(), pickupSpike2.getHeading(), 0.8);
        pickupSpike2Path.setZeroPowerAccelerationMultiplier(toPickup);

        toWall3 = follower.linearPathBuilder(pickupSpike2, pickupWall);
        toWall3.setLinearHeadingInterpolation(pickupSpike2.getHeading(), pickupWall.getHeading());
        toWall3.setZeroPowerAccelerationMultiplier(toWallZPAM);
        //frontWallToWall
        placeSub3Path = follower.linearPathBuilder(pickupWall, placeSub3.addReturn(frontSubPlaceOffset));
        placeSub3Path.setConstantHeadingInterpolation(placeSub3.getHeading());
        placeSub3Path.setZeroPowerAccelerationMultiplier(toPlaceZPAM);

        pickupSpike3Path = follower.linearPathBuilder(placeSub3.addReturn(frontSubToWallOffset), pickupSpike3);
        pickupSpike3Path.setLinearHeadingInterpolation(placeSub3.getHeading(), pickupSpike3.getHeading(), 0.8);
        pickupSpike3Path.setZeroPowerAccelerationMultiplier(pushingZPAM);

        toWall4 = follower.linearPathBuilder(pickupSpike3, pickupWall);
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
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState(Specimen6AutoIntakeEnum.) method)
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
                intakeSystem.setHSlidePos(Constants.Intake.autoPreExtendSlides);

                setPathState(Specimen6AutoIntakeEnum.placePreload);
                break;
            //go to pickup spike1
            case placePreload:
                //Checks if you are less than 1 inch away from target pos
                if (follower.getError(preloadPlace).getX() < 1.5 && follower.getError(preloadPlace).getY() < 1.5 || pathTimer.getElapsedTimeSeconds() > 3) {
                    setPathState(Specimen6AutoIntakeEnum.dropClaw0);
                }
                break;
            case dropClaw0:
                if (pathTimer.getElapsedTimeSeconds() > 0.15) {
                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    setPathState(Specimen6AutoIntakeEnum.visionLook);
                }
                break;
            //go to pickup wall preset
            case visionLook:
                if (pathTimer.getElapsedTimeSeconds() > 0.15) {
                    visionResult = blockVision.getBlockPosition();
                    if (visionResult.getHeading(AngleUnit.DEGREES) != -1) {
                        outtakeSystem.placePos(PlacePosEnum.wall);
                        outtakeSystem.setClawPos(Constants.Outtake.dropClaw);

                        visionSlidePos = follower.followYourHead(visionResult);
                        intakeSystem.setHSlidesInches(visionSlidePos);
                        setPathState(Specimen6AutoIntakeEnum.drivingVision);

                    } else if (pathTimer.getElapsedTimeSeconds() > 1) {
                        outtakeSystem.placePos(PlacePosEnum.wall);
                        outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                        intakeSystem.storePos();
                        hasOne = false;
                        follower.followPath(pickupSpike1Path);
                        setPathState(Specimen6AutoIntakeEnum.missFirstIntake);
                    }
                }
                break;
            case drivingVision:
                if ((Math.abs(intakeSystem.getHSlideTargetPos() - intakeSystem.getHSlidePos()) < 100 && follower.getHeadingError() < Math.toRadians(5)) || pathTimer.getElapsedTimeSeconds() > 0.5) {
                    intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
                    driveShake = true;
                    setPathState(Specimen6AutoIntakeEnum.startIntake);
                }
                break;
            case startIntake:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    intakeSystem.intakeUntilColor();
                    setPathState(Specimen6AutoIntakeEnum.intakingWithVision);
                }
            case intakingWithVision:
                intakeSystem.update();
                if (intakeSystem.intakeUntilColor()) {
                    actionTimer.resetTimer();

                    intakeSystem.storePos();
                    intakeSystem.setIntakePower(Constants.Intake.unjamSpeed);
                    driveShake = false;
                    hasOne = true;

                    follower.followPath(toWall1);
                    setPathState(Specimen6AutoIntakeEnum.unjam);

                } else if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                    intakeSystem.storePos();
                    intakeSystem.setIntakePower(Constants.Intake.spitSpeed);
                    driveShake = false;
                    hasOne = false;

                    setPathState(Specimen6AutoIntakeEnum.missFirstIntake);
                }
                break;
            case unjam:
                if (pathTimer.getElapsedTimeSeconds() > Constants.Intake.unjamTimeMillisAuto / 1000) {
                    intakeSystem.setIntakePower(Constants.Intake.intakeSpeed);
                    setPathState(Specimen6AutoIntakeEnum.reIntake);
                }
                break;
            case reIntake:
                if (intakeSystem.intakeUntil() || pathTimer.getElapsedTimeSeconds() > Constants.Intake.unjamTimeMillisAuto / 1000){
                    intakeSystem.setIntakePower(0);
                    setPathState(Specimen6AutoIntakeEnum.transferToTray);
                }
            case transferToTray:
                if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                    intakeSystem.setIntakePower(Constants.Intake.transferSpeed);

                    setPathState(Specimen6AutoIntakeEnum.wallPreset2);
                }
                break;
            case missFirstIntake:
                if (intakeSystem.getHSlideTargetPos() < 100 || pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(toWall1);
                    setPathState(Specimen6AutoIntakeEnum.wallPreset2);
                }

            case wallPreset2:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    outtakeSystem.placePos(PlacePosEnum.wallAuto);
                    setPathState(Specimen6AutoIntakeEnum.depoThePickup);
                }
                break;
            //change to be transfer and dump
            case depoThePickup:
                if (pathTimer.getElapsedTimeSeconds() > 0.6 || pathTimer.getElapsedTimeSeconds() > 1.75) {
                    intakeSystem.setDepoServoPos(Constants.Intake.depoDepo);
                    intakeSystem.setIntakePower(0);

                    setPathState(Specimen6AutoIntakeEnum.grabWall2);
                }
                break;
            case grabWall2:
                if (follower.getErrorDistance(pickupWallFirst) < 2 && outtakeSystem.seesWall() && pathTimer.getElapsedTimeSeconds() > 0.4) {
                    intakeSystem.setDepoServoPos(Constants.Intake.depoStore);
                    outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                    setPathState(Specimen6AutoIntakeEnum.goPlaceSub2);
                }
                break;
            //outtake to specimen place pos
            case goPlaceSub2:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(placeSub2Path);
                    outtakeSystem.setVSlidePos(Constants.Outtake.safeFromWallSlides);
                    setPathState(Specimen6AutoIntakeEnum.specPreset2);
                }
                break;
            case specPreset2:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    outtakeSystem.placePos(PlacePosEnum.highSpecimen);
                    setPathState(Specimen6AutoIntakeEnum.drivePlace2);
                }
                break;
            case drivePlace2:
                if (follower.getErrorDistance(placeSub2.addReturn(frontSubPlaceOffset)) < 1.5) {
                    follower.startTeleopDrive();
                    driveSlam = true;

                    setPathState(Specimen6AutoIntakeEnum.dropClaw2);
                }
                break;
            //go Front Wall 2
            case dropClaw2:
                driveTrain.drive(slamSpeed,0,0, DriveSpeedEnum.Auto);
                if (follower.getError(placeSub2).getY() < 1.5) {
                    // teleop drive = false
                    follower.breakFollowing();
                    driveSlam = false;

                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    outtakeSystem.setArmPos(Constants.Outtake.upArm);
                    outtakeSystem.setVSlidePos(0);
                    follower.followPath(pickupSpike1Path);
                    setPathState(Specimen6AutoIntakeEnum.goSpike1);
                }
                break;

            case goSpike1:
                if (follower.getError(pickupSpike1).getX() < 2 && follower.getError(pickupSpike1).getY() < 2 && follower.getError(pickupSpike1).getHeading() < Math.toRadians(5)) {
                    intakeSystem.setHSlidePos(Constants.Intake.intakeSlidePos);
                    setPathState(Specimen6AutoIntakeEnum.intakeSpike1);
                }
                break;
            case intakeSpike1:
                if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                    intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
                }
                if (intakeSystem.intakeUntil() || pathTimer.getElapsedTimeSeconds() > 1.5) {
                    intakeSystem.storePos();
                    actionTimer.resetTimer();
                    follower.followPath(toWall2);
                    setPathState(Specimen6AutoIntakeEnum.grabWall1);
                }
                break;
            //go to pickup spike3
            case grabWall1:
                if (actionTimer.getElapsedTimeSeconds() > 0.25 && actionTimer.getElapsedTimeSeconds() < 0.4) {
                    intakeSystem.setIntakePower(Constants.Intake.transferSpeed);
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.5) {
                    intakeSystem.setIntakePower(0);
                    intakeSystem.setDepoServoPos(Constants.Intake.depoDepo);
                }
                if ((follower.getError(pickupWall).getY() < 2 || pathTimer.getElapsedTimeSeconds() > 1.5) && outtakeSystem.seesWall() && actionTimer.getElapsedTimeSeconds() > 1) {
                    intakeSystem.setDepoServoPos(Constants.Intake.depoStore);
                    outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                    setPathState(Specimen6AutoIntakeEnum.goPlaceSub1);
                }
                break;
            case goPlaceSub1:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(placeSub2Path);
                    outtakeSystem.setVSlidePos(Constants.Outtake.safeFromWallSlides);
                    setPathState(Specimen6AutoIntakeEnum.specPreset1);
                }
                break;
            //outtake to specimen place pos
            case specPreset1:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    outtakeSystem.placePos(PlacePosEnum.highSpecimen);
                    setPathState(Specimen6AutoIntakeEnum.drivePlace1);
                }
                break;
            case drivePlace1:
                if (follower.getErrorDistance(placeSub2.addReturn(frontSubPlaceOffset)) < 1.5) {
                    follower.startTeleopDrive();
                    driveSlam = true;

                    setPathState(Specimen6AutoIntakeEnum.dropClaw1);
                }
                break;
            //go Front Wall 2
            case dropClaw1:
                if (follower.getError(placeSub2).getY() < 1.5) {
                    // teleop drive = false
                    follower.breakFollowing();
                    driveSlam = false;

                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    follower.followPath(pickupSpike2Path);
                    setPathState(Specimen6AutoIntakeEnum.wallPreset3);
                }
                break;
            case wallPreset3:
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    outtakeSystem.placePos(PlacePosEnum.wallAuto);
                    setPathState(Specimen6AutoIntakeEnum.goSpike2);
                }
                break;
            //part 6 placing specimen 4 ***********************************************************~
            //grab off wall and go to sub
            case goSpike2:
                if (follower.getError(pickupSpike2).getX() < 2 && follower.getError(pickupSpike2).getY() < 2 && follower.getError(pickupSpike2).getHeading() < Math.toRadians(5)) {
                    intakeSystem.setHSlidePos(Constants.Intake.intakeSlidePos);
                    setPathState(Specimen6AutoIntakeEnum.intakeSpike2);
                }
                break;
            //go to front pickup wall
            case intakeSpike2:
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
                }
                if (intakeSystem.intakeUntil() || pathTimer.getElapsedTimeSeconds() > 1) {
                    intakeSystem.storePos();
                    follower.followPath(toWall3);
                    actionTimer.resetTimer();
                    setPathState(Specimen6AutoIntakeEnum.grabWall3);
                }
                break;
            //grab off wall and go to sub
            case grabWall3:
                if (actionTimer.getElapsedTimeSeconds() > 0.25 && actionTimer.getElapsedTimeSeconds() < 0.4) {
                    intakeSystem.setIntakePower(Constants.Intake.transferSpeed);
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.5) {
                    intakeSystem.setIntakePower(0);
                    intakeSystem.setDepoServoPos(Constants.Intake.depoDepo);
                }
                if ((follower.getError(pickupWall).getY() < 2 || pathTimer.getElapsedTimeSeconds() > 1.5) && outtakeSystem.seesWall() && actionTimer.getElapsedTimeSeconds() > 1) {
                    intakeSystem.setDepoServoPos(Constants.Intake.depoStore);
                    outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                    setPathState(Specimen6AutoIntakeEnum.goPlaceSub3);
                }
                break;
            //outtake to specimen place pos
            case goPlaceSub3:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(placeSub3Path);
                    outtakeSystem.setVSlidePos(Constants.Outtake.safeFromWallSlides);
                    setPathState(Specimen6AutoIntakeEnum.specPreset3);
                }
                break;
            case specPreset3:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    outtakeSystem.placePos(PlacePosEnum.highSpecimen);
                    setPathState(Specimen6AutoIntakeEnum.drivePlace3);
                }
                break;
            case drivePlace3:
                if (follower.getErrorDistance(placeSub3.addReturn(frontSubPlaceOffset)) < 1.5) {
                    follower.startTeleopDrive();
                    driveSlam = true;

                    setPathState(Specimen6AutoIntakeEnum.dropClaw3);
                }
                break;
            //go Front Wall 2
            case dropClaw3:
                if (follower.getError(placeSub3).getY() < 1.5) {
                    // teleop drive = false
                    follower.breakFollowing();
                    driveSlam = false;

                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);

                    follower.followPath(pickupSpike3Path);
                    setPathState(Specimen6AutoIntakeEnum.wallPreset4);
                }
                break;
            case wallPreset4:
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    outtakeSystem.placePos(PlacePosEnum.wallAuto);
                    setPathState(Specimen6AutoIntakeEnum.grabWall4);
                }
                break;
            //part 7 placing specimen 5 ***********************************************************~
            //grab off wall and go to sub
            case goSpike3:
                if (follower.getError(pickupSpike3).getX() < 2 && follower.getError(pickupSpike3).getY() < 2 && follower.getError(pickupSpike3).getHeading() < Math.toRadians(5)) {
                    intakeSystem.setHSlidePos(Constants.Intake.intakeSlidePos);
                    setPathState(Specimen6AutoIntakeEnum.intakeSpike2);
                }
            break;
            case intakeSpike3:
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
                }
                if (intakeSystem.intakeUntil() || pathTimer.getElapsedTimeSeconds() > 1) {
                    intakeSystem.storePos();
                    follower.followPath(toWall4);
                    actionTimer.resetTimer();
                    setPathState(Specimen6AutoIntakeEnum.grabWall4);
                }
                break;
            case grabWall4:
                if (actionTimer.getElapsedTimeSeconds() > 0.25 && actionTimer.getElapsedTimeSeconds() < 0.4) {
                    intakeSystem.setIntakePower(Constants.Intake.transferSpeed);
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.5) {
                    intakeSystem.setIntakePower(0);
                    intakeSystem.setDepoServoPos(Constants.Intake.depoDepo);
                }
                if ((follower.getError(pickupWall).getY() < 2 || pathTimer.getElapsedTimeSeconds() > 1.5) && outtakeSystem.seesWall() && actionTimer.getElapsedTimeSeconds() > 1) {
                    intakeSystem.setDepoServoPos(Constants.Intake.depoStore);
                    outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                    setPathState(Specimen6AutoIntakeEnum.goPlaceSub4);
                }
                break;
            //outtake to specimen place pos
            case goPlaceSub4:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(placeSub4Path);
                    outtakeSystem.setVSlidePos(Constants.Outtake.safeFromWallSlides);
                    setPathState(Specimen6AutoIntakeEnum.specPreset4);
                }
                break;
            case specPreset4:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    outtakeSystem.placePos(PlacePosEnum.highSpecimen);
                    setPathState(Specimen6AutoIntakeEnum.drivePlace4);
                }
                break;
            case drivePlace4:
                if (follower.getErrorDistance(placeSub4.addReturn(frontSubPlaceOffset)) < 1.5) {
                    follower.startTeleopDrive();
                    driveSlam = true;

                    setPathState(Specimen6AutoIntakeEnum.dropClaw4);
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
                    if (hasOne && opmodeTimer.getElapsedTimeSeconds() < 29) {
                        setPathState(Specimen6AutoIntakeEnum.wallPreset5);
                    } else {
                        setPathState(Specimen6AutoIntakeEnum.goPark);
                    }
                }
                break;

            case wallPreset5:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    outtakeSystem.placePos(PlacePosEnum.wallAuto);
                    setPathState(Specimen6AutoIntakeEnum.grabWall5);
                }
                break;
            //part 8 placing specimen 6?!?!?!?!? **************************************************~
            //grab off wall and go to sub
            case grabWall5:
                if (follower.getErrorDistance(pickupWall) < 2.5 && outtakeSystem.seesWall()) {
                    outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                    setPathState(Specimen6AutoIntakeEnum.goPlaceSub5);
                }
                break;
            //outtake to specimen place pos
            case goPlaceSub5:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(placeSub5Path);
                    outtakeSystem.setVSlidePos(Constants.Outtake.safeFromWallSlides);
                    setPathState(Specimen6AutoIntakeEnum.specPreset5);
                }
                break;
            case specPreset5:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    outtakeSystem.placePos(PlacePosEnum.highSpecimen);
                    setPathState(Specimen6AutoIntakeEnum.drivePlace5);
                }
                break;
            case drivePlace5:
                if (follower.getErrorDistance(placeSub5.addReturn(frontSubPlaceOffset)) < 1.5) {
                    follower.startTeleopDrive();
                    driveSlam = true;

                    setPathState(Specimen6AutoIntakeEnum.dropClaw5);
                }
                break;
            //go Front Wall 2
            case dropClaw5:
                if (follower.getError(placeSub5).getY() < 1.5) {
                    // teleop drive = false
                    follower.breakFollowing();
                    driveSlam = false;

                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    setPathState(Specimen6AutoIntakeEnum.goPark);
                }
                break;
            //Park path
            case goPark:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(parkPath);
                    setPathState(Specimen6AutoIntakeEnum.teleopPreset);
                }
                break;
            case teleopPreset:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    outtakeSystem.setArmPos(Constants.Outtake.upArm);
                    outtakeSystem.setVSlidePos(0);
                    setPathState(Specimen6AutoIntakeEnum.end);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(Specimen6AutoIntakeEnum pState) {
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
        if (driveShake && pathTimer.getElapsedTimeSeconds() > 0.8) {

            if (Math.round((pathTimer.getElapsedTimeSeconds() - 0.8) * 3) % 2 == 0 ){
                driveTrain.drive(0,0, -0.5, DriveSpeedEnum.Auto);
            } else {
                driveTrain.drive(0,0, 0.5, DriveSpeedEnum.Auto);
            }
            if (Math.round((pathTimer.getElapsedTimeSeconds() - 0.8) * 5) % 2 == 0 ){
                intakeSystem.setHSlidesInches(visionSlidePos + 4);
            } else {
                intakeSystem.setHSlidesInches(visionSlidePos - 1);
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
            setPathState(Specimen6AutoIntakeEnum.end);
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
        if (false) {
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addData("loopTime", loopTime - opmodeTimer.getElapsedTime());
            telemetry.addData("tripped", driveTrain.isStalled(3));
            telemetry.addData("opmode", opmodeTimer.getElapsedTimeSeconds());
//            telemetry.addData("fl", driveTrain.getDrivePowers()[0]);
//            telemetry.addData("bl", driveTrain.getDrivePowers()[1]);
//            telemetry.addData("fr", driveTrain.getDrivePowers()[2]);
//            telemetry.addData("br", driveTrain.getDrivePowers()[3]);
//            telemetry.addData("ma", Math.max(Math.max(driveTrain.getDriveCurrent()[0], driveTrain.getDriveCurrent()[1]), Math.max(driveTrain.getDriveCurrent()[2], driveTrain.getDriveCurrent()[3])));
            loopTime = opmodeTimer.getElapsedTime();
            telemetry.update();
        }
    }

    /** We do not use this because everything should automatically disable **/
    public void stop() {
    }
}