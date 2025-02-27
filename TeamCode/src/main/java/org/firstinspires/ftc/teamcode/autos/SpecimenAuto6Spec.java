package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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
import org.firstinspires.ftc.teamcode.utility.Specimen6AutoEnum;

//Match 1 transfer too long, down with servo then out with intake for 3rd          miss 2
//Match 2 Wiring                                                                   miss 4
//match 3 down with servo then out with intake for 3rd, grabed too early x2        miss 2
//match 4 Bad clip                                                                 miss 0
//match 5 down with servo then out with intake for 3rd, grabbed to early x2        miss 2
//Tournament miss one from arm not coming down early                               miss 1


@Autonomous(name = "6 Specimen Auto", group = " Comp", preselectTeleOp = "New Tele-op Red")
public class SpecimenAuto6Spec extends OpMode {

    Climber climber;
    Follower follower;
    Attempt89 blockVision;
    Drivetrain driveTrain;
    PowerTakeOff powerTakeOff;
    IntakeSystem intakeSystem;
    OuttakeSystem outtakeSystem;

    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private Specimen6AutoEnum pathState;

    /** Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left. But we don't want do so we don't, 0,0 is center off the field
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     **/

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(8.5, -63, Math.toRadians(90));

    /** Scoring Poses of our robot. */
    private final Pose preloadPlace = new Pose(9, -32.5, Math.toRadians(90));
    private final Pose placeSub1 = new Pose(10, -32.5, Math.toRadians(90));
    private final Pose placeSub2 = new Pose(4, -32.5, Math.toRadians(90));
    private final Pose placeSub3 = new Pose(2, -32.5, Math.toRadians(90));
    private final Pose placeSub4 = new Pose(0, -32.5, Math.toRadians(90));

    private final Pose spike1ControlPoint1 = new Pose(55, -62, 0);
    private final Pose spike1ControlPoint2 = new Pose(22.5, -8.5, 0);
    private final Pose backSpike1 = new Pose(48, -12, Math.toRadians(90));

    private final Pose pushedSpike1 = new Pose(48.5, -50, Math.toRadians(90));

    private final Pose spike2ControlPoint1 = new Pose(39, -6, 0);
    private final Pose spike2ControlPoint2 = new Pose(66, -6, 0);
    private final Pose pushedSpike2 = new Pose(57, -50, Math.toRadians(90));

    private final Pose spike3ControlPoint1 = new Pose(53, -5, 0);
    private final Pose backSpike3 = new Pose(64.5, -15, Math.toRadians(90));

    private final Pose pushedSpike3 = new Pose(64.5, -50, Math.toRadians(90));

    private final Pose frontWall  = new Pose(38.5, -50, Math.toRadians(90));
    private final Pose pickupWall = new Pose(38.5, -60, Math.toRadians(90));
    private final Pose pickupWallRightSide = new Pose(64, -61, Math.toRadians(90));
    private final Pose frontSubOffset = new Pose(10, 0, Math.toRadians(0));

    private final Pose park = new Pose(58, -58, Math.toRadians(90));

    //Various Variables
    private double loopTime = 0;
    private double slamSpeed = 0.8;
    private boolean driveShake = false;
    private boolean drivePlace = false;
    private Vector currentVelocity;

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

    private Path parkPath; //IS THAT A BTD REFERENCE?


    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        climber = new Climber(hardwareMap);
        follower = new Follower(hardwareMap);
        driveTrain = new Drivetrain(hardwareMap);
        powerTakeOff = new PowerTakeOff(hardwareMap);
        intakeSystem = new IntakeSystem(hardwareMap, RobotSideEnum.Auto);
        outtakeSystem = new OuttakeSystem(hardwareMap);

        outtakeSystem.setArmPos(Constants.Outtake.initAutoArm);

        pathTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        follower.setStartingPose(startPose);

        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(Specimen6AutoEnum.armClearing);
    }

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        scorePreload     = follower.linearPathBuilder(startPose, preloadPlace);

        pickupSpike1Path = new Path(new BezierCurve(new Point(preloadPlace), new Point(spike1ControlPoint1), new Point(spike1ControlPoint2), new Point(backSpike1)));
        pickupSpike1Path.setConstantHeadingInterpolation(Math.toRadians(90));

        pushSpike1Path   = follower.linearPathBuilder(backSpike1, pushedSpike1);

        pickupSpike2Path = new Path(new BezierCurve(new Point(pushedSpike1), new Point(spike2ControlPoint1), new Point(spike2ControlPoint2), new Point(pushedSpike2)));
        pickupSpike2Path.setConstantHeadingInterpolation(Math.toRadians(90));

        pickupSpike3Path = new Path(new BezierCurve(new Point(pushedSpike2), new Point(spike3ControlPoint1), new Point(backSpike3)));
        pickupSpike3Path.setConstantHeadingInterpolation(Math.toRadians(90));

        pushSpike3Path   = follower.linearPathBuilder(backSpike3, pushedSpike3);


        frontWallToWall  = follower.linearPathBuilder(frontWall, pickupWall);


        toWall1 = follower.linearPathBuilder(pushedSpike3, pickupWallRightSide);
        //frontWallToWall
        placeSub1Path = new Path(new BezierCurve(new Point(pickupWallRightSide), new Point(placeSub1.addReturn(frontSubOffset))));
        placeSub1Path.setConstantHeadingInterpolation(placeSub1.getHeading());


        toFrontWall2 = new Path(new BezierCurve(new Point(placeSub1), new Point(frontWall)));
        toFrontWall2.setConstantHeadingInterpolation(placeSub1.getHeading());
        //frontWallToWall
        placeSub2Path = new Path(new BezierCurve(new Point(pickupWall), new Point(placeSub2.addReturn(frontSubOffset))));
        placeSub2Path.setConstantHeadingInterpolation(placeSub2.getHeading());


        toFrontWall3 = new Path(new BezierCurve(new Point(placeSub2), new Point(frontWall)));
        toFrontWall3.setConstantHeadingInterpolation(placeSub2.getHeading());
        //frontWallToWall
        placeSub2Path = new Path(new BezierCurve(new Point(pickupWall), new Point(placeSub3.addReturn(frontSubOffset))));
        placeSub3Path.setConstantHeadingInterpolation(placeSub3.getHeading());


        toFrontWall4 = new Path(new BezierCurve(new Point(placeSub3), new Point(frontWall)));
        toFrontWall4.setConstantHeadingInterpolation(placeSub3.getHeading());
        //frontWallToWall
        placeSub2Path = new Path(new BezierCurve(new Point(pickupWall), new Point(placeSub4.addReturn(frontSubOffset))));
        placeSub4Path.setConstantHeadingInterpolation(placeSub4.getHeading());

        parkPath = follower.linearPathBuilder(placeSub4, park);

    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState(Specimen6AutoEnum.) method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            //go to score preload
            case armClearing:
                outtakeSystem.setArmPos(Constants.Outtake.intakeArm);
                outtakeSystem.setVSlidePos(Constants.Outtake.safeFromClimberBar);
                setPathState(Specimen6AutoEnum.driveGoScorePreload);
                break;
            case driveGoScorePreload:
                if (pathTimer.getElapsedTimeSeconds() > 0.6) {
                    follower.followPath(scorePreload);
                    setPathState(Specimen6AutoEnum.armGoScorePreload);
                }
            case armGoScorePreload:
                if (Math.abs(outtakeSystem.getVSlidePos() - outtakeSystem.getVSlideTargetPos()) < 100) {
                    outtakeSystem.setArmPos(Constants.Outtake.specimenArm);
                    setPathState(Specimen6AutoEnum.slideGoScorePreload);
                }
                break;
            case slideGoScorePreload:
                if (pathTimer.getElapsedTimeSeconds() > 0.05) {
                    outtakeSystem.setVSlidePos(Constants.Outtake.highSpecimenSlides - 50);
                    setPathState(Specimen6AutoEnum.placePreload);
                }
                break;
            //go to pickup spike1
            case placePreload:
                //Checks if you are less than 1 inch away from target pos
                if (follower.getError(preloadPlace).getX() < 1 && follower.getError(preloadPlace).getY() < 1) {
                    follower.followPath(pickupSpike1Path);
                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    setPathState(Specimen6AutoEnum.droppedClaw0);
                }
                break;
            //go to pickup wall preset
            case droppedClaw0:
                if (pathTimer.getElapsedTimeSeconds() > 0.17) {
                    outtakeSystem.setArmPos(Constants.Outtake.upArm);
                    outtakeSystem.setVSlidePos(0);
                    setPathState(Specimen6AutoEnum.pushSpike1);
                }
            //go to pickup spike2
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
                if (currentVelocity.getXComponent() < 0.5 && currentVelocity.getYComponent() < 0.5 && pathTimer.getElapsedTimeSeconds() > 0.25) {
                    setPathState(Specimen6AutoEnum.pickupWall1);
                }
                break;
            //grab off wall and go to sub
            case pickupWall1:
//                if (follower.getError(backSpike3).getY() < 1) { todo try this
                if (currentVelocity.getXComponent() < 0.25 && currentVelocity.getYComponent() < 0.25 && pathTimer.getElapsedTimeSeconds() > 0.25) {
                    follower.followPath(toWall1);
                    setPathState(Specimen6AutoEnum.grabWall1);
                }
                break;
            case grabWall1:
//                if (follower.getError(pickupWallRightSide).getY() < 1.5) { todo try this then copy to all instances
                if (currentVelocity.getYComponent() < 0.1) {
                    outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                    setPathState(Specimen6AutoEnum.goPlaceSub1);
                }
                break;
            //outtake to specimen place pos
            case goPlaceSub1:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(placeSub1Path);
                    outtakeSystem.placePos(PlacePosEnum.highSpecimen);
                    setPathState(Specimen6AutoEnum.drivePlace1);
                }
                break;
            case drivePlace1:
                //todo if needed change to velocity
                if (follower.getError(placeSub1).getX() < 1 && follower.getError(placeSub1).getY() < 1) {
                    follower.breakFollowing();
                    follower.driveSlam(true);

                    driveTrain.drive(slamSpeed,0,0, DriveSpeedEnum.Auto);
                    setPathState(Specimen6AutoEnum.dropClaw2);
                }
            //go Front Wall 2
            case dropClaw1:
                driveTrain.drive(slamSpeed,0,0, DriveSpeedEnum.Auto);
                if (currentVelocity.getYComponent() < 0.1) {
                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    follower.followPath(toFrontWall2);
                    setPathState(Specimen6AutoEnum.wallPreset2);
                }
                break;
            //drop specimen
            case wallPreset2:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    outtakeSystem.placePos(PlacePosEnum.wallAuto);
                    setPathState(Specimen6AutoEnum.frontWallToWall2);
                }
                break;
            //part 5 placing specimen 3 ***********************************************************~
            //front Wall To Wall
            case frontWallToWall2:
                if (currentVelocity.getYComponent() < 0.1) {
                    follower.followPath(frontWallToWall);
                    setPathState(Specimen6AutoEnum.grabWall2);
                }
                break;
            //grab off wall and go to sub
            case grabWall2:
                if (currentVelocity.getYComponent() < 0.1) {
                    outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                    setPathState(Specimen6AutoEnum.goPlaceSub2);
                }
                break;
            //outtake to specimen place pos
            case goPlaceSub2:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(placeSub1Path);
                    outtakeSystem.placePos(PlacePosEnum.highSpecimen);
                    setPathState(Specimen6AutoEnum.drivePlace2);
                }
                break;
            case drivePlace2:
                //todo if needed change to velocity
                if (follower.getError(placeSub1).getX() < 1 && follower.getError(placeSub1).getY() < 1) {
                    follower.breakFollowing();
                    follower.driveSlam(true);

                    driveTrain.drive(slamSpeed,0,0, DriveSpeedEnum.Auto);
                    setPathState(Specimen6AutoEnum.dropClaw2);
                }
                //go Front Wall 2
            case dropClaw2:
                driveTrain.drive(slamSpeed,0,0, DriveSpeedEnum.Auto);
                if (currentVelocity.getYComponent() < 0.1) {
                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    follower.followPath(toFrontWall2);
                    setPathState(Specimen6AutoEnum.wallPreset2);
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
                if (currentVelocity.getYComponent() < 0.1) {
                    follower.followPath(frontWallToWall);
                    setPathState(Specimen6AutoEnum.grabWall3);
                }
                break;
            //grab off wall and go to sub
            case grabWall3:
                if (currentVelocity.getYComponent() < 0.1) {
                    outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                    setPathState(Specimen6AutoEnum.goPlaceSub3);
                }
                break;
            //outtake to specimen place pos
            case goPlaceSub3:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(placeSub1Path);
                    outtakeSystem.placePos(PlacePosEnum.highSpecimen);
                    setPathState(Specimen6AutoEnum.drivePlace3);
                }
                break;
            case drivePlace3:
                //todo if needed change to velocity
                if (follower.getError(placeSub1).getX() < 1 && follower.getError(placeSub1).getY() < 1) {
                    follower.breakFollowing();
                    follower.driveSlam(true);

                    driveTrain.drive(slamSpeed,0,0, DriveSpeedEnum.Auto);
                    setPathState(Specimen6AutoEnum.dropClaw3);
                }
                //go Front Wall 2
            case dropClaw3:
                driveTrain.drive(slamSpeed,0,0, DriveSpeedEnum.Auto);
                if (currentVelocity.getYComponent() < 0.1) {
                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    follower.followPath(toFrontWall2);
                    setPathState(Specimen6AutoEnum.wallPreset2);
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
                if (currentVelocity.getYComponent() < 0.1) {
                    follower.followPath(frontWallToWall);
                    setPathState(Specimen6AutoEnum.grabWall4);
                }
                break;
            //grab off wall and go to sub
            case grabWall4:
                if (currentVelocity.getYComponent() < 0.1) {
                    outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                    setPathState(Specimen6AutoEnum.goPlaceSub4);
                }
                break;
            //outtake to specimen place pos
            case goPlaceSub4:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(placeSub1Path);
                    outtakeSystem.placePos(PlacePosEnum.highSpecimen);
                    setPathState(Specimen6AutoEnum.drivePlace4);
                }
                break;
            case drivePlace4:
                //todo if needed change to velocity
                if (follower.getError(placeSub1).getX() < 1 && follower.getError(placeSub1).getY() < 1) {
                    follower.breakFollowing();
                    follower.driveSlam(true);

                    driveTrain.drive(slamSpeed,0,0, DriveSpeedEnum.Auto);
                    setPathState(Specimen6AutoEnum.dropClaw5);
                }
                //go Front Wall 2
            case dropClaw5:
                driveTrain.drive(slamSpeed,0,0, DriveSpeedEnum.Auto);
                if (currentVelocity.getYComponent() < 0.1) {
                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    follower.followPath(toFrontWall2);
                    setPathState(Specimen6AutoEnum.wallPreset2);
                }
                break;
            //Park path
            case goPark:
                if (follower.getError(placeSub1).getY() < 1) {
                    follower.followPath(parkPath);
                    setPathState(Specimen6AutoEnum.dropClaw4);
                }
                break;
            case teleopPreset:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    outtakeSystem.setArmPos(Constants.Outtake.upArm);
                    outtakeSystem.setVSlidePos(0);
                    setPathState(Specimen6AutoEnum.end);
                }
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(Specimen6AutoEnum pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        // These loop the movements of the robot
        follower.update();

        currentVelocity = follower.getVelocity();

        autonomousPathUpdate();
        if (driveShake && pathTimer.getElapsedTimeSeconds() > 1.2) {
            if (Math.round((pathTimer.getElapsedTimeSeconds() - 1.2) * 2.5) % 2 == 0 ){
                driveTrain.drive(0,0, -0.5, DriveSpeedEnum.Auto);
            } else {
                driveTrain.drive(0,0, 0.5, DriveSpeedEnum.Auto);
            }
        }
        //Dumb path recovery
        if (currentVelocity.getXComponent() < 0.1 && currentVelocity.getYComponent() < 0.1 && pathTimer.getElapsedTimeSeconds() > 3 && !recoveryDebounce) {
            recoveryTime = opmodeTimer.getElapsedTimeSeconds();
            recoveryDebounce = true;
            recoverOnce = false;
        } else if (currentVelocity.getXComponent() > 0.1 && currentVelocity.getYComponent() > 0.1) {
            recoveryDebounce = false;
        }

        if (currentVelocity.getXComponent() < 0.1 && currentVelocity.getYComponent() < 0.1 && opmodeTimer.getElapsedTimeSeconds() > recoveryTime + 3) {
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
            if (currentVelocity.getXComponent() < 0.1 && currentVelocity.getYComponent() < 0.1 && pathTimer.getElapsedTimeSeconds() > 1) {
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
            loopTime = opmodeTimer.getElapsedTime();
            telemetry.update();
        }
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}


