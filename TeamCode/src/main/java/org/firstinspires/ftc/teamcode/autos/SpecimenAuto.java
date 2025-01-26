package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Drawing;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.BlockVision;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.PowerTakeOff;
import org.firstinspires.ftc.teamcode.utility.DriveSpeedEnum;
import org.firstinspires.ftc.teamcode.utility.PlacePosEnum;
import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

@Autonomous(name = "Specimen Auto", group = " Comp", preselectTeleOp = "New Tele-op Blue")
public class SpecimenAuto extends OpMode {

    Climber climber;
    Follower follower;
    Drivetrain driveTrain;
    BlockVision blockVision;
    PowerTakeOff powerTakeOff;
    IntakeSystem intakeSystem;
    OuttakeSystem outtakeSystem;

    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

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
    private final Pose preloadPlace = new Pose(8, -32, Math.toRadians(90));
    private final Pose placeSub1 = new Pose(6, -32, Math.toRadians(90));
    private final Pose placeSub2 = new Pose(4, -32, Math.toRadians(90));
    private final Pose placeSub3 = new Pose(2, -32, Math.toRadians(90));
    private final Pose placeSub4 = new Pose(0, -32, Math.toRadians(90));

    private final Pose pickupSpike1 = new Pose(28.5, -41.5, Math.toRadians(55));
    private final Pose pickupSpike2 = new Pose(44, -47, Math.toRadians(55));
    private final Pose pickupSpike3 = new Pose(54, -49, Math.toRadians(59));

    private final Pose spitSpike1 = new Pose(31, -53, Math.toRadians(-35));
    private final Pose spitSpike2 = new Pose(41, -53, Math.toRadians(-35));
    private final Pose spitSpike3 = new Pose(41, -53, Math.toRadians(-35));

    private final Pose frontWall = new Pose(38, -54, Math.toRadians(90));
    private final Pose pickupWall = new Pose(38, -61.7, Math.toRadians(90));

    private final Pose controlPointPose = new Pose(38, -34, Math.toRadians(59));;
    private final Pose park = new Pose(63.75, -64.3, Math.toRadians(90));

    private final double humanWaitTime = 0.9;
    private final double humanWaitTime2 = 1.25;

    private final double pickupTimeout = 1.5;

    private final double spitTime = 0.5;

    private boolean driveShake = false;

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload;

    private Path pickupSpike1Path;
    private Path spitSpike1Path;

    private Path pickupSpike2Path;
    private Path spitSpike2Path;

    private Path pickupSpike3Path;
    private Path spitSpike3Path;

    private Path frontWallToWall;

    private Path toFrontWall1;
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
        blockVision = new BlockVision(hardwareMap, RobotSideEnum.Auto);
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
        setPathState(0);
    }

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
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
        scorePreload        = follower.linearPathBuilder(startPose, preloadPlace);

        pickupSpike1Path    = follower.linearPathBuilder(preloadPlace, pickupSpike1);
        spitSpike1Path      = follower.linearPathBuilder(pickupSpike1, spitSpike1);

        pickupSpike2Path    = follower.linearPathBuilder(spitSpike1, pickupSpike2);
        spitSpike2Path      = follower.linearPathBuilder(pickupSpike2, spitSpike2);

        pickupSpike3Path    = follower.linearPathBuilder(spitSpike2, pickupSpike3);
        spitSpike3Path      = follower.linearPathBuilder(pickupSpike3, spitSpike3);

        frontWallToWall     = follower.linearPathBuilder(frontWall, pickupWall);


        toFrontWall1 = follower.linearPathBuilder(spitSpike3, frontWall);
        //frontWallToWall
        placeSub1Path = new Path(new BezierCurve(new Point(pickupWall), new Point(controlPointPose), new Point(startPose), new Point(placeSub1)));
        placeSub1Path.setConstantHeadingInterpolation(placeSub1.getHeading());


        toFrontWall2 = new Path(new BezierCurve(new Point(placeSub1), new Point(startPose), new Point(controlPointPose), new Point(frontWall)));
        toFrontWall2.setConstantHeadingInterpolation(placeSub1.getHeading());
        //frontWallToWall
        placeSub2Path = new Path(new BezierCurve(new Point(pickupWall), new Point(controlPointPose), new Point(startPose), new Point(placeSub2)));
        placeSub2Path.setConstantHeadingInterpolation(placeSub2.getHeading());


        toFrontWall3 = new Path(new BezierCurve(new Point(placeSub2), new Point(startPose), new Point(controlPointPose), new Point(frontWall)));
        toFrontWall3.setConstantHeadingInterpolation(placeSub2.getHeading());
        //frontWallToWall
        placeSub3Path = new Path(new BezierCurve(new Point(pickupWall), new Point(controlPointPose), new Point(startPose), new Point(placeSub3)));
        placeSub3Path.setConstantHeadingInterpolation(placeSub3.getHeading());


        toFrontWall4 = new Path(new BezierCurve(new Point(placeSub3), new Point(startPose), new Point(controlPointPose), new Point(frontWall)));
        toFrontWall4.setConstantHeadingInterpolation(placeSub3.getHeading());
        //frontWallToWall
        placeSub4Path = new Path(new BezierCurve(new Point(pickupWall), new Point(controlPointPose), new Point(startPose), new Point(placeSub4)));
        placeSub4Path.setConstantHeadingInterpolation(placeSub4.getHeading());

        parkPath = follower.linearPathBuilder(placeSub4, park);

    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            //go to score preload
            case 0:
                follower.setMaxPower(1);
                follower.followPath(scorePreload);
                outtakeSystem.setVSlidePos(Constants.Outtake.highSpecimenSlides - 50);
                outtakeSystem.setArmPos(Constants.Outtake.specimenArm);
                setPathState(1);
                break;
            //go to pickup spike1
            case 1:
                //Checks if you are less than 1 inch away from target pos
                if (follower.getError(preloadPlace).getX() < 1 && follower.getError(preloadPlace).getY() < 1) {
                    follower.followPath(pickupSpike1Path, false);
                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    setPathState(2);
                }
                break;
            //start intaking spike1
            case 2:
                if (follower.getError(pickupSpike1).getHeading() < Math.toRadians(25)) {
                    driveShake = true;
                    intakeSystem.setHSlidePos(Constants.Intake.intakeSlidePos);
                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTimeSeconds() > 0.15) {
                    outtakeSystem.placePos(PlacePosEnum.wallAuto);
                    intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
                    setPathState(4);
                }
                break;
            //done intaking spike1 move to spiting
            case 4:
                if (intakeSystem.intakeUntil() || pathTimer.getElapsedTimeSeconds() > pickupTimeout) {
                    driveShake = false;
                    follower.followPath(spitSpike1Path);
                    intakeSystem.setIntakePower(0);
                    intakeSystem.setIntakeServoPos(Constants.Intake.wristStore);
                    intakeSystem.setHSlidePos(Constants.Intake.autoHSlides);
                    setPathState(5);
                }
                break;
            //start spiting
            case 5:
                if (follower.getError(spitSpike1).getHeading() < Math.toRadians(7)) {
                    intakeSystem.setIntakePower(Constants.Intake.spitSpeed);
                    setPathState(6);
                }
                break;
            //part 2 *******************************************************************************
            //go to pickup spike2
            case 6:
                if (pathTimer.getElapsedTimeSeconds() > spitTime) {
                    follower.followPath(pickupSpike2Path, false);
                    intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
                    intakeSystem.setIntakePower(0);
                    setPathState(7);
                }
                break;
            //start intaking spike2
            case 7:
                if (follower.getError(pickupSpike2).getHeading() < Math.toRadians(25)) {
                    driveShake = true;
                    intakeSystem.setHSlidePos(Constants.Intake.intakeSlidePos);
                    setPathState(8);
                }
                break;
            //done intaking spike2 move to spiting
            case 8:
                if (intakeSystem.intakeUntil() || pathTimer.getElapsedTimeSeconds() > pickupTimeout) {
                    driveShake = false;
                    follower.followPath(spitSpike2Path);
                    intakeSystem.setIntakePower(0);
                    intakeSystem.setIntakeServoPos(Constants.Intake.wristStore);
                    intakeSystem.setHSlidePos(Constants.Intake.autoHSlides - 200);
                    setPathState(9);
                }
                break;
            //start spiting
            case 9:
                if (follower.getError(spitSpike2).getHeading() < Math.toRadians(7)) {
                    intakeSystem.setIntakePower(Constants.Intake.spitSpeed);
                    setPathState(10);
                }
                break;
            //part 3 *******************************************************************************
            //go to pickup spike3
            case 10:
                if (pathTimer.getElapsedTimeSeconds() > spitTime) {
                    follower.followPath(pickupSpike3Path);
                    intakeSystem.setIntakePower(0);
                    setPathState(11);
                }
                break;
            //start intaking spike3
            case 11:
                if (follower.getError(pickupSpike3).getHeading() < Math.toRadians(35)) {
                    intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
                    intakeSystem.setHSlidePos(Constants.Intake.intakeSlidePos);
                    setPathState(12);
                }
                break;
            //done intaking spike3 move to spiting
            case 12:
                if (intakeSystem.intakeUntil() || pathTimer.getElapsedTimeSeconds() > pickupTimeout) {
                    follower.followPath(spitSpike3Path);
                    intakeSystem.setIntakePower(0);
                    intakeSystem.setIntakeServoPos(Constants.Intake.wristStore);
                    intakeSystem.setHSlidePos(Constants.Intake.autoHSlides);
                    setPathState(13);
                }
                break;
            //start spiting
            case 13:
                if (follower.getError(spitSpike3).getHeading() < Math.toRadians(7)) {
                    intakeSystem.setIntakePower(Constants.Intake.spitSpeed);
                    setPathState(14);
                }
                break;
            //part 4 placing specimen 2 ************************************************************
            //go to pickup off wall
            case 14:
                if (pathTimer.getElapsedTimeSeconds() > spitTime) {
                    follower.followPath(toFrontWall1);
                    intakeSystem.storePos();
                    setPathState(15);
                }
                break;
            //front Wall To Wall
            case 15:
                if (pathTimer.getElapsedTimeSeconds() > humanWaitTime) {
                    follower.followPath(frontWallToWall);
                    setPathState(16);
                }
                break;
            //grab off wall and go to sub
            case 16:
                if (follower.getError(pickupWall).getX() < 1 && follower.getError(pickupWall).getY() < 1) {
                    outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                    setPathState(17);
                }
                break;
            //outtake to specimen place pos
            case 17:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(placeSub1Path);
                    outtakeSystem.placePos(PlacePosEnum.highSpecimen);
                    setPathState(18);
                }
                break;
            //drop specimen
            case 18:
                if (follower.getError(placeSub1).getY() < 1) {
                    follower.followPath(toFrontWall2);
                    setPathState(19);
                }
                break;
            //back to front wall
            case 19:
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    outtakeSystem.placePos(PlacePosEnum.wallAuto);
                    setPathState(20);
                }
                break;
            //part 5 placing specimen 3 ************************************************************
            //front Wall To Wall
            case 20:
                if (pathTimer.getElapsedTimeSeconds() > humanWaitTime2) {
                    follower.followPath(frontWallToWall);
                    setPathState(21);
                }
                break;
            //grab off wall and go to sub
            case 21:
                if (follower.getError(pickupWall).getX() < 1 && follower.getError(pickupWall).getY() < 1) {
                    outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                    setPathState(22);
                }
                break;
            //outtake to specimen place pos
            case 22:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(placeSub2Path);
                    outtakeSystem.placePos(PlacePosEnum.highSpecimen);
                    setPathState(23);
                }
                break;
            //drop specimen
            case 23:
                if (follower.getError(placeSub1).getY() < 1) {
                    follower.followPath(toFrontWall3);
                    setPathState(24);
                }
                break;
            //back to front wall
            case 24:
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    outtakeSystem.placePos(PlacePosEnum.wallAuto);
                    setPathState(25);
                }
                break;
            //part 6 placing specimen 4 ************************************************************
            //front Wall To Wall
            case 25:
                if (pathTimer.getElapsedTimeSeconds() > humanWaitTime2) {
                    follower.followPath(frontWallToWall);
                    setPathState(26);
                }
                break;
            //grab off wall and go to sub
            case 26:
                if (follower.getError(pickupWall).getX() < 1 && follower.getError(pickupWall).getY() < 1) {
                    outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                    setPathState(27);
                }
                break;
            //outtake to specimen place pos
            case 27:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(placeSub3Path);
                    outtakeSystem.placePos(PlacePosEnum.highSpecimen);
                    setPathState(28);
                }
                break;
            //drop specimen
            case 28:
                if (follower.getError(placeSub1).getY() < 1) {
                    follower.followPath(toFrontWall4);
                    setPathState(29);
                }
                break;
            //back to front wall
            case 29:
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    outtakeSystem.placePos(PlacePosEnum.wallAuto);
                    setPathState(30);
                }
                break;
            //part 7 placing specimen 5 ************************************************************
            //front Wall To Wall
            case 30:
                if (pathTimer.getElapsedTimeSeconds() > humanWaitTime2) {
                    follower.followPath(frontWallToWall);
                    setPathState(31);
                }
                break;
            //grab off wall and go to sub
            case 31:
                if (follower.getError(pickupWall).getX() < 1 && follower.getError(pickupWall).getY() < 1) {
                    outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                    setPathState(32);
                }
                break;
            //outtake to specimen place pos
            case 32:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(placeSub4Path);
                    outtakeSystem.placePos(PlacePosEnum.highSpecimen);
                    setPathState(33);
                }
                break;
            //drop specimen
            case 33:
                if (follower.getError(placeSub1).getY() < 1) {
                    follower.followPath(parkPath);
                    setPathState(34);
                }
                break;
            //back to front wall
            case 34:
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    setPathState(35);
                }
                break;
            case 35:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    outtakeSystem.setArmPos(Constants.Outtake.initTeleopArm);
                    outtakeSystem.setVSlidePos(0);
                    setPathState(36);
                }
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();
        if (driveShake && pathTimer.getElapsedTimeSeconds() > 0.9) {
            if (Math.round((pathTimer.getElapsedTimeSeconds() - 0.9) * 2.5) % 2 == 0 ){
                driveTrain.drive(0,0, -0.5, DriveSpeedEnum.Auto);
            } else {
                driveTrain.drive(0,0, 0.5, DriveSpeedEnum.Auto);
            }
        }

        // Feedback to FTC Dashboard
        Drawing.drawDebug(follower);

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}


