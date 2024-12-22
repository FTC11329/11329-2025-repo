package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Drawing;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
//import org.firstinspires.ftc.teamcode.pedroPathing.config.subsystem.ClawSubsystem;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "PedroAuto", group = " Examples")
public class PedroAuto extends OpMode {

    private Follower follower;
    private Drivetrain drivetrain;

    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /** This is our claw subsystem.
     * We call its methods to manipulate the servos that it has within the subsystem. */
//    public ClawSubsystem claw;

    /** Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(8.5, -63, Math.toRadians(90));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose preloadPlace = new Pose(7, -36, Math.toRadians(90));
    private final Pose placeSub1 = new Pose(4, -36, Math.toRadians(90));
    private final Pose placeSub2 = new Pose(1, -36, Math.toRadians(90));
    private final Pose placeSub3 = new Pose(-2,-36, Math.toRadians(90));
    private final Pose placeSub4 = new Pose(-5,-36, Math.toRadians(90));

    private final Pose pickupSpike1 = new Pose(31, -47, Math.toRadians(53));
    private final Pose pickupSpike2 = new Pose(41, -47, Math.toRadians(53));
    private final Pose pickupSpike3 = new Pose(51, -47, Math.toRadians(53));

    private final Pose spitSpike1 = new Pose(32, -47, Math.toRadians(-26));
    private final Pose spitSpike2 = new Pose(43, -47, Math.toRadians(-26));
    private final Pose spitSpike3 = new Pose(54, -47, Math.toRadians(-26));

    private final Pose frontWall = new Pose(35, -57, Math.toRadians(90));
    private final Pose pickupWall = new Pose(35, -62, Math.toRadians(90));


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



    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        follower = new Follower(hardwareMap);
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        follower.setStartingPose(startPose);

        buildPaths();

//        claw = new ClawSubsystem(hardwareMap);

//        // Set the claw to positions for init
//        claw.closeClaw();
//        claw.startClaw();
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
        placeSub1Path = new Path(new BezierCurve(new Point(pickupWall), new Point(startPose), new Point(placeSub1)));
        placeSub1Path.setConstantHeadingInterpolation(placeSub1.getHeading());

        toFrontWall2 = new Path(new BezierCurve(new Point(placeSub1), new Point(startPose), new Point(pickupSpike2), new Point(frontWall)));
        toFrontWall2.setConstantHeadingInterpolation(placeSub1.getHeading());
        //frontWallToWall
        placeSub2Path = new Path(new BezierCurve(new Point(pickupWall), new Point(startPose), new Point(placeSub2)));
        placeSub2Path.setConstantHeadingInterpolation(placeSub2.getHeading());


        toFrontWall3 = new Path(new BezierCurve(new Point(placeSub2), new Point(startPose), new Point(pickupSpike2), new Point(frontWall)));
        toFrontWall3.setConstantHeadingInterpolation(placeSub2.getHeading());
        //frontWallToWall
        placeSub3Path = new Path(new BezierCurve(new Point(pickupWall), new Point(startPose), new Point(placeSub3)));
        placeSub3Path.setConstantHeadingInterpolation(placeSub3.getHeading());



        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            //go to score preload
            case 0:
                follower.followPath(scorePreload);
                // go to place pos with outake
                setPathState(1);
                break;
            //go to pickup spike1
            case 1:
                //Checks if you are less than 1 inch away from target pos
                if (follower.getError(preloadPlace).getX() < 1 && follower.getError(preloadPlace).getY() < 1) {
                    follower.followPath(pickupSpike1Path);
                    //go to pickup wall with outake
                    setPathState(2);
                }
                break;
            //start intaking spike1
            case 2:
                if (follower.getError(pickupSpike1).getHeading() < Math.toRadians(15)) {
                    //set HExtend to out and start intaking
                    setPathState(3);
                }
                break;
            //done intaking spike1 move to spiting
            case 3:
                //change to when intake has a piece or timeout time
                if (follower.getError(pickupSpike1).getY() < 1 && follower.getError(pickupSpike1).getX() < 1) {
                    follower.followPath(spitSpike1Path);
                    //retract HExtend a little
                    setPathState(4);
                }
                break;
            //start spiting
            case 4:
                if (follower.getError(spitSpike1).getHeading() < Math.toRadians(7)) {
                    //spit
                    setPathState(5);
                }
                break;
            //part 2 *******************************************************************************
            //go to pickup spike2
            case 5:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(pickupSpike2Path);
                    //stop Spit
                    setPathState(6);
                }
                break;
            //start intaking spike2
            case 6:
                if (follower.getError(pickupSpike2).getHeading() < Math.toRadians(15)) {
                    //set HExtend to out and start intaking
                    setPathState(7);
                }
                break;
            //done intaking spike2 move to spiting
            case 7:
                //change to when intake has a piece or timeout time
                if (follower.getError(pickupSpike2).getY() < 1 && follower.getError(pickupSpike2).getX() < 1) {
                    follower.followPath(spitSpike2Path);
                    //retract HExtend a little
                    setPathState(8);
                }
                break;
            //start spiting
            case 8:
                if (follower.getError(spitSpike2).getHeading() < Math.toRadians(7)) {
                    //spit
                    setPathState(9);
                }
                break;
            //part 3 *******************************************************************************
            //go to pickup spike2
            case 9:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(pickupSpike3Path);
                    //stop Spit
                    setPathState(10);
                }
                break;
            //start intaking spike2
            case 10:
                if (follower.getError(pickupSpike3).getHeading() < Math.toRadians(15)) {
                    //set HExtend to out and start intaking
                    setPathState(11);
                }
                break;
            //done intaking spike2 move to spiting
            case 11:
                //change to when intake has a piece or timeout time
                if (follower.getError(pickupSpike3).getY() < 1 && follower.getError(pickupSpike3).getX() < 1) {
                    follower.followPath(spitSpike3Path);
                    //retract HExtend a little
                    setPathState(12);
                }
                break;
            //start spiting
            case 12:
                if (follower.getError(spitSpike3).getHeading() < Math.toRadians(7)) {
                    //spit
                    setPathState(13);
                }
                break;
            //part 4 *******************************************************************************
            //go to pickup off wall
            case 13:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(toFrontWall1, true);
                    //stop Spit
                    setPathState(14);
                }
                break;
            //front Wall To Wall
            case 14:
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(frontWallToWall);
                    setPathState(15);
                }
                break;
            //grab off wall and go to sub
            case 15:
                if (follower.getError(pickupWall).getX() < 1 && follower.getError(pickupWall).getY() < 1) {
                    follower.followPath(placeSub1Path);
                    //grab off wall
                    setPathState(16);
                }
                break;
            //drop sample
            case 16:
                if (follower.getError(placeSub1).getX() < 1 && follower.getError(placeSub1).getY() < 1) {
                    //drop claw
                    setPathState(17);
                }
                break;
            //back to front wall
            case 17:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(toFrontWall2);
                    //go to intake wall preset
                    setPathState(18);
                }
                break;
            //part 5 *******************************************************************************
            //front Wall To Wall
            case 18:
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(frontWallToWall);
                    setPathState(19);
                }
                break;
            //grab off wall and go to sub
            case 19:
                if (follower.getError(pickupWall).getX() < 1 && follower.getError(pickupWall).getY() < 1) {
                    follower.followPath(placeSub2Path);
                    //grab off wall
                    setPathState(20);
                }
                break;
            //drop sample
            case 20:
                if (follower.getError(placeSub2).getX() < 1 && follower.getError(placeSub2).getY() < 1) {
                    //drop claw
                    setPathState(21);
                }
                break;
            //back to front wall
            case 21:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(toFrontWall2);
                    //go to intake wall preset
                    setPathState(22);
                }
                break;

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

        // Feedback to FTC Dashboard
        Drawing.drawDebug(follower);

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        Pose target = new Pose();
        target.setX(follower.getCurrentPath().getPoint(1).getX());
        target.setY(follower.getCurrentPath().getPoint(1).getY());
        target.setHeading(0);
        telemetry.addData("error", follower.getError(target));
        telemetry.addData("drive speed", Math.max(Math.max(drivetrain.getDrivePowers()[0], drivetrain.getDrivePowers()[1]), Math.max(drivetrain.getDrivePowers()[2], drivetrain.getDrivePowers()[3])));
        telemetry.addData("error", follower.getError(pickupSpike1).getHeading());
        telemetry.addData("errorT", Math.toRadians(15));
        telemetry.addData("errorB", follower.getError(pickupSpike1).getHeading() < Math.toRadians(15));
        telemetry.update();
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}


