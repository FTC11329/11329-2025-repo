package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedropathing.follower.*;
import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.BezierCurve;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Path;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Point;
import org.firstinspires.ftc.teamcode.pedropathing.util.Drawing;
import org.firstinspires.ftc.teamcode.pedropathing.util.Timer;

//Match 1 transfer too long, down with servo then out with intake for 3rd          miss 2
//Match 2 Wiring                                                                   miss 4
//match 3 down with servo then out with intake for 3rd, grabed too early x2        miss 2
//match 4 Bad clip                                                                 miss 0
//match 5 down with servo then out with intake for 3rd, grabbed to early x2        miss 2
//Tournament miss one from arm not coming down early                               miss 1



@Disabled
@Autonomous(name = "Spline Test(pid)", group = "Comp")
public class SplineTest extends OpMode {

    Follower follower;

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
    private final Pose preloadPlace = new Pose(9, -32, Math.toRadians(90));

    private final Pose spike1ControlPoint1 = new Pose(47, -64.5, 0);
    private final Pose spike1ControlPoint2 = new Pose(22.5, -8.5, 0);
    private final Pose backSpike1 = new Pose(48, -12, Math.toRadians(90));

    private final Pose pushedSpike1 = new Pose(48.5, -50, Math.toRadians(90));

    private final Pose spike2ControlPoint1 = new Pose(41, -12, 0);
    private final Pose spike2ControlPoint2 = new Pose(64, -12, 0);
    private final Pose pushedSpike2 = new Pose(57, -50, Math.toRadians(90));

    private final Pose spike3ControlPoint1 = new Pose(53, -5, 0);
    private final Pose backSpike3 = new Pose(64, -10.5, Math.toRadians(90));

    private final Pose pushedSpike3 = new Pose(64, -50, Math.toRadians(90));

    private final Pose frontWall  = new Pose(38, -53, Math.toRadians(90));
    private final Pose pickupWall = new Pose(38, -60, Math.toRadians(90));
    private final Pose pickupWallRightSide = new Pose(64, -60, Math.toRadians(90));

    private final double humanWaitTime = 0.25;
    private final double humanWaitTime2 = 1.25;

    private final double pickupTimeout = 1;

    private double loopTime = 0;

    private boolean driveShake = false;

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path pickupSpike1Path;
    private Path pushSpike1Path;

    private Path pickupSpike2Path;

    private Path pickupSpike3Path;
    private Path pushSpike3Path;

    private Path toWall1;
    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        follower = new Follower(hardwareMap);

        pathTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        follower.setStartingPose(preloadPlace);

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
        setPathState(1);
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

        pickupSpike1Path = new Path(new BezierCurve(new Point(preloadPlace), new Point(spike1ControlPoint1), new Point(spike1ControlPoint2), new Point(backSpike1)));
        pickupSpike1Path.setConstantHeadingInterpolation(Math.toRadians(90));

        pushSpike1Path   = follower.linearPathBuilder(backSpike1, pushedSpike1);

        pickupSpike2Path = new Path(new BezierCurve(new Point(pushedSpike1), new Point(spike2ControlPoint1), new Point(spike2ControlPoint2), new Point(pushedSpike2)));
        pickupSpike2Path.setConstantHeadingInterpolation(Math.toRadians(90));

        pickupSpike3Path = new Path(new BezierCurve(new Point(pushedSpike2), new Point(spike3ControlPoint1), new Point(backSpike3)));
        pickupSpike3Path.setConstantHeadingInterpolation(Math.toRadians(90));

        pushSpike3Path   = follower.linearPathBuilder(backSpike3, pushedSpike3);

        toWall1 = follower.linearPathBuilder(pushedSpike3, pickupWallRightSide);
        //frontWallToWall
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            //go to score preload
            case 1:
                //Checks if you are less than 1 inch away from target pos
                follower.followPath(pickupSpike1Path);
                setPathState(pathState+1);
                break;
            //go to pickup wall preset
            case 2:
                if (pathTimer.getElapsedTimeSeconds() > 0.17) {
                    setPathState(pathState+1);
                }
                //go to pickup spike2
            case 3:
                if (follower.getError(backSpike1).getX() < 4 && follower.getError(backSpike1).getY() < 4) {
                    follower.followPath(pushSpike1Path);
                    setPathState(pathState+1);
                }
            case 4:
                if (follower.getError(pushedSpike1).getX() < 2 && follower.getError(pushedSpike1).getY() < 2) {
                    follower.followPath(pickupSpike2Path);
                    setPathState(pathState+1);
                }
                break;
            //go to pickup spike3
            case 5:
                if (follower.getError(pushedSpike2).getX() < 2 && follower.getError(pushedSpike2).getY() < 2) {
                    follower.followPath(pickupSpike3Path);
                    setPathState(pathState+1);
                }
                break;
            //go to front pickup wall
            case 6:
                if (follower.getError(backSpike3).getX() < 4 && follower.getError(backSpike3).getY() < 4) {
                    follower.followPath(pushSpike3Path);
                    setPathState(pathState+1);
                }
                break;
            case 7:
                if (follower.getError(pushedSpike3).getY() < 1) {
                    setPathState(pathState+1);
                }
                break;
            //grab off wall and go to sub
            case 8:
                if (pathTimer.getElapsedTimeSeconds() > humanWaitTime) {
                    follower.followPath(toWall1);
                    setPathState(17);
                }
            case 17:
                if (follower.getError(pickupWallRightSide).getY() < 1 || pathTimer.getElapsedTimeSeconds() > pickupTimeout) {
                    setPathState(pathState+1);
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


