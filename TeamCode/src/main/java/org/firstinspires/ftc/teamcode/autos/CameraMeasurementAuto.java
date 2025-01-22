package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.localizers.OTOSLocalizer;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Drawing;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.BlockVision;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.MultiDistanceCalculator;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.PowerTakeOff;
import org.firstinspires.ftc.teamcode.utility.DriveSpeedEnum;
import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

@Autonomous(name = "Camera Auto", group = " Comp", preselectTeleOp = "Tele-op Blue")
public class CameraMeasurementAuto extends OpMode {

    Climber climber;
    Follower follower;
    Drivetrain driveTrain;
    BlockVision blockVision;
    PowerTakeOff powerTakeOff;
    IntakeSystem intakeSystem;
    OuttakeSystem outtakeSystem;
    MultiDistanceCalculator multiDistanceCalculator;

    private Timer pathTimer, actionTimer, opmodeTimer;

    boolean polar = true; // determines if you rotate the robot or if you move horizontally

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
    private final Pose subIntake = new Pose(4, -31, Math.toRadians(90)); //todo: set the y back from the wall a little to give the robot space to rotate

    private final Pose placeSub1 = new Pose(6, -34, Math.toRadians(90));

    private final Pose spitBlock = new Pose(31, -53, Math.toRadians(-35));

    private final Pose park = new Pose(63.75, -64.3, Math.toRadians(90));

    Pose target;

    private boolean driveShake = false;

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path firstIntakePath;

    private Path spitBlockPath;

    private Path extraIntakePath;

    private Path parkPath; //IS THAT A BTD REFERENCE?


    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        climber = new Climber(hardwareMap);
        follower = new Follower(hardwareMap);
        driveTrain = new Drivetrain(hardwareMap);
        blockVision = new BlockVision(hardwareMap, RobotSideEnum.Blue);
        powerTakeOff = new PowerTakeOff(hardwareMap);
        intakeSystem = new IntakeSystem(hardwareMap, RobotSideEnum.Blue);
        outtakeSystem = new OuttakeSystem(hardwareMap);
        multiDistanceCalculator = new MultiDistanceCalculator(hardwareMap);

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
        firstIntakePath        = follower.linearPathBuilder(startPose, subIntake);

        //frontWallToWall
        spitBlockPath = new Path(new BezierCurve(new Point(subIntake), new Point(startPose), new Point(placeSub1)));
        spitBlockPath.setConstantHeadingInterpolation(subIntake.getHeading());

        extraIntakePath = new Path(new BezierCurve(new Point(spitBlock), new Point(startPose), new Point(subIntake)));
        extraIntakePath.setConstantHeadingInterpolation(spitBlock.getHeading());
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // drive to the sub
                follower.setMaxPower(1);
                setPathState(1);
                break;
            case 1:
                follower.followYourHeart(1.0);
                setPathState(2);
            case 2:
                if (follower.getVelocityMagnitude() < .2    ) {
                    telemetry.addData("time", actionTimer.getElapsedTimeSeconds());
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
        if (driveShake && pathTimer.getElapsedTimeSeconds() > 2.5) {
            if (Math.round((pathTimer.getElapsedTimeSeconds() - 2.5) * 2.5) % 2 == 0 ){
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
        if (target != null) {
            telemetry.addData("targetx", target.getX());
            telemetry.addData("targety", target.getY());
            telemetry.addData("targeth", target.getHeading());
        }
        telemetry.update();

    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}


