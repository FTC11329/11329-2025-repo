package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

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

@Autonomous(name = "Camera Measurement Auto", group = "Test")
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

    private int measurementType = 0;
    private  double movementX = 1;
    private  double extendo = 1;
    private  double rotation = 1;

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

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdateX() {
        switch (pathState) {
            case 0:
                // drive to the sub
                follower.setMaxPower(1);
                setPathState(1);
                break;
            case 1:
                follower.followYourHeart(movementX);
                setPathState(2);
                break;
            case 2:
                if (follower.getVelocityMagnitude() < .05) {
                    telemetry.addData("time", actionTimer.getElapsedTimeSeconds());
                    setPathState(3);
                }
                break;
            case 3:
                follower.followYourHeart(-movementX);
                setPathState(4);
                break;
            case 4:
                if (follower.getVelocityMagnitude() < .05) {
                    telemetry.addData("-time" + movementX, actionTimer.getElapsedTimeSeconds());
                    setPathState(5);
                }
                break;
            case 5:
                if (follower.getVelocityMagnitude() < .01){
                    if (movementX > 12) {
                        setPathState(6);
                    }
                    else {
                        movementX += 2;
                        setPathState(0);
                    }
                }
                break;
        }
    }

    public void autonomousPathUpdateExtendo() {
        switch (pathState) {
            case 0:
                // drive to the sub
                intakeSystem.setIntakePower(1);
                setPathState(1);
                break;
            case 1:
                intakeSystem.setHSlidesInches(extendo);
                setPathState(2);
                break;
            case 2:
                if (Math.abs(intakeSystem.getHSlideTargetPos() - intakeSystem.getHSlidePos()) < .05) {
                    telemetry.addData("time", actionTimer.getElapsedTimeSeconds());
                    setPathState(3);
                }
                break;
            case 3:
                intakeSystem.setHSlidePos(Constants.Intake.minSlidePos);
                setPathState(4);
                break;
            case 4:
                if (Math.abs(intakeSystem.getHSlideTargetPos() - intakeSystem.getHSlidePos()) < .05) {
                    telemetry.addData("-time" + movementX, actionTimer.getElapsedTimeSeconds());
                    setPathState(5);
                }
                break;
            case 5:
                if (Math.abs(intakeSystem.getHSlideTargetPos() - intakeSystem.getHSlidePos()) < .01){
                    if (extendo > 12) {
                        setPathState(6);
                    }
                    else {
                        extendo += 2;
                        setPathState(0);
                    }
                }
                break;
        }
    }

    //Isn't quite finished yet.
    public void autonomousPathUpdateRotation() {
        switch (pathState) {
            case 0:
                // drive to the sub
                setPathState(1);
                break;
            case 1:
                driveTrain.drive(0,0, rotation, DriveSpeedEnum.Auto);
                setPathState(2);
                break;
            case 2:
                if (Math.abs(intakeSystem.getHSlideTargetPos() - intakeSystem.getHSlidePos()) < .05) {
                    telemetry.addData("time", actionTimer.getElapsedTimeSeconds());
                    setPathState(3);
                }
                break;
            case 3:
                driveTrain.drive(0,0, -rotation, DriveSpeedEnum.Auto);
                setPathState(4);
                break;
            case 4:
                if (Math.abs(intakeSystem.getHSlideTargetPos() - intakeSystem.getHSlidePos()) < .05) {
                    telemetry.addData("-time" + movementX, actionTimer.getElapsedTimeSeconds());
                    setPathState(5);
                }
                break;
            case 5:
                if (Math.abs(intakeSystem.getHSlideTargetPos() - intakeSystem.getHSlidePos()) < .01){
                    if (rotation > 12) {
                        setPathState(6);
                    }
                    else {
                        extendo += 2;
                        setPathState(0);
                    }
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
        intakeSystem.update();
        switch (measurementType){
            case 0:
                autonomousPathUpdateX();
                break;
            case 1:
                autonomousPathUpdateExtendo();
                break;
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


