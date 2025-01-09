package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Drawing;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.BlockVision;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.PowerTakeOff;
import org.firstinspires.ftc.teamcode.utility.DriveSpeedEnum;
import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

public class SampleAuto {
    Telemetry telemetry;
    HardwareMap hardwareMap;
    RobotSideEnum robotSide;

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
    private int transferState = -1;

    /** Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left. But we don't want do so we don't, 0,0 is center off the field
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     **/

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(-40, -63, Math.toRadians(90));

    /** Scoring Poses of our robot. */
    private final Pose preloadPlace = new Pose(-55.5, -55.5, Math.toRadians(63));
    private final Pose intakeSpike1 = new Pose(-55, -53.5, Math.toRadians(75.5));
    private final Pose placeSpike1 = new Pose(-59.98, -54.09, Math.toRadians(74.5));

    private final Pose intakeSpike2 = new Pose(-60.4, -51.9, Math.toRadians(85.5));
    private final Pose placeSpike2 = new Pose(-61.7, -53, Math.toRadians(80));

    private final Pose intakeSpike3 = new Pose(-59.67, -51.5, Math.toRadians(113));
    private final Pose placeSpike3 = new Pose(-58.6, -55, Math.toRadians(55.8));

    private final Pose subIntake = new Pose(-23, -7.5, Math.toRadians(0));
    private final Pose subIntakeControlPoint = new Pose(-58.5, -12, Math.toRadians(0));

    private Pose target = new Pose();
    private boolean driveShake = false;
    private boolean transferSample = false;

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload;

    private Path intakeSpike1Path;
    private Path placeSpike1Path;


    private Path intakeSpike2Path;
    private Path placeSpike2Path;

    private Path intakeSpike3Path;
    private Path placeSpike3Path;

    private Path intakeSubPath;
    private Path outtakeSubPath;

    public SampleAuto(HardwareMap hardwareMap, Telemetry telemetry, RobotSideEnum robotSide) {
        this.robotSide = robotSide;
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        climber = new Climber(hardwareMap);
        follower = new Follower(hardwareMap);
        driveTrain = new Drivetrain(hardwareMap);
        blockVision = new BlockVision(hardwareMap, robotSide);
        powerTakeOff = new PowerTakeOff(hardwareMap);
        intakeSystem = new IntakeSystem(hardwareMap, robotSide);
        outtakeSystem = new OuttakeSystem(hardwareMap);

        outtakeSystem.setArmPos(Constants.Outtake.initAutoArm);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();

        opmodeTimer.resetTimer();

        follower.setStartingPose(startPose);

        buildPaths();
    }

    public void init_loop() {
        telemetry.addData("Test Block", blockVision.getBestSampleBlockPos());
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

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
        scorePreload = follower.linearPathBuilder(startPose, preloadPlace);

        intakeSpike1Path = follower.linearPathBuilder(preloadPlace, intakeSpike1);
        placeSpike1Path = follower.linearPathBuilder(intakeSpike1, placeSpike1);


        intakeSpike2Path = follower.linearPathBuilder(placeSpike1, intakeSpike2);
        placeSpike2Path = follower.linearPathBuilder(intakeSpike2, placeSpike2);

        intakeSpike3Path = follower.linearPathBuilder(placeSpike2, intakeSpike3);
        placeSpike3Path = follower.linearPathBuilder(intakeSpike3, placeSpike3);

        intakeSubPath = new Path(new BezierCurve(new Point(placeSpike3), new Point(subIntakeControlPoint), new Point(subIntake)));
        intakeSubPath.setTangentHeadingInterpolation();

        outtakeSubPath = new Path(new BezierCurve(new Point(subIntake), new Point(subIntakeControlPoint), new Point(placeSpike3)));
        outtakeSubPath.setTangentHeadingInterpolation();
    }
    public void autonomousPathUpdate() {
        if (transferSample) {
            telemetry.addData("transfer", transferSample);
            telemetry.addData("transferState", transferState);
            telemetry.update();
            switch (transferState) {
                case -1:
                    actionTimer.resetTimer();
                    transferState = 0;
                    break;
                case 0:
                    if (actionTimer.getElapsedTimeSeconds() > .3){
                        intakeSystem.setIntakePower(Constants.Intake.transferSpeed);
                        setTransferState(1);
                    }
                    break;
                case 1:
                    if (actionTimer.getElapsedTimeSeconds() > .3 && intakeSystem.readyToTranfer()){
                        outtakeSystem.setVSlidePos(Constants.Outtake.intakeSlides);
                        transferState = 2;
                        actionTimer.resetTimer();
                        setTransferState(2);
                    }
                    break;
                case 2:
                    if (actionTimer.getElapsedTimeSeconds() > .5){
                        intakeSystem.setIntakeServoPos(Constants.Intake.wristClear);
                        intakeSystem.setIntakePower(0);
                        outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                        setTransferState(3);
                    }
                    break;
                case 3:
                    if (actionTimer.getElapsedTimeSeconds() > .3) {
                        outtakeSystem.setVSlidePos(Constants.Outtake.safeFromHSlides);
                        setTransferState(4);
                    }
                    break;
                case 4:
                    if (actionTimer.getElapsedTimeSeconds() > .3) {
                        outtakeSystem.setVSlidePos(Constants.Outtake.highBasketSlides);
                        outtakeSystem.setArmPos(Constants.Outtake.upArm);
                        setTransferState(5);
                    }
                    break;
                case 5:
                    if (actionTimer.getElapsedTimeSeconds() > .4 && outtakeSystem.getVSlidePos() > outtakeSystem.getVSlideTargetPos() - 50) {
                        transferSample = false;
                        actionTimer.resetTimer();
                        setTransferState(-1);
                    }
            }
        } else {
            telemetry.addData("transfer", transferSample);
            telemetry.update();
        }
        switch (pathState) {
            //go to score preload
            case 0:
                follower.followPath(scorePreload);
                outtakeSystem.setVSlidePos(Constants.Outtake.highBasketSlides);
                setPathState(1);
                break;
            case 1:
                if (outtakeSystem.getVSlidePos() > outtakeSystem.getVSlideTargetPos() - 50) {
                    intakeSystem.setHSlidePos(Constants.Intake.intakeSlidePos);
                    outtakeSystem.setArmPos(Constants.Outtake.basketArm);
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTimeSeconds() > .75) {
                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTimeSeconds() > .3){
                    follower.followPath(intakeSpike1Path);
                    outtakeSystem.setArmPos(Constants.Outtake.intakeArm);
                    intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
                    outtakeSystem.setVSlidePos(Constants.Outtake.intakeWaitSlides);
                    driveShake = true;
                    setPathState(4);
                }
                break;
            case 4:
                if (intakeSystem.intakeUntil() || pathTimer.getElapsedTimeSeconds() > 2){
                    follower.followPath(placeSpike1Path);
                    intakeSystem.storePos();
                    transferSample = true;
                    driveShake = false;
                    setPathState(5);
                }
                break;
            case 5:
                if (!transferSample) {
                    intakeSystem.setHSlidePos(Constants.Intake.intakeSlidePos);
                    outtakeSystem.setArmPos(Constants.Outtake.basketArm);
                    setPathState(6);
                }
                break;
            case 6:
                if (pathTimer.getElapsedTimeSeconds() > .2) {
                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    setPathState(7);
                }
                break;
            case 7:
                if (pathTimer.getElapsedTimeSeconds() > .3) {
                    follower.followPath(intakeSpike2Path);
                    outtakeSystem.setArmPos(Constants.Outtake.intakeArm);
                    intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
                    outtakeSystem.setVSlidePos(Constants.Outtake.intakeWaitSlides);
                    driveShake = true;
                    setPathState(8);
                }
                break;
            case 8:
                if (intakeSystem.intakeUntil() || pathTimer.getElapsedTimeSeconds() > 2){
                    driveShake = false;
                    follower.followPath(placeSpike2Path);
                    intakeSystem.storePos();
                    transferSample = true;
                    setPathState(9);
                }
                break;
            case 9:
                if (!transferSample) {
                    outtakeSystem.setArmPos(Constants.Outtake.basketArm);
                    intakeSystem.setHSlidePos(Constants.Intake.intakeSlidePos);
                    setPathState(10);
                }
                break;
            case 10:
                if (pathTimer.getElapsedTimeSeconds() > .2) {
                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    setPathState(11);
                }
                break;
            case 11:
                if (pathTimer.getElapsedTimeSeconds() > .3) {
                    follower.followPath(intakeSpike3Path);
                    outtakeSystem.setArmPos(Constants.Outtake.intakeArm);
                    intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
                    outtakeSystem.setVSlidePos(Constants.Outtake.intakeWaitSlides);
                    driveShake = true;
                    setPathState(12);
                }
                break;
            case 12:
                if (intakeSystem.intakeUntil() || pathTimer.getElapsedTimeSeconds() > 2){
                    driveShake = false;
                    follower.followPath(placeSpike3Path);
                    intakeSystem.storePos();
                    transferSample = true;
                    setPathState(13);
                }
                break;
            case 13:
                if (!transferSample) {
                    outtakeSystem.setArmPos(Constants.Outtake.basketArm);
                    setPathState(14);
                }
                break;
            case 14:
                if (pathTimer.getElapsedTimeSeconds() > .2) {
                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    setPathState(15);
                }
                break;
            case 15:
                if (pathTimer.getElapsedTimeSeconds() > .2){
                    outtakeSystem.setArmPos(Constants.Outtake.upArm);
                    setPathState(16);
                }
            case 16:
                if (pathTimer.getElapsedTimeSeconds() > .3) {
                    follower.followPath(intakeSubPath);
                    outtakeSystem.setVSlidePos(Constants.Outtake.intakeWaitSlides);
                    outtakeSystem.setArmPos(Constants.Outtake.intakeArm);
                    setPathState(17);
                }
                break;
            case 17:
                if (follower.getError(subIntake).getX() < 1 && follower.getError(subIntake).getY() < 1) {
                    target = blockVision.getBestSampleBlockPos();
                    intakeSystem.setHSlidesInches(target.getY());
                    follower.runBlockErrorPID(target.getX());
                    setPathState(18);
                }
                break;
            case 18:
//                follower.runBlockErrorPID();
                if (pathTimer.getElapsedTimeSeconds() > .3) {
                    intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
                    setPathState(19);
                }
                break;
            case 19:
//                follower.runBlockErrorPID();
                if (intakeSystem.intakeUntil()) {
                    intakeSystem.storePos();
                    setPathState(20);
                }
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    public void setTransferState(int tState) {
        transferState = tState;
        actionTimer.resetTimer();
    }

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
        telemetry.addData("target", target);
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}
