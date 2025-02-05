package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utility.SampleAutoEnum;
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
    private SampleAutoEnum pathState;
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
    private final Pose placeSpike1 = new Pose(-59, -54.09, Math.toRadians(74.5));

    private final Pose intakeSpike2 = new Pose(-60.4, -51.9, Math.toRadians(90));
    private final Pose placeSpike2 = new Pose(-61.7, -53, Math.toRadians(80));

    private final Pose intakeSpike3 = new Pose(-59.67, -51.5, Math.toRadians(113));
    private final Pose placeSpike3 = new Pose(-61.7, -53, Math.toRadians(80));

    private final Pose subIntake = new Pose(-24.5, -7.5, Math.toRadians(0));
    private final Pose subIntakeControlPoint = new Pose(-58.5, -12, Math.toRadians(0));

    private Pose target = new Pose();
    private boolean driveShake = false;
    private boolean driveSweep = false;
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
        setPathState(SampleAutoEnum.scorePreload);
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
        intakeSubPath.setLinearHeadingInterpolation(placeSpike3.getHeading(), subIntake.getHeading());

        outtakeSubPath = new Path(new BezierCurve(new Point(subIntake), new Point(placeSpike1), new Point(subIntakeControlPoint)));
        outtakeSubPath.setLinearHeadingInterpolation(subIntake.getHeading(), placeSpike1.getHeading());
    }
    public void autonomousPathUpdate() {
        if (transferSample) {
            telemetry.addData("transfer", transferSample);
            telemetry.addData("transferState", transferState);
            switch (transferState) {
                case -1:
                    actionTimer.resetTimer();
                    transferState = 0;
                    break;
                case 0:
                    if (actionTimer.getElapsedTimeSeconds() > .45){
                        intakeSystem.setIntakePower(Constants.Intake.transferSpeed);

                        setTransferState(1);
                    }
                    break;
                case 1:
                    if (actionTimer.getElapsedTimeSeconds() > .2 && intakeSystem.readyToTranfer()){
                        outtakeSystem.setVSlidePos(Constants.Outtake.intakeSlides);

                        setTransferState(2);
                    }
                    break;
                case 2:
                    if (actionTimer.getElapsedTimeSeconds() > .35){
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
                        intakeSystem.setIntakeServoPos(Constants.Intake.wristStore);
                        transferSample = false;

                        setTransferState(-1);
                    }
            }
        } else {
            telemetry.addData("transfer", transferSample);
        }
        switch (pathState) {
            case scorePreload:
                follower.followPath(scorePreload);
                outtakeSystem.setVSlidePos(Constants.Outtake.highBasketSlides);
                setPathState(SampleAutoEnum.placePreload);
                break;
            case placePreload:
                if (outtakeSystem.getVSlidePos() > outtakeSystem.getVSlideTargetPos() - 50) {
                    intakeSystem.setHSlidePos(Constants.Intake.intakeSlidePos);
                    outtakeSystem.setArmPos(Constants.Outtake.basketArm);
                    setPathState(SampleAutoEnum.dropClaw);
                }
                break;
            case dropClaw:
                if (pathTimer.getElapsedTimeSeconds() > .75) {
                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    setPathState(SampleAutoEnum.intakeSpike1);
                }
                break;
            case intakeSpike1:
                if (pathTimer.getElapsedTimeSeconds() > .3){
                    follower.followPath(intakeSpike1Path);
                    outtakeSystem.setArmPos(Constants.Outtake.intakeArm);
                    intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
                    outtakeSystem.setVSlidePos(Constants.Outtake.intakeWaitSlides);
                    driveShake = true;
                    setPathState(SampleAutoEnum.spike1Transfer);
                }
                break;
            case spike1Transfer:
                if (intakeSystem.intakeUntil() || pathTimer.getElapsedTimeSeconds() > 2){
                    follower.followPath(placeSpike1Path);
                    intakeSystem.storePos();
                    transferSample = true;
                    driveShake = false;
                    setPathState(SampleAutoEnum.placeSample);
                }
                break;
            case placeSample:
                if (!transferSample) {
                    intakeSystem.setHSlidesInches(10); // we had issues with sweeping the block
                    outtakeSystem.setArmPos(Constants.Outtake.basketArm);
                    setPathState(SampleAutoEnum.dropClaw2);
                }
                break;
            case dropClaw2:
                if (pathTimer.getElapsedTimeSeconds() > .2) {
                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
                    setPathState(SampleAutoEnum.intakeSpike2);
                }
                break;
            case intakeSpike2:
                if (pathTimer.getElapsedTimeSeconds() > .3) {
                    follower.followPath(intakeSpike2Path);
                    outtakeSystem.setArmPos(Constants.Outtake.intakeArm);
                    outtakeSystem.setVSlidePos(Constants.Outtake.intakeWaitSlides);
                    driveShake = true;
                    setPathState(SampleAutoEnum.spike2Transfer);
                }
                break;
            case spike2Transfer:
                if (intakeSystem.intakeUntil() || pathTimer.getElapsedTimeSeconds() > 2){
                    driveShake = false;
                    follower.followPath(placeSpike2Path);
                    intakeSystem.storePos();
                    transferSample = true;
                    setPathState(SampleAutoEnum.placeSample2);
                } else if (pathTimer.getElapsedTimeSeconds() > .2) {
                    intakeSystem.setHSlidePos(Constants.Intake.intakeSlidePos);
                }
                break;
            case placeSample2:
                if (!transferSample) {
                    outtakeSystem.setArmPos(Constants.Outtake.basketArm);
                    intakeSystem.setHSlidePos(Constants.Intake.intakeSlidePos);
                    setPathState(SampleAutoEnum.dropClaw3);
                }
                break;
            case dropClaw3:
                if (pathTimer.getElapsedTimeSeconds() > .2) {
                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    setPathState(SampleAutoEnum.intakeSpike3);
                }
                break;
            case intakeSpike3:
                if (pathTimer.getElapsedTimeSeconds() > .3) {
                    follower.followPath(intakeSpike3Path);
                    outtakeSystem.setArmPos(Constants.Outtake.intakeArm);
                    intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
                    outtakeSystem.setVSlidePos(Constants.Outtake.intakeWaitSlides);
                    driveShake = true;
                    setPathState(SampleAutoEnum.spike3Transfer);
                }
                break;
            case spike3Transfer:
                if (intakeSystem.intakeUntil() || pathTimer.getElapsedTimeSeconds() > 2){
                    driveShake = false;
                    follower.followPath(placeSpike3Path);
                    intakeSystem.storePos();
                    transferSample = true;
                    setPathState(SampleAutoEnum.placeSample3);
                }
                break;
            case placeSample3:
                if (!transferSample) {
                    driveShake = false;
                    outtakeSystem.setArmPos(Constants.Outtake.basketArm);
                    setPathState(SampleAutoEnum.dropClaw4);
                }
                break;
            case dropClaw4:
                if (pathTimer.getElapsedTimeSeconds() > .2) {
                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    setPathState(SampleAutoEnum.intakeArm);
                }
                break;
            case intakeArm:
                if (pathTimer.getElapsedTimeSeconds() > .2){
                    outtakeSystem.setArmPos(Constants.Outtake.intakeArm);
                    setPathState(SampleAutoEnum.goSub1);
                }
                break;
            case goSub1:
                if (pathTimer.getElapsedTimeSeconds() > .3) {
                    follower.followPath(intakeSubPath);
                    outtakeSystem.setVSlidePos(Constants.Outtake.intakeWaitSlides);
                    setPathState(SampleAutoEnum.visionSearch1);
                }
                break;
            case visionSearch1:
                if (follower.getVelocity().getXComponent() <= 1 && follower.getVelocity().getYComponent() <= 1) {
                    target = blockVision.getBestSampleBlockPos();
                    if (target != null) {
                        intakeSystem.setHSlidesInches(target.getY());
                        follower.followYourHeart(target.getX());
                        setPathState(SampleAutoEnum.subIntake1);
                        driveSweep = true;
                    }
                }
                break;
            case subIntake1:
                if (pathTimer.getElapsedTimeSeconds() > .8) {
                    intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);

                    setPathState(SampleAutoEnum.transferSample4);
                }
                break;
            case transferSample4:
                if (intakeSystem.intakeUntil()) {
                    follower.followPath(outtakeSubPath);
                    intakeSystem.storePos();
                    transferSample = true;
                    setPathState(SampleAutoEnum.placeSample4);
                }
                break;
            case placeSample4:
                if (!transferSample && follower.getError(placeSpike1).getX() > 1 && follower.getError(placeSpike1).getY() > 1) {
                    outtakeSystem.setArmPos(Constants.Outtake.basketArm);

                    setPathState(SampleAutoEnum.dropClaw5);
                }
            case dropClaw5:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);

                    setPathState(SampleAutoEnum.intakeArm2);
                }
                break;
            case intakeArm2:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    outtakeSystem.setArmPos(Constants.Outtake.intakeArm);
                    
                    setPathState(SampleAutoEnum.goSub1);
                }
                break;
        }
    }

    public void setPathState(SampleAutoEnum pState) {
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
                driveTrain.drive(0,0, 0.5, DriveSpeedEnum.Auto);
            } else {
                driveTrain.drive(0,0, -0.5, DriveSpeedEnum.Auto);
            }
        }

        if (driveSweep && pathTimer.getElapsedTimeSeconds() > 1.9) {
            if (Math.round((pathTimer.getElapsedTimeSeconds() - 0.9) * .5) % 2 == 0 ){
                driveTrain.drive(0,0, 0.5, DriveSpeedEnum.Auto);
            } else {
                driveTrain.drive(0,0, -0.5, DriveSpeedEnum.Auto);
            }
        }

        // Feedback to FTC Dashboard
        Drawing.drawDebug(follower);

        // Feedback to Driver Hub
        if (true) {
            telemetry.addData("target", target);
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }
}
