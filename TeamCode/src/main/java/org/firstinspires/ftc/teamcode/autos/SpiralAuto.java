package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedropathing.follower.FollowerConstants;
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
import org.firstinspires.ftc.teamcode.utility.SpiralAutoEnum;

public class SpiralAuto {
    Climber climber;
    Follower follower;
    Drivetrain driveTrain;
    PowerTakeOff powerTakeOff;
    IntakeSystem intakeSystem;
    OuttakeSystem outtakeSystem;

    private Timer pathTimer, opmodeTimer;

    private SpiralAutoEnum pathState;

    // Moving Poses of our robot.
    private final Pose startPose = new Pose(-63, 60, Math.toRadians(180));
    private final Pose almostUp1 = new Pose(25, 58, Math.toRadians(180));
    private final Pose up1    = new Pose(55, 58, Math.toRadians(-90));
    private final Pose almostRight1 = new Pose(55, -25, Math.toRadians(-90));
    private final Pose right1 = new Pose(55, -55, Math.toRadians(180));
    private final Pose almostDown1  = new Pose(-25, -55, Math.toRadians(180));
    private final Pose down1  = new Pose(-55, -55, Math.toRadians(90));
    private final Pose almostLeft1  = new Pose(-55, 15, Math.toRadians(90));
    private final Pose left1  = new Pose(-55, 36, Math.toRadians(0));

    private final Pose almostUp2    = new Pose(15, 36, Math.toRadians(0));
    private final Pose up2    = new Pose(36, 36, Math.toRadians(90));
    private final Pose almostRight2 = new Pose(36, -15, Math.toRadians(90));
    private final Pose right2 = new Pose(36, -33, Math.toRadians(45));
    private final Pose spit   = new Pose(-22, -32, Math.toRadians(45));
    private final Pose down2  = new Pose(-31, -32, Math.toRadians(45));
    private final Pose retract = new Pose(-31, -24, Math.toRadians(45));
    private final Pose left2  = new Pose(-31, 12, Math.toRadians(45));

    private final Pose up3    = new Pose(12, 12, Math.toRadians(45));
    private final Pose right3 = new Pose(13.75, -10, Math.toRadians(0));
    //Various Variables
    private double loopTime = 0;
    private double errorDistance = 9;

    private double endTime = 0;
    // These are our Paths and PathChains that we will define in buildPaths()
    PathChain up1Path, right1Path, down1Path, left1Path, up2Path, right2Path;
    Path down2Path, left2Path, up3Path, right3Path;
    RobotSideEnum robotSide;
    Telemetry telemetry;
    HardwareMap hardwareMap;

    public SpiralAuto(HardwareMap hardwareMap, Telemetry telemetry, RobotSideEnum robotSide) {
        this.robotSide = robotSide;
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }
    /** This method is called once at the init of the OpMode. **/
    public void init() {
        climber = new Climber(hardwareMap);
        follower = new Follower(hardwareMap);
        driveTrain = new Drivetrain(hardwareMap);
        powerTakeOff = new PowerTakeOff(hardwareMap);
        intakeSystem = new IntakeSystem(hardwareMap, robotSide);
        outtakeSystem = new OuttakeSystem(hardwareMap, robotSide, true);

        outtakeSystem.setArmPos(Constants.Outtake.initAutoSpecArm);
        outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
        intakeSystem.setDepoServoPos(Constants.Intake.depoDepo);

        intakeSystem.disable();
        outtakeSystem.disable();

        pathTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        follower.setStartingPose(startPose);

        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    public void start() {
        opmodeTimer.resetTimer();

        outtakeSystem.setArmPos(Constants.Outtake.upArm);

        setPathState(SpiralAutoEnum.up1);
    }

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        double fastSpeedZPAM = 0.05;
        up1Path = follower.pathBuilder()
                .addPath(follower.linearPathBuilder(startPose, almostUp1))
                .setConstantHeadingInterpolation(almostUp1.getHeading())
                .addPath(follower.linearPathBuilder(almostUp1, up1))
                .setLinearHeadingInterpolation(almostUp1.getHeading(), up1.getHeading(), 0.5)
                .build();
        right1Path = follower.pathBuilder()
                .addPath(follower.linearPathBuilder(up1, almostRight1))
                .setConstantHeadingInterpolation(almostRight1.getHeading())
                .addPath(follower.linearPathBuilder(almostRight1, right1))
                .setLinearHeadingInterpolation(almostRight1.getHeading(), right1.getHeading(), 0.5)
                .build();
        down1Path = follower.pathBuilder()
                .addPath(follower.linearPathBuilder(right1, almostDown1))
                .setConstantHeadingInterpolation(almostDown1.getHeading())
                .addPath(follower.linearPathBuilder(almostDown1, down1))
                .setLinearHeadingInterpolation(almostDown1.getHeading(), down1.getHeading(), 0.5)
                .build();
        left1Path = follower.pathBuilder()
                .addPath(follower.linearPathBuilder(down1, almostLeft1))
                .setConstantHeadingInterpolation(almostLeft1.getHeading())
                .addPath(follower.linearPathBuilder(almostLeft1, left1))
                .setLinearHeadingInterpolation(almostLeft1.getHeading(), left1.getHeading(), 0.5)
                .build();


        up2Path = follower.pathBuilder()
                .addPath(follower.linearPathBuilder(left1, almostUp2))
                .setConstantHeadingInterpolation(almostUp2.getHeading())
                .addPath(follower.linearPathBuilder(almostUp2, up2))
                .setLinearHeadingInterpolation(almostUp2.getHeading(), up2.getHeading(), 0.5)
                .build();
        right2Path = follower.pathBuilder()
                .addPath(follower.linearPathBuilder(up2, almostRight2))
                .setConstantHeadingInterpolation(almostRight2.getHeading())
                .addPath(follower.linearPathBuilder(almostRight2, right2))
                .setLinearHeadingInterpolation(almostRight2.getHeading(), right2.getHeading(), 0.5)
                .build();
        down2Path = follower.linearPathBuilder(right2, down2);
        left2Path = follower.linearPathBuilder(down2, left2);

        up3Path = follower.linearPathBuilder(left2, up3);
        right3Path = follower.linearPathBuilder(up3, right3);
        right3Path.setLinearHeadingInterpolation(up3.getHeading(), right3.getHeading(), 0.85);
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState(Specimen6AutoEnum.) method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case up1:
                follower.followPath(up1Path);
                setPathState(SpiralAutoEnum.right1);
                break;

            case right1:
                if (follower.getErrorDistance(up1) < errorDistance) {
                    follower.followPath(right1Path);
                    setPathState(SpiralAutoEnum.down1);
                }
                break;

            case down1:
                if (follower.getErrorDistance(right1) < errorDistance) {
                    follower.followPath(down1Path);
                    setPathState(SpiralAutoEnum.left1);
                }
                break;

            case left1:
                if (follower.getErrorDistance(down1) < errorDistance) {
                    follower.followPath(left1Path);
                    setPathState(SpiralAutoEnum.up2);
                }
                break;

            case up2:
                if (follower.getErrorDistance(left1) < errorDistance) {
                    follower.followPath(up2Path);
                    setPathState(SpiralAutoEnum.right2);
                }
                break;

            case right2:
                if (follower.getErrorDistance(up2) < errorDistance) {
                    follower.followPath(right2Path);
                    setPathState(SpiralAutoEnum.down2);
                }
                break;

            case down2:
                if (follower.getErrorDistance(right2) < errorDistance) {
                    follower.followPath(down2Path);
                    intakeSystem.reEnable(0);
                    intakeSystem.setHSlidePos(Constants.Intake.maxSlidePos);
                    setPathState(SpiralAutoEnum.extend2);
                }
                break;

            case extend2:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
                    setPathState(SpiralAutoEnum.spit);
                }
                break;

            case spit:
                if (follower.getErrorDistance(spit) < 3) {
                    intakeSystem.setIntakePower(-0.75);
                    setPathState(SpiralAutoEnum.left2);
                }
                break;

            case left2:
                if (follower.getErrorDistance(down2) < errorDistance) {
                    follower.followPath(left2Path);
                    setPathState(SpiralAutoEnum.retract);
                }
                break;

            case retract:
                if (follower.getErrorDistance(retract) < 5) {
                    intakeSystem.storePos();
                    intakeSystem.setIntakePower(-0.9);
                    setPathState(SpiralAutoEnum.up3);
                }
                break;

            case up3:
                if (follower.getErrorDistance(left2) < errorDistance) {
                    intakeSystem.setIntakePower(0);
                    follower.followPath(up3Path);
                    setPathState(SpiralAutoEnum.right3);
                }
                break;

            case right3:
                if (follower.getErrorDistance(up3) < errorDistance) {
                    follower.followPath(right3Path);
                    setPathState(SpiralAutoEnum.finish);
                }
                break;

            case finish:
                if (follower.getErrorDistance(right3) < 1) {
                    endTime = opmodeTimer.getElapsedTimeSeconds();
                    setPathState(SpiralAutoEnum.done);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(SpiralAutoEnum pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    public void loop() {
        // These loop the movements of the robot
        follower.update();

        autonomousPathUpdate();

        // Feedback to FTC Dashboard
        Drawing.drawDebug(follower);

        // Feedback to Driver Hub
        if (true) {
            telemetry.addData("path state", pathState);
//            telemetry.addData("x", follower.getPose().getX());
//            telemetry.addData("y", follower.getPose().getY());
//            telemetry.addData("heading", follower.getPose().getHeading());
//            telemetry.addData("tripped", driveTrain.isStalled(3));
//            telemetry.addData("opmode", opmodeTimer.getElapsedTimeSeconds());
//            telemetry.addData("loopTime", loopTime - opmodeTimer.getElapsedTime());
//            telemetry.addData("fl", driveTrain.getDrivePowers()[0]);
//            telemetry.addData("bl", driveTrain.getDrivePowers()[1]);
//            telemetry.addData("fr", driveTrain.getDrivePowers()[2]);
//            telemetry.addData("br", driveTrain.getDrivePowers()[3]);
//            telemetry.addData("ma", Math.max(Math.max(driveTrain.getDriveCurrent()[0], driveTrain.getDriveCurrent()[1]), Math.max(driveTrain.getDriveCurrent()[2], driveTrain.getDriveCurrent()[3])));
            loopTime = opmodeTimer.getElapsedTime();
        }
        telemetry.addData("TIME", endTime);
        telemetry.update();

    }

    public void stop() {
        driveTrain.stopDrive();
    }
}