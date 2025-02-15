package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Drawing;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.Attempt89;
import org.firstinspires.ftc.teamcode.subsystems.BlockVision;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.MultiDistanceCalculator;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.PowerTakeOff;
import org.firstinspires.ftc.teamcode.utility.DriveSpeedEnum;
import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

@Autonomous(name = "Camera Auto", group = "Test")
public class CameraAuto extends OpMode {

    Climber climber;
    Follower follower;
    Drivetrain driveTrain;
    BlockVision blockVision;
    PowerTakeOff powerTakeOff;
    IntakeSystem intakeSystem;
    OuttakeSystem outtakeSystem;
    MultiDistanceCalculator multiDistanceCalculator;

    RobotSideEnum robotSideEnum;

    Attempt89 attempt89;
    private Timer pathTimer, actionTimer, opmodeTimer;

    boolean polar = false; // determines if you rotate the robot or if you move horizontally

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
    private final Pose subIntake = new Pose(4, -40, Math.toRadians(90)); //todo: set the y back from the wall a little to give the robot space to rotate

    private final Pose placeSub1 = new Pose(6, -34, Math.toRadians(90));

    private final Pose spitBlock = new Pose(31, -53, Math.toRadians(-35));

    private final Pose park = new Pose(63.75, -64.3, Math.toRadians(90));

    Pose2D target;

    private boolean driveShake = false;

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path firstIntakePath;

    boolean work = true; // yayy!!
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
        attempt89 = new Attempt89(hardwareMap, RobotSideEnum.Blue);
        outtakeSystem.setArmPos(Constants.Outtake.initAutoArm);
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        follower.setStartingPose(startPose);
        attempt89.switchPipeline(2);

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

    public void buildPaths() {
        firstIntakePath        = follower.linearPathBuilder(startPose, subIntake);

        //frontWallToWall
        spitBlockPath = new Path(new BezierCurve(new Point(subIntake), new Point(startPose), new Point(placeSub1)));
        spitBlockPath.setConstantHeadingInterpolation(subIntake.getHeading());

        extraIntakePath = new Path(new BezierCurve(new Point(spitBlock), new Point(startPose), new Point(subIntake)));
        extraIntakePath.setConstantHeadingInterpolation(spitBlock.getHeading());
    }

    public Pose getBlockPosition() {

        double[][][] distanceArray = multiDistanceCalculator.fillImageArray();
        for (int block = 0; block < distanceArray[0].length; block++) {
            for (int frame = 0; frame < distanceArray[0][0].length; frame++) {
                telemetry.addData("DistanceArray[0][" + block + "][" + frame + "]: ", distanceArray[0][block][frame]);
                telemetry.addData("DistanceArray[1][" + block + "][" + frame + "]: ", distanceArray[1][block][frame]);
            }
        }
        //telemetry.addData("newDistanceArray[][][]: ", distanceArray);

        double[][] newDistanceArray = multiDistanceCalculator.findAverageOfFullFrames(distanceArray);
        for (int block = 0; block < newDistanceArray[0].length; block++) {
            telemetry.addData("newDistanceArray[0][" + block + "]:", newDistanceArray[0][block]);
            telemetry.addData("newDistanceArray[1][" + block + "]:", newDistanceArray[1][block]);
        }
        //telemetry.addData("newDistanceArray[][]: ", newDistanceArray);

        // Call the method to find the smallest non-zero values
        double[] finalValues = multiDistanceCalculator.MinimizeTime(newDistanceArray);
        //telemetry.addData("finalValues[][]: ", finalValues);

        //find the closest non-zero distance block

        return new Pose(finalValues[0], finalValues[1], 0.0);
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // drive to the sub
//                follower.setMaxPower(1);
                //follower.followPath(firstIntakePath, true);
//                outtakeSystem.setArmPos(Constants.Outtake.upArm);
                setPathState(1);
                break;
            case 1:
                target = attempt89.getBestBlock(2); //red
                if (target != null) {
                    telemetry.addData("targetx", target.getX(DistanceUnit.INCH));
                    telemetry.addData("targety", target.getY(DistanceUnit.INCH));
                    telemetry.addData("target", target.getHeading(AngleUnit.DEGREES));
                }
                telemetry.update();
                if (follower.getVelocityMagnitude() < .1 && follower.getVelocityMagnitude() < .1 && pathTimer.getElapsedTimeSeconds() > 1.5) {
                    //camera takes photo and starts intake
                    if (target != null){
                        if (polar) {
                            //Thanks Mr. Raney
                            double r = Math.sqrt((target.getY(DistanceUnit.INCH)*target.getY(DistanceUnit.INCH))+(target.getX(DistanceUnit.INCH)*target.getX(DistanceUnit.INCH)));
                            double theta = Math.atan(target.getY(DistanceUnit.INCH)/ target.getX(DistanceUnit.INCH));
                            intakeSystem.setHSlidesInches(r);
                            follower.followYourHead(Math.toRadians(theta));
                        } else {
                            intakeSystem.setHSlidesInches(target.getY(DistanceUnit.INCH));
                            follower.followYourHeart(target.getX(DistanceUnit.INCH));
                        }
//                        driveShake = true; //updated so that there is a longer delay
                        setPathState(2);
                    } else {
                        telemetry.addData("","No Blocks Found");
                        telemetry.update();
                    }
                    break;
                }
                break;
            case 2:
                //puts the wrist down after the slides are in the sub
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
                    setPathState(3);
                }
                break;
            case 3:
                // intakes and moves to store pos
                if (intakeSystem.intakeUntilColor()) {
                    driveShake = false;
                    intakeSystem.storePos();
                    setPathState(4);
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
        if (driveShake && pathTimer.getElapsedTimeSeconds() > 2.5) {
            if (Math.round((pathTimer.getElapsedTimeSeconds() - 2.5) * 2.5) % 2 == 0 ){
                driveTrain.drive(0,0, -0.5, DriveSpeedEnum.Auto);
            } else {
                driveTrain.drive(0,0, 0.5, DriveSpeedEnum.Auto);
            }
        }



    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}


