package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Path;
import org.firstinspires.ftc.teamcode.pedropathing.util.Drawing;
import org.firstinspires.ftc.teamcode.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.Attempt89;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.PowerTakeOff;
import org.firstinspires.ftc.teamcode.utility.DriveSpeedEnum;
import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

@Autonomous(name = "Camera Auto Wiggle", group = "Test")
public class CameraAutoWiggle extends OpMode {

    Climber climber;
    Follower follower;
    Drivetrain driveTrain;
    PowerTakeOff powerTakeOff;
    IntakeSystem intakeSystem;
    OuttakeSystem outtakeSystem;

    RobotSideEnum robotSideEnum;

    Attempt89 attempt89;
    private Timer pathTimer, actionTimer, opmodeTimer;

    boolean polar = true; // determines if you rotate the robot or if you move horizontally
    boolean wiggle = false; // This makes the robot move while true
    boolean wiggleBot = false; // This makes the robot try wiggle pathing
    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState = 1;

    /** Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left. But we don't want do so we don't, 0,0 is center off the field
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     **/

    private final Pose startPose = new Pose(0, 0, Math.toRadians(90));

    Pose2D target;

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path firstIntakePath;

    boolean work = true; // yayy!!


    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        climber = new Climber(hardwareMap);
        follower = new Follower(hardwareMap);
        driveTrain = new Drivetrain(hardwareMap);
        powerTakeOff = new PowerTakeOff(hardwareMap);
        intakeSystem = new IntakeSystem(hardwareMap, RobotSideEnum.Blue);
        outtakeSystem = new OuttakeSystem(hardwareMap);
        attempt89 = new Attempt89(hardwareMap, RobotSideEnum.Blue);
        outtakeSystem.setArmPos(Constants.Outtake.intakeWallArm);
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        follower.setStartingPose(startPose);
        attempt89.switchPipeline(0);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        target = attempt89.getBlockPosition();
        if (target != null) {
            telemetry.addData("targetx", target.getX(DistanceUnit.INCH));
            telemetry.addData("targety", target.getY(DistanceUnit.INCH));
            telemetry.addData("target", target.getHeading(AngleUnit.DEGREES));
        }
        telemetry.update();
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(1);
    }



    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 1:
                target = attempt89.getBlockPosition();
                telemetry.addData("targetx", target.getX(DistanceUnit.INCH));
                telemetry.addData("targety", target.getY(DistanceUnit.INCH));
                telemetry.addData("target", target.getHeading(AngleUnit.DEGREES));
                telemetry.update();
                //camera takes photo and starts intake
                if (target != null && target.getHeading(AngleUnit.DEGREES) != -1){
                    if (polar) {
                        //Thanks Mr. Raney
                        if (wiggleBot) {
                            intakeSystem.setHSlidesInches(follower.followYourHeadTwo(target));
                            wiggle = true;
                        } else {
                            intakeSystem.setHSlidesInches(follower.followYourHead(target));
                        }
                    } else {
                        intakeSystem.setHSlidesInches(target.getY(DistanceUnit.INCH));
                        follower.followYourHeart(target.getX(DistanceUnit.INCH));
                    }
                    setPathState(2);
                } else {
                    telemetry.addData("","No Blocks Found");
                    telemetry.update();
                }
                break;

            case 2:
                //puts the wrist down after the slides are in the sub
                if (follower.getHeadingOffset() < Math.toRadians(1) && pathTimer.getElapsedTimeSeconds() > 2.5) { // hopefully this will activate when heading error small
                    follower.telemetryDebug(telemetry);
                    wiggle = false;
                    intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
                    setPathState(3);
                }
                break;
            case 3:
                // intakes and moves to store pos
                if (pathTimer.getElapsedTimeSeconds() > .2) {

                    if (intakeSystem.intakeUntil()) {
                        intakeSystem.storePos();
                        setPathState(4);
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
        Drawing.drawDebug(follower);
        autonomousPathUpdate();
        if (wiggle && pathTimer.getElapsedTimeSeconds() < .25) {driveTrain.drive(0,0.5, 0, DriveSpeedEnum.Auto);}
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}


