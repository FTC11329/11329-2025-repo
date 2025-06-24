package org.firstinspires.ftc.teamcode.criAutos;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.criAutos.Planners.PathPlanner;
import org.firstinspires.ftc.teamcode.criAutos.Planners.StartLeftOuter;
import org.firstinspires.ftc.teamcode.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.Attempt89;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.PowerTakeOff;
import org.firstinspires.ftc.teamcode.utility.Robot;
import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

@Autonomous(name = "ErrorFinder", group = "Comp")
public class ErrorFinder extends OpMode {
    private Robot robot;
    private PathPlanner step;

    Timer timer = new Timer();
    private int state = 0;

    @Override
    public void init() {
        // Initialize subsystems
        Climber climber = new Climber(hardwareMap);
        Follower follower = new Follower(hardwareMap);
        Attempt89 blockVision = new Attempt89(hardwareMap, RobotSideEnum.Blue);
        Drivetrain driveTrain = new Drivetrain(hardwareMap);
        PowerTakeOff powerTakeOff = new PowerTakeOff(hardwareMap);
        IntakeSystem intakeSystem = new IntakeSystem(hardwareMap, RobotSideEnum.Blue);
        OuttakeSystem outtakeSystem = new OuttakeSystem(hardwareMap, RobotSideEnum.Blue, true);

        outtakeSystem.setArmPos(Constants.Outtake.initAutoNearWallArm);

        // Create robot context
        robot = new Robot(climber, follower, telemetry, blockVision, driveTrain, powerTakeOff, intakeSystem, outtakeSystem);

        // Set start pose
        Pose startPose = CommonPoses.startLeftOuter;

        // step to test
        step = new StartLeftOuter.ToPlaceBasket(robot, startPose, true);
    }

    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void loop() {
        switch (state) {
            case 0:
                if (step.run()) {
                    robot.follower.turnTo(Math.toRadians(90));
                    timer.resetTimer();
                    state = 1;
                }
                break;
            case 1:
                if (timer.getElapsedTimeSeconds() > 1) {
                    robot.follower.breakFollowing();
                    state = 2;
                }
                break;
            case 2:
                telemetry.addData("Done", true);
                telemetry.addLine("Actual Pose");
                Pose actPose = robot.follower.getPose();
                telemetry.addData("X", actPose.getX());
                telemetry.addData("Y", actPose.getY());
                telemetry.addData("Heading", actPose.getHeading());
                telemetry.addLine("Target Pose");
                Pose tarPose = step.getEndPose();
                telemetry.addData("X", tarPose.getX());
                telemetry.addData("Y", tarPose.getY());
                telemetry.addData("Heading", tarPose.getHeading());
                telemetry.update();
        }

    }

}
