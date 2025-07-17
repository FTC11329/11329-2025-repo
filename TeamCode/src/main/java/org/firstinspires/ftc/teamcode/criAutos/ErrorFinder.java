package org.firstinspires.ftc.teamcode.criAutos;

import static org.firstinspires.ftc.teamcode.criAutos.CommonPoses.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.criAutos.Planners.*;
import org.firstinspires.ftc.teamcode.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedropathing.util.Drawing;
import org.firstinspires.ftc.teamcode.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.Attempt89;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.PowerTakeOff;
import org.firstinspires.ftc.teamcode.utility.PlacePosEnum;
import org.firstinspires.ftc.teamcode.utility.Robot;
import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;
import org.firstinspires.ftc.teamcode.utility.RobotStateVariables;
import org.firstinspires.ftc.teamcode.utility.StateMachine;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Error Finder", group = " Comp", preselectTeleOp = "New Tele-op Blue")
public class ErrorFinder extends OpMode {
    int finished = 0;
    // todo How the robot is setup
    RobotSideEnum robotSide = RobotSideEnum.Red;
    PlacePosEnum startPos = PlacePosEnum.wall;
    Pose startPose = startRightInner;
//    Pose endPose = new Pose(0, 0, Math.toRadians(0));
    Pose endPose = new Pose(-48, 0, Math.toRadians(0));
//    Pose endPose = new Pose(0, 96, Math.toRadians(0));
    Pose totalOffset = new Pose();
    Timer loopTimer = new Timer();
    private Robot robot;
    private List<PathPlanner> steps;
    private PathPlanner resetStep;
    private int currentStep = 0;

    @Override
    public void init() {
        // Initialize subsystems
        Climber climber = new Climber(hardwareMap);
        Follower follower = new Follower(hardwareMap);
        Attempt89 blockVision = new Attempt89(hardwareMap, robotSide);
        Drivetrain driveTrain = new Drivetrain(hardwareMap);
        PowerTakeOff powerTakeOff = new PowerTakeOff(hardwareMap);
        StateMachine stateMachine = new StateMachine();
        IntakeSystem intakeSystem = new IntakeSystem(hardwareMap, robotSide);
        OuttakeSystem outtakeSystem = new OuttakeSystem(hardwareMap, robotSide, true);

        RobotStateVariables robotState = new RobotStateVariables(startPos, robotSide);

        outtakeSystem.setArmPos(Constants.Outtake.initAutoNearWallArm);
        outtakeSystem.setWristPos(Constants.Outtake.initAutoNearWallWrist);

        follower.setStartingPose(startPose);


        // Create robot context
        robot = new Robot(climber, follower, telemetry, blockVision, driveTrain, powerTakeOff, intakeSystem, stateMachine, outtakeSystem, robotState, true);

        // todo: Build step list
        steps = new ArrayList<>();

        steps.add(new StartRightInner.ToRightInnerBar(robot, lastPose(), true));

        steps.add(new FromBarRightInner.ToWall(robot, lastPose(), 1, true));

        steps.add(new FromWallRight.ToRightInnerBar(robot, lastPose(), true));

        steps.add(new FromBarRightInner.ToWall(robot, lastPose(), 2, true));

        steps.add(new FromWallRight.ToRightInnerBar(robot, lastPose(), true));

        steps.add(new FromBarRightInner.ToWall(robot, lastPose(), 3, true));

//        steps.add(new TestPaths.ToPose(robot, lastPose(), endPose));

        resetStep = new TestPaths.ToPose(robot, endPose, startPose);
    }


    private Pose lastPose() {
        if (steps.isEmpty()) {
            return startPose;
        } else {
            return steps.get(steps.size() - 1).getEndPoseEst();
        }
    }

    @Override
    public void start() {
        robot.start();
        steps.get(currentStep).buildPaths(new Pose());
    }

    @Override
    public void loop() {
        switch (finished) {
            case 0:
                robot.follower.update();
                Drawing.drawDebug(robot.follower);

                robot.loop();

                PathPlanner step = steps.get(currentStep);
                boolean done = step.run();

                telemetry.addData("time", robot.opmodeTimer.getElapsedTimeSeconds());
                telemetry.addData("name", step.getName());
                telemetry.addData("stuck", robot.follower.isRobotStuck());
                telemetry.update();

                if (done) {
                    totalOffset = steps.get(currentStep).getOffset();
                    currentStep++;
                    if (currentStep >= steps.size()) {
                        telemetry.addData("Done", true);
                        telemetry.update();
                        finished = 1;
                        return;
                    }
                    steps.get(currentStep).buildPaths(totalOffset);
                }
                break;
            case 1:
                robot.follower.breakFollowing();
                robot.follower.update();
                finished = 2;
                break;
            case 2:
                robot.follower.update();
                Pose offsetCurrentPose = robot.follower.getPose().subtractReturn(totalOffset);
                Pose offsetTargetPose = endPose;
                Pose endOffset =  offsetTargetPose.subtractReturn(offsetCurrentPose);
                telemetry.addData("X", -endOffset.getX());
                telemetry.addData("Y", -endOffset.getY());
                telemetry.addLine();
                telemetry.addData("total offset X", totalOffset.getX());
                telemetry.addData("total offset Y", totalOffset.getY());
                telemetry.update();
                if (gamepad1.dpad_up) {
                    finished = 3;
                    resetStep.buildPaths(totalOffset);
                }
                break;
            case 3:
                robot.follower.update();
                Drawing.drawDebug(robot.follower);

                robot.loop();

                resetStep.run();
                break;

        }

    }
}