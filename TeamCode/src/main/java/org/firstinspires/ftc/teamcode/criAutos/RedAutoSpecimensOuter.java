package org.firstinspires.ftc.teamcode.criAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.criAutos.Planners.*;

import static org.firstinspires.ftc.teamcode.criAutos.CommonPoses.*;

import org.firstinspires.ftc.teamcode.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedropathing.util.Drawing;
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

@Autonomous(name = "CRI Red Auto Specimens Outer", group = " Comp", preselectTeleOp = "New Tele-op Blue")
public class RedAutoSpecimensOuter extends OpMode {
    // todo How the robot is setup
    RobotSideEnum robotSide = RobotSideEnum.Red;
    //Wall or Low Spec
    PlacePosEnum startPos = PlacePosEnum.wall;
    Pose startPose = startRightInner;


    Pose totalOffset = new Pose();
    private Robot robot;
    private List<PathPlanner> steps;
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

//      remove me if we dont change low spec
//        if (startPos == PlacePosEnum.wall) {
            outtakeSystem.setArmPos(Constants.Outtake.initAutoNearWallArm);
            outtakeSystem.setWristPos(Constants.Outtake.initAutoNearWallWrist);
//        } else if (startPos == PlacePosEnum.lowSpecimen) {
//            outtakeSystem.setArmPos(Constants.Outtake.initAutoUnderBarArm);
//            outtakeSystem.setWristPos(Constants.Outtake.initAutoUnderBarWrist);
//        }

        follower.setStartingPose(startPose);

        // Create robot context
        robot = new Robot(climber, follower, telemetry, blockVision, driveTrain, powerTakeOff, intakeSystem, stateMachine, outtakeSystem, robotState, true);

        // todo: Build step list
        steps = new ArrayList<>();
        steps.add(new StartLeftOuter.ToPlaceLeftOuterBar(robot, lastPose(), true));

        steps.add(new FromBarLeftOuter.ToWall(robot, lastPose(), true, true));

        steps.add(new FromWallLeft.ToLeftOuterBar(robot, lastPose(), false));

        steps.add(new FromBarLeftOuter.ToWall(robot, lastPose(), true, true, new Pose(0, -1)));

        steps.add(new FromWallLeft.ToLeftOuterBar(robot, lastPose(), true));

        steps.add(new FromBarLeftOuter.ToWall(robot, lastPose(), true, true));

        steps.add(new FromWallLeft.ToLeftOuterBar(robot, lastPose(), true));

        steps.add(new FromBarLeftOuter.ToWall(robot, lastPose(), true, true));

        steps.add(new FromWallLeft.ToLeftOuterBar(robot, lastPose(), true, new Pose(0, -1)));

        steps.add(new StartLeftInner.ToLeftInnerBar(robot, lastPose(), true));

        steps.add(new FromBarLeftInner.ToWall(robot, lastPose(), 1, true));

        steps.add(new FromWallLeft.ToLeftInnerBar(robot, lastPose(), false));

        steps.add(new FromBarLeftInner.ToWall(robot, lastPose(), 2, false, new Pose(0, -1)));

        steps.add(new FromWallRight.ToRightInnerBar(robot, lastPose(), true));

        steps.add(new FromBarRightInner.ToWall(robot, lastPose(), 1, true));

        steps.add(new FromWallRight.ToRightInnerBar(robot, lastPose(), false));
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
        robot.follower.update();
        Drawing.drawDebug(robot.follower);
        // Stops the robot if done
        if (currentStep >= steps.size()) {
            telemetry.addData("Done", true);
            telemetry.update();
            return;
        }

        robot.loop();

        PathPlanner step = steps.get(currentStep);
        boolean done = step.run();

        telemetry.addData("time", robot.opmodeTimer.getElapsedTimeSeconds());
        telemetry.addData("offset", totalOffset);
        telemetry.addData("velocity", robot.follower.getVelocity());
        telemetry.addData("name", step.getName());

        telemetry.update();

        if (done) {
            totalOffset = steps.get(currentStep).getOffset();
            currentStep++;
            if (currentStep >= steps.size()) {
                telemetry.addData("Done", true);
                telemetry.update();
                return;
            }
            steps.get(currentStep).buildPaths(totalOffset);
        }
    }
}