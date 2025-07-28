package org.firstinspires.ftc.teamcode.criAutos;

import static org.firstinspires.ftc.teamcode.criAutos.CommonPoses.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.criAutos.Planners.*;
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

@Autonomous(name = "CRI RED Right Outer SPEC 2 HIGH", group = "   Comp", preselectTeleOp = "Tele-op Red")
public class RightOuterAutos extends OpMode {
    // todo How the robot is setup
    RobotSideEnum robotSide = RobotSideEnum.Red;
    //Wall or Low Spec
    PlacePosEnum startPos = PlacePosEnum.wall;
    Pose startPose = startRightOuter;



    // Outer Right High, Outer Right Low, Outer Right High, Park
    boolean auto1;
    // Outer Right High, Inner Right High, Inner Right Low, Inner Right High
    boolean auto2;
    // Outer Right High, Inner Right High, Inner Left High, Inner Right High
    boolean auto3;
    Pose totalOffset = new Pose();
    private Robot robot;
    private List<PathPlanner> steps;
    private int currentStep = 0;

    @Override
    public void init() {
        // Todo what auto you want to run

        // Outer Right High, Outer Right Low, Outer Right High, Park
        auto1 = false;
        // Outer Right High, Inner Right High, Inner Right Low, Outer Right Low
        auto2 = true;
        // Outer Right High, Baskets
        auto3 = false;

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
        //Outer Right High
        steps.add(new StartRightOuter.ToPlaceBarRightOuter(robot, lastPose(), true, new Pose(-1.5, 0)));

        Pose thisOffset;
        if (auto1) {
            thisOffset = new Pose(0, 1.5);
        } else {
            thisOffset = new Pose(0, 0);
        }
        if (!auto3) {
            steps.add(new FromBarRightOuter.ToWall(robot, lastPose(), true, true, thisOffset));
        }

        if (auto1) {
            //Outer Right Low
            steps.add(new FromWallRight.ToRightOuterBar(robot, lastPose(), false, new Pose(-1.5, 0)));

            steps.add(new FromBarRightOuter.ToWall(robot, lastPose(), true, true));

            //Outer Right High
            steps.add(new FromWallRight.ToRightOuterBar(robot, lastPose(), true));

            steps.add(new FromBarRightOuter.ToWall(robot, lastPose(), true, true, true));


        } else if (auto2) {
            //Inner Right High
            steps.add(new FromWallRight.ToRightInnerBar(robot, lastPose(), true, new Pose(0, 1)));

            steps.add(new FromBarRightInner.ToWall(robot, lastPose(), 1, true, new Pose(0, 1)));

            //Outer Right Low
            steps.add(new FromWallRight.ToRightOuterBar(robot, lastPose(), false, new Pose(0, -1)));

            steps.add(new FromBarRightOuter.ToWall(robot, lastPose(), true, true, new Pose(-1, 1.6)));

            //Inner Right Low
            steps.add(new FromWallRight.ToRightInnerBar(robot, lastPose(), false));

            steps.add(new FromBarRightInner.ToWall(robot, lastPose(), 2, true, true));

        } else if (auto3) {
            //Inner Right High
            steps.add(new FromBarRightOuter.ToBasket(robot, lastPose(), false));

            steps.add(new FromBlueBasket.ToPickupAndPlaceSubYellow(robot, lastPose()));
            steps.add(new FromBlueBasket.ToPickupAndPlaceSubYellow(robot, lastPose()));
            steps.add(new FromBlueBasket.ToPickupAndPlaceSubYellow(robot, lastPose()));
            steps.add(new FromBlueBasket.ToPickupAndPlaceSubYellow(robot, lastPose()));
            steps.add(new FromBlueBasket.ToPickupAndPlaceSubYellow(robot, lastPose()));
            steps.add(new FromBlueBasket.ToPickupAndPlaceSubYellow(robot, lastPose()));

        }

        if (auto3) {
            robot.blockVision.switchPipeline(0);
        } else {
            robot.blockVision.switchPipeline(robotSide);
        }
    }


    boolean secondInit = false;
    boolean buttonDebounce = false;
    int initState = 0;
    boolean visionWorks = false;

    public void init_loop() {
        if (secondInit) {
            telemetry.addLine("READY!");
            telemetry.update();
            return;
        }
        if (gamepad1.dpad_up && !buttonDebounce) {
            initState++;
            buttonDebounce = true;
        }
        if (gamepad1.dpad_down && !buttonDebounce) {
            initState--;
            buttonDebounce = true;
        }
        if (!gamepad1.dpad_up && !gamepad1.dpad_down) {
            buttonDebounce = false;
        }
        switch (initState) {
            case 0:
                telemetry.addLine("DOUBLE CHECK TIME");
                telemetry.addData("Robot Side", robotSide);
                telemetry.addData("Start Pose", startPose);
                if (auto1) {
                    telemetry.addData("Auto Ran  ", "Auto1");
                } else if (auto2) {
                    telemetry.addData("Auto Ran  ", "Auto2");
                } else if (auto3) {
                    telemetry.addData("Auto Ran  ", "Auto3");
                }
                if (!visionWorks && robot.blockVision.getBlockPosition().getHeading(AngleUnit.DEGREES) != -1) {
                    visionWorks = true;
                }
                telemetry.addData("Vision works", visionWorks);
                break;
            case 1:
                robot.follower.updatePose();
                telemetry.addLine("LOCALIZATION TEST");
                telemetry.addData("x", robot.follower.getPose().getX());
                telemetry.addData("y", robot.follower.getPose().getY());
                telemetry.addData("heading", Math.toDegrees(robot.follower.getPose().getHeading()));
                break;
            case 2:
                init();
                secondInit = true;
                break;
        }
        telemetry.update();
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