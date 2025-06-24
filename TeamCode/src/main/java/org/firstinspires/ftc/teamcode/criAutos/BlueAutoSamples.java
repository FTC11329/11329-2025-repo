package org.firstinspires.ftc.teamcode.criAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.criAutos.Planners.FromRedBasket;
import org.firstinspires.ftc.teamcode.criAutos.Planners.PathPlanner;
import org.firstinspires.ftc.teamcode.criAutos.Planners.StartLeftOuter;
import org.firstinspires.ftc.teamcode.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
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

@Autonomous(name = "CRI Blue Samples", group = "Comp", preselectTeleOp = "New Tele-op Blue")
public class BlueAutoSamples extends OpMode {
    // todo WHERE DO YOU START WITH THE ARM AND WHAT SIDE
    RobotSideEnum robotSide = RobotSideEnum.Blue;
    PlacePosEnum startPos = PlacePosEnum.wall;
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

        RobotStateVariables robotState = new RobotStateVariables(startPos);

        outtakeSystem.setArmPos(Constants.Outtake.initAutoNearWallArm);

        // Create robot context
        robot = new Robot(climber, follower, telemetry, blockVision, driveTrain, powerTakeOff, intakeSystem, stateMachine, outtakeSystem, robotState, true);

        // Set start pose
        Pose startPose = new Pose();

        // Build step list
        steps = new ArrayList<>();
        steps.add(new StartLeftOuter.ToPlaceBasket(robot, startPose, true));
        steps.add(new FromRedBasket.ToPickupAndPlaceSpike1(robot, lastPose(), true));
        steps.add(new FromRedBasket.ToPickupAndPlaceSpike2(robot, lastPose(), true));
        steps.add(new FromRedBasket.ToPickupAndPlaceSpike3(robot, lastPose()));
        steps.add(new FromRedBasket.ToPickupAndPlaceSubYellow(robot, lastPose()));
        steps.add(new FromRedBasket.ToPickupAndPlaceSubYellow(robot, lastPose()));
        steps.add(new FromRedBasket.ToPickupAndPlaceSubYellow(robot, lastPose()));
        steps.add(new FromRedBasket.ToPickupAndPlaceSubYellow(robot, lastPose()));
    }

    private Pose lastPose() {
        return steps.get(steps.size() - 1).getEndPose();
    }

    @Override
    public void start() {
        robot.start();
        steps.get(currentStep).buildPaths(new Pose());
    }

    @Override
    public void loop() {
        robot.loop();
        if (currentStep >= steps.size()) {
            telemetry.addData("Done", true);
            telemetry.update();
            return;
        }

        PathPlanner step = steps.get(currentStep);
        boolean done = step.run();

        if (done) {
            Pose totalOffset = steps.get(currentStep).getOffset();
            currentStep++;
            steps.get(currentStep).buildPaths(totalOffset);
        }
    }
}
