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
import org.firstinspires.ftc.teamcode.utility.Robot;
import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "CRI Blue Samples", group = " Comp", preselectTeleOp = "New Tele-op Blue")
public class BlueAutoSamples extends OpMode {
    private Robot robot;
    private List<PathPlanner> steps;
    private int currentStep = 0;

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

        outtakeSystem.setArmPos(Constants.Outtake.initAutoSpecArm);

        // Create robot context
        robot = new Robot(climber, follower, telemetry, blockVision, driveTrain, powerTakeOff, intakeSystem, outtakeSystem);

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
            currentStep++;
        }
    }
}
