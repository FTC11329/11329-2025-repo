package org.firstinspires.ftc.teamcode.criAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.criAutos.Planners.FromBlueBasket;
import org.firstinspires.ftc.teamcode.criAutos.Planners.PathPlanner;
import org.firstinspires.ftc.teamcode.criAutos.Planners.StartLeft;
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

@Autonomous(name = "CRI Blue 1", group = " Comp", preselectTeleOp = "New Tele-op Blue")
public class BlueAuto1 extends OpMode {
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

        // Create robot context (with telemetry)
        robot = new Robot(climber, follower, telemetry, blockVision, driveTrain, powerTakeOff, intakeSystem, outtakeSystem);

        // Set start pose
        Pose startPose = new Pose();

        // Build step list
        steps = new ArrayList<>();
        steps.add(new StartLeft.ToPlaceBasket(robot, startPose));
        steps.add(new FromBlueBasket.ToPickupAndPlaceSpike1(robot, steps.get(0).getEndPose())); // Example
    }

    @Override
    public void loop() {
        if (currentStep >= steps.size()) {
            telemetry.addData("Status", "All steps completed");
            telemetry.update();
            return;
        }

        PathPlanner step = steps.get(currentStep);
        boolean done = step.run(robot); // Pass robot context

        if (done) {
            currentStep++;
        }
    }
}
