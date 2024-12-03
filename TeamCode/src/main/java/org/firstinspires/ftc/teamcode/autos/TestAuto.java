package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.roadrunner.FailoverAction;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSystem;
import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

@Config
@Autonomous(name = "testAuto", group = "ZAutonomous")
public class TestAuto extends LinearOpMode {
    Pose2d initialPose = new Pose2d(16.5,-63,Math.toRadians(90));
    MecanumDrive drive;
    IntakeSystem intakeSystem;
    OuttakeSystem outtakeSystem;

    @Override
    public void runOpMode() {

        drive = new MecanumDrive(hardwareMap, initialPose);
        intakeSystem = new IntakeSystem(hardwareMap, RobotSideEnum.Red);
        outtakeSystem = new OuttakeSystem(hardwareMap);


        Action path1 = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-59.25, -36.5), Math.toRadians(180))
                .waitSeconds(2)
                .build();

        Action path2 = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-42.5, -42.5, Math.toRadians(45)), Math.toRadians(-45))
                .build();

        FailoverAction failAction = new FailoverAction(path1, intakeSystem.retract());
        Action extend = intakeSystem.extend();
        Action intake = intakeSystem.intakeColor(failAction);
        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        extend,
                        new ParallelAction(
                                failAction,
                                intake
                        ),
                        path2
                )
        );

    }
}
