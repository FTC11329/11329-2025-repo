package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@Config
@Autonomous(name = "testAuto", group = "Autonomous")
public class TestAuto extends LinearOpMode {
    Pose2d initialPose = new Pose2d(-30.5,-63.25,Math.toRadians(90));
    MecanumDrive drive;

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, initialPose);
        Action path1 = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-59.25, -36.5), Math.toRadians(180))
                .waitSeconds(2)
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-42.5, -42.5, Math.toRadians(45)), Math.toRadians(-45))
                .build();
        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        path1
                )
        );

    }
}
