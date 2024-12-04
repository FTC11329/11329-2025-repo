package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSystem;
import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

@Config
@Autonomous(name = "Red Specimen (no worky)", group = "Autonomous")
public class RedSpecimenAuto extends LinearOpMode {
    Pose2d initialPose = new Pose2d(8.5,-63,Math.toRadians(90));
    Vector2d pos1 = new Vector2d(4, -30.5);
    Pose2d pos2p1 = new Pose2d(30, -47, Math.toRadians(43));
    Pose2d pos2p2 = new Pose2d(40, -47, Math.toRadians(43));
    Pose2d pos2p3 = new Pose2d(50, -47, Math.toRadians(43));
    Pose2d pos3 = new Pose2d(27, -47, Math.toRadians(-31));
    Pose2d pos4 = new Pose2d(38, -60, Math.toRadians(90));
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
                .splineToConstantHeading(pos1, Math.toRadians(90))
                .waitSeconds(1)
                .build();

        Action path2p1 = drive.actionBuilder(new Pose2d(pos1.x, pos1.y, initialPose.heading.toDouble()))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(pos2p1, Math.toRadians(0))
                .build();
        Action path2p2 = drive.actionBuilder(new Pose2d(pos1.x, pos1.y, initialPose.heading.toDouble()))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(pos2p2, Math.toRadians(0))
                .build();
        Action path2p3 = drive.actionBuilder(new Pose2d(pos1.x, pos1.y, initialPose.heading.toDouble()))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(pos2p3, Math.toRadians(0))
                .build();

        Action path3p1 = drive.actionBuilder(pos2p1)
                .setTangent(0)
                .splineToLinearHeading(pos3, Math.toRadians(0))
                .build();
        Action path3p2 = drive.actionBuilder(pos2p2)
                .setTangent(0)
                .splineToLinearHeading(pos3, Math.toRadians(0))
                .build();
        Action path3p3 = drive.actionBuilder(pos2p3)
                .setTangent(0)
                .splineToLinearHeading(pos3, Math.toRadians(0))
                .build();

        Action path4 = drive.actionBuilder(pos3)
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(pos4, Math.toRadians(90))
                .build();

        Action path5 = drive.actionBuilder(pos4)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(pos1, Math.toRadians(90))
                .build();

        Action wait = drive.actionBuilder(pos2p1)
                .waitSeconds(1)
                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        outtakeSystem.toSpecimen(),
                        path1,
                        outtakeSystem.pastSpecimen(),
                        wait,
                        outtakeSystem.drop(),
                        path2p1,
                        outtakeSystem.toWallSpecimen(),
                        intakeSystem.extend(1600),
                        intakeSystem.intakeColor(),
                        path3p1,
                        intakeSystem.spit(),
                        intakeSystem.retract(),
                        path4,
                        outtakeSystem.toSpecimen(),
                        path5,
                        //run2
                        outtakeSystem.pastSpecimen(),
                        wait,
                        outtakeSystem.drop(),
                        path2p2,
                        outtakeSystem.toWallSpecimen(),
                        intakeSystem.extend(1600),
                        intakeSystem.intakeColor(),
                        path3p2,
                        intakeSystem.spit(),
                        intakeSystem.retract(),
                        path4,
                        outtakeSystem.toSpecimen(),
                        path5



                )
        );

    }
}
