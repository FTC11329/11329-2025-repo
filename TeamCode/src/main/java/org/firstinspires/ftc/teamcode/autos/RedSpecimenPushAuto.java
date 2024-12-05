package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.FailoverAction;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSystem;
import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

@Config
@Autonomous(name = "Red Specimen Push", group = "Autonomous")
public class RedSpecimenPushAuto extends LinearOpMode {
    Pose2d initialPose = new Pose2d(8.5,-63,Math.toRadians(90));
    Vector2d bar = new Vector2d(4, -30.5);
    Vector2d medPos1 = new Vector2d(36, -48);
    Vector2d medPos2 = new Vector2d(37, -13);

    Vector2d block1     = new Vector2d(50, -13);
    Vector2d pushBlock1 = new Vector2d(50, -56);
    Vector2d block2     = new Vector2d(60, -13);
    Vector2d pushBlock2 = new Vector2d(60, -56);
    Vector2d block3     = new Vector2d(65, -13);
    Vector2d pushBlock3 = new Vector2d(49, -56);

    Vector2d pickupWall = new Vector2d(50, -60.5);
    Vector2d pickupWallBlock2 = new Vector2d(60, -60.5);
    //only do 2
    MecanumDrive drive;
    IntakeSystem intakeSystem;
    OuttakeSystem outtakeSystem;

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, initialPose);
        intakeSystem = new IntakeSystem(hardwareMap, RobotSideEnum.Red);
        outtakeSystem = new OuttakeSystem(hardwareMap);


        Action startToBar = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(bar, Math.toRadians(90))
                .waitSeconds(1)
                .build();

        Action barToPushBlock2 = drive.actionBuilder(bar)
                .setTangent(Math.toRadians(-45))
                .splineToConstantHeading(medPos1, Math.toRadians(0))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(medPos2, Math.toRadians(90))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(block1, Math.toRadians(0))
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(pushBlock1, Math.toRadians(-90))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(block1, Math.toRadians(90))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(block2, Math.toRadians(0))
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(pushBlock2, Math.toRadians(-90))
                .build();
        Action pushBlock2ToPickupWall = drive.actionBuilder(pushBlock2)
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(pickupWallBlock2, Math.toRadians(-90))
                .build();
        Action pickupWallToBar = drive.actionBuilder(pickupWallBlock2)
                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(bar, Math.toRadians(90))
                .build();
        Action barToPickupWall = drive.actionBuilder(bar)
                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(pickupWall, Math.toRadians(90))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                    new ParallelAction(
                        outtakeSystem.updateAction(),
                        new SequentialAction(
                            new ParallelAction(
                                    new SequentialAction(
                                            drive.waitSecondsAction(0.5),
                                            outtakeSystem.toSpecimen()
                                    ),
                                    startToBar
                            ),
                            outtakeSystem.pastSpecimen(),
                            new ParallelAction(
                                    new SequentialAction(
                                            drive.waitSecondsAction(0.25),
                                            outtakeSystem.drop(),
                                            drive.waitSecondsAction(0.3),
                                            outtakeSystem.toWallSpecimen()
                                    ),
                                    barToPushBlock2
                            ),
                            pushBlock2ToPickupWall,
                                //2nd cycle
                            outtakeSystem.grab(),
                            drive.waitSecondsAction(0.4),
                            outtakeSystem.toSpecimen(),
                            pickupWallToBar,
                            outtakeSystem.pastSpecimen(),
                            new ParallelAction(
                                    new SequentialAction(
                                            drive.waitSecondsAction(0.25),
                                            outtakeSystem.drop(),
                                            drive.waitSecondsAction(0.3),
                                            outtakeSystem.toWallSpecimen()
                                    ),
                                    barToPickupWall
                            )
                        )
                    ),
                    new ParallelAction(
                        outtakeSystem.updateAction(),
                        new SequentialAction(
                            //3rd cycle
                            outtakeSystem.grab(),
                            drive.waitSecondsAction(0.4),
                            outtakeSystem.toSpecimen(),
                            pickupWallToBar,
                            outtakeSystem.pastSpecimen(),
                            drive.waitSecondsAction(0.25),
                            outtakeSystem.drop()
                        )
                    )
                )
        );

    }
}
