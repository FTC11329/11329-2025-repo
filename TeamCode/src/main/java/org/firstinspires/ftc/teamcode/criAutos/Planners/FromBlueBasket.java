package org.firstinspires.ftc.teamcode.criAutos.Planners;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Attempt89;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.PowerTakeOff;
import org.firstinspires.ftc.teamcode.utility.Robot;

public class FromBlueBasket {
    public static class ToPickupAndPlaceSpike1 extends PathPlanner {
        volatile Climber climber;
        volatile Follower follower;
        volatile Telemetry telemetry;
        volatile Attempt89 blockVision;
        volatile Drivetrain driveTrain;
        volatile PowerTakeOff powerTakeOff;
        volatile IntakeSystem intakeSystem;
        volatile OuttakeSystem outtakeSystem;

        Pose startPose;

        //Poses

        //Variables

        public ToPickupAndPlaceSpike1(Robot robot, Pose startPose) {
            this.climber       = robot.climber;
            this.follower      = robot.follower;
            this.telemetry     = robot.telemetry;
            this.driveTrain    = robot.driveTrain;
            this.blockVision   = robot.blockVision;
            this.powerTakeOff  = robot.powerTakeOff;
            this.intakeSystem  = robot.intakeSystem;
            this.outtakeSystem = robot.outtakeSystem;

            this.startPose = startPose;

            buildPaths();
        }

        public void buildPaths() {

        }

        @Override
        public Pose getEndPose() {
            return null;
        }

        @Override
        public int getEndCase() {
            return 0;
        }

        @Override
        public void run(int x) {
            int state = x - startCase;
            switch (state) {
                case 0:

            }
        }
    }
}
