package org.firstinspires.ftc.teamcode.criAutos.Planners;

import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.utility.Robot;

public class StartLeft {
    public static class ToPlaceBasket implements PathPlanner {
        private final Robot robot;

        private final Pose startPose;

        private int state = 0;

        private boolean isFinished = false;

        public ToPlaceBasket(Robot robot, Pose startPose) {
            this.robot = robot;
            this.startPose = startPose;
            buildPaths();
        }

        public void buildPaths() {
            // Build trajectory or whatever needed
        }

        @Override
        public Pose getEndPose() {
            return new Pose(); // Replace with real final pose
        }

        @Override
        public boolean run(Robot robot) {
            switch (state) {
                case 0:
                    robot.follower.followPath();
            }

            return isFinished;
        }
    }
}
