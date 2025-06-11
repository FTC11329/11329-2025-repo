package org.firstinspires.ftc.teamcode.criAutos.Planners;

import org.firstinspires.ftc.teamcode.criAutos.CommonPoses;
import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Path;
import org.firstinspires.ftc.teamcode.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.utility.Robot;

public class TestPathPlanners {
    public static class ToAboveRedBasket implements PathPlanner {
        // Variables
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        public ToAboveRedBasket(Robot robot, Pose startPose) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            buildPaths();
        }

        //Paths
        Path toOpen;
        public void buildPaths() {
            toOpen = robot.follower.linearPathBuilder(startPose, new Pose(0, 96));
        }

        @Override
        public Pose getEndPose() {
            return new Pose();
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
            }

            return isFinished;
        }

        public void setState(int state) {
            this.state = state;
            pathTimer.resetTimer();
        }
    }
}
