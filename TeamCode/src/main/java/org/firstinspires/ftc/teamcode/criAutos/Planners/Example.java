package org.firstinspires.ftc.teamcode.criAutos.Planners;

import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Path;
import org.firstinspires.ftc.teamcode.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.utility.Robot;


import static org.firstinspires.ftc.teamcode.criAutos.CommonPoses.*;
public class Example {
    public static class NAME implements PathPlanner {
        /// DESCRIPTION
        // Variables
        Pose offset = new Pose();
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        public NAME(Robot robot, Pose startPose) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
        }
        //Poses
        //todo
        Pose startPoseAdded;
        Pose redBasketAdded;

        //Paths
        Path toBasket;

        @Override
        public void buildPaths(Pose offset) {
            this.offset.add(offset);

            startPoseAdded = startPose.addReturn(offset);
            redBasketAdded = redBasket.addReturn(offset);

            toBasket = robot.follower.linearPathBuilder(startPoseAdded, redBasketAdded.addReturn(offset));
        }

        @Override
        public Pose getEndPoseEst() {
            //todo
            return new Pose();
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
            }

            return isFinished;
        }

        public void setPathState(int state) {
            this.state = state;
            pathTimer.resetTimer();
        }

        //todo
        public Pose getOffset() {
            return offset.addReturn(new Pose());
        }

        public void addToOffset(Pose offset) {
            this.offset = offset;
        }

        public String getName() {
            return "NAME";
        }
    }
}
