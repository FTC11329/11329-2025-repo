package org.firstinspires.ftc.teamcode.criAutos.Planners;

import static org.firstinspires.ftc.teamcode.criAutos.CommonPoses.redBasket;

import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Path;
import org.firstinspires.ftc.teamcode.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.utility.Robot;

public class Hold {
    public static class HoldPos implements PathPlanner {
        /// DESCRIPTION
        // Variables
        Pose offset = new Pose();
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        public HoldPos(Robot robot, Pose startPose) {
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
        }

        @Override
        public Pose getEndPoseEst() {
            return startPose;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    robot.follower.holdPoint(startPoseAdded);
                    setPathState(1);
            }

            return isFinished;
        }

        public void setPathState(int state) {
            this.state = state;
            pathTimer.resetTimer();
        }

        public Pose getOffset() {
            return offset.addReturn(new Pose());
        }

        public void addToOffset(Pose offset) {
            this.offset = offset;
        }

        public String getName() {
            return "HOLD";
        }
    }
}
