package org.firstinspires.ftc.teamcode.criAutos.Planners;

import static org.firstinspires.ftc.teamcode.criAutos.CommonPoses.*;

import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Path;
import org.firstinspires.ftc.teamcode.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.utility.Robot;

public class StartLeftInner {
    //todo paths
    public static class ToPlaceBar implements PathPlanner {
        /// Places Low bar left inner
        /// Ends facing left sub intake at store preset
        /// Expects arm to start under the bar
        // Variables
        boolean highBar;
        Pose offset;
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        public ToPlaceBar(Robot robot, Pose startPose, boolean highBar) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.highBar = highBar;
        }
        //Poses
        //todo
        Pose startLeftOuterAdded;
        Pose redBasketAdded;


        //Paths
        Path toBasket;

        @Override
        public void buildPaths(Pose offset) {
            this.offset = offset;

            startLeftOuterAdded = startLeftOuter.addReturn(offset);
            redBasketAdded = redBasket.addReturn(offset);

            toBasket = robot.follower.linearPathBuilder(startLeftOuterAdded, redBasketAdded);
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


        public String getName() {
            return "Start Left inner To place Bar" + state;
        }
    }
}
