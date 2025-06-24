package org.firstinspires.ftc.teamcode.criAutos.Planners;

import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Path;
import org.firstinspires.ftc.teamcode.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.utility.Robot;

public class TestPaths {
    public static class ToAboveRedBasket implements PathPlanner {
        // Variables
        Pose offset;
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
        }
        //Poses
        //todo
        private final Pose aboveRedBasket = new Pose(0, 96);

        //Paths
        Path toOpen;

        @Override
        public void buildPaths(Pose offset) {
            this.offset = offset;
            toOpen = robot.follower.linearPathBuilder(startPose, aboveRedBasket);
            toOpen.setConstantHeadingInterpolation(startPose.getHeading());
        }

        @Override
        public Pose getEndPose() {
            return aboveRedBasket;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    robot.follower.followPath(toOpen);
                    setState(1);
                    break;
                case 1:
                    if (robot.follower.getErrorDistance(aboveRedBasket) < 0.75) {
                        isFinished = true;
                    }
                    break;
            }

            return isFinished;
        }

        public void setState(int state) {
            this.state = state;
            pathTimer.resetTimer();
        }

        //todo
        public Pose getOffset() {
            return offset.addReturn(new Pose());
        }
    }

    public static class ToAboveBlueBasket implements PathPlanner {
        // Variables
        Pose offset;
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        public ToAboveBlueBasket(Robot robot, Pose startPose) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
        }
        //Poses
        private final Pose aboveBlueBasket = new Pose(0, -96);

        //Paths
        Path toOpen;

        @Override
        public void buildPaths(Pose offset) {
            this.offset = offset;
            toOpen = robot.follower.linearPathBuilder(startPose, aboveBlueBasket);
            toOpen.setConstantHeadingInterpolation(startPose.getHeading());
        }

        @Override
        public Pose getEndPose() {
            return aboveBlueBasket;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    robot.follower.followPath(toOpen);
                    setState(1);
                    break;
                case 1:
                    if (robot.follower.getErrorDistance(aboveBlueBasket) < 0.75) {
                        isFinished = true;
                    }
                    break;
            }

            return isFinished;
        }

        public void setState(int state) {
            this.state = state;
            pathTimer.resetTimer();
        }

        //todo
        public Pose getOffset() {
            return offset.addReturn(new Pose());
        }
    }

    public static class ToBelowLeftSub implements PathPlanner {
        // Variables
        Pose offset;
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        public ToBelowLeftSub(Robot robot, Pose startPose) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
        }
        //Poses
        private final Pose belowLeftSub = new Pose(-24, 24);

        //Paths
        Path toOpen;

        @Override
        public void buildPaths(Pose offset) {
            this.offset = offset;
            toOpen = robot.follower.linearPathBuilder(startPose, belowLeftSub);
            toOpen.setConstantHeadingInterpolation(startPose.getHeading());
        }

        @Override
        public Pose getEndPose() {
            return belowLeftSub;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    robot.follower.followPath(toOpen);
                    setState(1);
                    break;
                case 1:
                    if (robot.follower.getErrorDistance(belowLeftSub) < 0.75) {
                        isFinished = true;
                    }
                    break;
            }

            return isFinished;
        }

        public void setState(int state) {
            this.state = state;
            pathTimer.resetTimer();
        }

        //todo
        public Pose getOffset() {
            return offset.addReturn(new Pose());
        }
    }

    public static class ToBelowRightSub implements PathPlanner {
        // Variables
        Pose offset;
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        public ToBelowRightSub(Robot robot, Pose startPose) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
        }
        //Poses
        private final Pose belowRightSub = new Pose(-24, -24);

        //Paths
        Path toOpen;

        @Override
        public void buildPaths(Pose offset) {
            this.offset = offset;
            toOpen = robot.follower.linearPathBuilder(startPose, belowRightSub);
            toOpen.setConstantHeadingInterpolation(startPose.getHeading());
        }

        @Override
        public Pose getEndPose() {
            return belowRightSub;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    robot.follower.followPath(toOpen);
                    setState(1);
                    break;
                case 1:
                    if (robot.follower.getErrorDistance(belowRightSub) < 0.75) {
                        isFinished = true;
                    }
                    break;
            }

            return isFinished;
        }

        public void setState(int state) {
            this.state = state;
            pathTimer.resetTimer();
        }

        //todo
        public Pose getOffset() {
            return offset.addReturn(new Pose());
        }
    }

    public static class ToCenterField implements PathPlanner {
        // Variables
        Pose offset;
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        public ToCenterField(Robot robot, Pose startPose) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
        }
        //Poses
        private final Pose centerField = new Pose(0, 0);

        //Paths
        Path toOpen;

        @Override
        public void buildPaths(Pose offset) {
            this.offset = offset;
            toOpen = robot.follower.linearPathBuilder(startPose, centerField);
            toOpen.setConstantHeadingInterpolation(startPose.getHeading());
        }

        @Override
        public Pose getEndPose() {
            return centerField;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    robot.follower.followPath(toOpen);
                    setState(1);
                    break;
                case 1:
                    if (robot.follower.getErrorDistance(centerField) < 0.75) {
                        isFinished = true;
                    }
                    break;
            }

            return isFinished;
        }

        public void setState(int state) {
            this.state = state;
            pathTimer.resetTimer();
        }

        //todo
        public Pose getOffset() {
            return offset.addReturn(new Pose());
        }
    }


    public static class WaitSeconds implements PathPlanner {
        /// Waits the amount of time passed through in seconds
        // Variables
        Pose offset;
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        private double seconds;
        public WaitSeconds(Robot robot, Pose startPose, double seconds) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.seconds = seconds;
        }
        //Poses

        //Paths

        @Override
        public void buildPaths(Pose offset) {
            this.offset = offset;
        }

        @Override
        public Pose getEndPose() {
            return startPose;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    setPathState(1);
                    break;
                case 1:
                    if (pathTimer.getElapsedTimeSeconds() > seconds) {
                        isFinished = false;
                    }
                    break;
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
    }


}
