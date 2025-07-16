package org.firstinspires.ftc.teamcode.criAutos.Planners;

import static org.firstinspires.ftc.teamcode.criAutos.CommonPoses.*;

import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Path;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.PathChain;
import org.firstinspires.ftc.teamcode.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.utility.Robot;

public class TestPaths {
    public static class ToStartRightOuter implements PathPlanner {
        // Variables
        Pose offset = new Pose();
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        public ToStartRightOuter(Robot robot, Pose startPose, Pose offset) {
            addToOffset(offset);
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
        }

        public ToStartRightOuter(Robot robot, Pose startPose) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
        }
        //Poses

        //Paths
        Path toOpen;

        @Override
        public void buildPaths(Pose offset) {
            this.offset.add(offset);
            toOpen = robot.follower.linearPathBuilder(startPose, startRightOuter.addReturn(offset));
            toOpen.setConstantHeadingInterpolation(startPose.getHeading());
        }

        @Override
        public Pose getEndPoseEst() {
            return startRightOuter;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    robot.follower.followPath(toOpen);
                    setState(1);
                    break;
                case 1:
                    if (robot.follower.getErrorDistance(startRightOuter.addReturn(offset)) < 0.75) {
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
        public void addToOffset(Pose offset) {
            this.offset = offset;
        }

        public Pose getOffset() {
            return offset.addReturn(new Pose());
        }

        public String getName() {
            return "Test Path" + state;
        }
    }
    public static class ToAboveRedBasket implements PathPlanner {
        // Variables
        Pose offset = new Pose();
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        public ToAboveRedBasket(Robot robot, Pose startPose, Pose offset) {
            addToOffset(offset);
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
        }
        public ToAboveRedBasket(Robot robot, Pose startPose) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
        }
        //Poses
        private final Pose aboveRedBasket = new Pose(0, 96);

        //Paths
        Path toOpen;

        @Override
        public void buildPaths(Pose offset) {
            this.offset.add(offset);
            toOpen = robot.follower.linearPathBuilder(startPose, aboveRedBasket.addReturn(offset));
            toOpen.setConstantHeadingInterpolation(startPose.getHeading());
        }

        @Override
        public Pose getEndPoseEst() {
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
                    if (robot.follower.getErrorDistance(aboveRedBasket.addReturn(offset)) < 0.75) {
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
        public void addToOffset(Pose offset) {
            this.offset = offset;
        }

        public Pose getOffset() {
            return offset.addReturn(new Pose());
        }

        public String getName() {
            return "Test Path" + state;
        }
    }

    public static class ToAboveBlueBasket implements PathPlanner {
        // Variables
        Pose offset = new Pose();
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        public ToAboveBlueBasket(Robot robot, Pose startPose, Pose offset) {
            addToOffset(offset);
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
        }
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
            this.offset.add(offset);
            toOpen = robot.follower.linearPathBuilder(startPose, aboveBlueBasket.addReturn(offset));
            toOpen.setConstantHeadingInterpolation(startPose.getHeading());
        }

        @Override
        public Pose getEndPoseEst() {
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
                    if (robot.follower.getErrorDistance(aboveBlueBasket.addReturn(offset)) < 0.75) {
                        setState(2);
                    }
                    break;
                case 2:
                    if (pathTimer.getElapsedTimeSeconds() > 1) {
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
        public void addToOffset(Pose offset) {
            this.offset = offset;
        }

        public Pose getOffset() {
            return offset.addReturn(new Pose());
        }

        public String getName() {
            return "Test Path" + state;
        }
    }

    public static class ToBelowLeftSub implements PathPlanner {
        // Variables
        Pose offset = new Pose();
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        public ToBelowLeftSub(Robot robot, Pose startPose, Pose offset) {
            addToOffset(offset);
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
        }
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
            this.offset.add(offset);
            toOpen = robot.follower.linearPathBuilder(startPose, belowLeftSub.addReturn(offset));
            toOpen.setConstantHeadingInterpolation(startPose.getHeading());
        }

        @Override
        public Pose getEndPoseEst() {
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
                    if (robot.follower.getErrorDistance(belowLeftSub.addReturn(offset)) < 0.75) {
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
        public void addToOffset(Pose offset) {
            this.offset = offset;
        }

        public Pose getOffset() {
            return offset.addReturn(new Pose());
        }

        public String getName() {
            return "Test Path" + state;
        }
    }

    public static class ToBelowRightSub implements PathPlanner {
        // Variables
        Pose offset = new Pose();
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;

        public ToBelowRightSub(Robot robot, Pose startPose, Pose offset) {
            addToOffset(offset);
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
        }
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
            this.offset.add(offset);
            toOpen = robot.follower.linearPathBuilder(startPose, belowRightSub.addReturn(offset));
            toOpen.setConstantHeadingInterpolation(startPose.getHeading());
        }

        @Override
        public Pose getEndPoseEst() {
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
                    if (robot.follower.getErrorDistance(belowRightSub.addReturn(offset)) < 0.75) {
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
        public void addToOffset(Pose offset) {
            this.offset = offset;
        }

        public Pose getOffset() {
            return offset.addReturn(new Pose());
        }

        public String getName() {
            return "Test Path" + state;
        }
    }

    public static class ToCenterField implements PathPlanner {
        // Variables
        Pose offset = new Pose();
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;

        public ToCenterField(Robot robot, Pose startPose, Pose offset) {
            addToOffset(offset);
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
        }
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
            this.offset.add(offset);
            toOpen = robot.follower.linearPathBuilder(startPose, centerField.addReturn(offset));
            toOpen.setConstantHeadingInterpolation(startPose.getHeading());
        }

        @Override
        public Pose getEndPoseEst() {
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
                    if (robot.follower.getErrorDistance(centerField.addReturn(offset)) < 0.75) {
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

        public void addToOffset(Pose offset) {
            this.offset = offset;
        }

        public Pose getOffset() {
            return offset.addReturn(new Pose());
        }

        public String getName() {
            return "Test Path" + state;
        }
    }

    public static class ToPose implements PathPlanner {
        // Variables
        Pose offset = new Pose();
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        private Pose endPose;
        public ToPose(Robot robot, Pose startPose, Pose endPose, Pose offset) {
            addToOffset(offset);
            pathTimer = new Timer();
            this.robot = robot;
            this.endPose = endPose;
            this.startPose = startPose;
        }
        public ToPose(Robot robot, Pose startPose, Pose endPose) {
            pathTimer = new Timer();
            this.robot = robot;
            this.endPose = endPose;
            this.startPose = startPose;
        }
        //Poses
        Pose endPoseAdded;
        Pose startPoseAdded;

        //Paths
        PathChain toOpen;

        @Override
        public void buildPaths(Pose offset) {
            this.offset.add(offset);
            endPoseAdded = endPose.addReturn(offset);
            startPoseAdded = startPose.addReturn(offset);
            toOpen = robot.follower.linearPathChainBuilder(startPoseAdded, endPoseAdded);
            if (endPoseAdded.getHeading() == 0) {
                toOpen.getPath(0).setConstantHeadingInterpolation(startPoseAdded.getHeading());
            }
        }

        @Override
        public Pose getEndPoseEst() {
            return endPoseAdded;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    robot.follower.followPath(toOpen);
                    setState(1);
                    break;
                case 1:
                    if (robot.follower.getErrorDistance(endPoseAdded) < 0.75) {
                        setState(2);
                    }
                    break;
                case 2:
                    if (pathTimer.getElapsedTimeSeconds() > 1.5) {
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

        public void addToOffset(Pose offset) {
            this.offset = offset;
        }

        public Pose getOffset() {
            return offset.addReturn(new Pose());
        }

        public String getName() {
            return "Test Path" + state;
        }
    }


    public static class WaitSeconds implements PathPlanner {
        /// Waits the amount of time passed through in seconds
        // Variables
        Pose offset = new Pose();
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        private double seconds;
        public WaitSeconds(Robot robot, Pose startPose, double seconds, Pose offset) {
            addToOffset(offset);
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.seconds = seconds;
        }

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
            this.offset.add(offset);
        }

        @Override
        public Pose getEndPoseEst() {
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
                        isFinished = true;
                    }
                    break;
            }

            return isFinished;
        }

        public void setPathState(int state) {
            this.state = state;
            pathTimer.resetTimer();
        }

        public void addToOffset(Pose offset) {
            this.offset = offset;
        }

        public Pose getOffset() {
            return offset.addReturn(new Pose());
        }

        public String getName() {
            return "Test Path" + state;
        }
    }


}
