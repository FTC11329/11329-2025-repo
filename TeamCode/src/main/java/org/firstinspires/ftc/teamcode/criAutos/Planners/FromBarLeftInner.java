package org.firstinspires.ftc.teamcode.criAutos.Planners;

import static org.firstinspires.ftc.teamcode.criAutos.CommonPoses.*;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.PathBuilder;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.PathChain;
import org.firstinspires.ftc.teamcode.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.utility.Robot;

public class FromBarLeftInner {
    public static class ToWall implements PathPlanner {
        /// Option to push spike: 0 is none, 1 is bottom, 2 is middle, 3 is top
        /// Option to go left or right wall
        /// Expects robot.state.whereami to be correct
        /// Ends at left or right wall
        // Variables
        boolean leftWall;
        boolean park = false;
        int pushSpike;
        Pose offset = new Pose();
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        public ToWall(Robot robot, Pose startPose, int pushSpike, boolean leftWall, Pose offset) {
            addToOffset(offset);
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.leftWall = leftWall;
            this.pushSpike = pushSpike;
        }

        public ToWall(Robot robot, Pose startPose, int pushSpike, boolean leftWall) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.leftWall = leftWall;
            this.pushSpike = pushSpike;
        }

        public ToWall(Robot robot, Pose startPose, int pushSpike, boolean leftWall, boolean park) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.leftWall = leftWall;
            this.pushSpike = pushSpike;
            this.park = park;
        }


        //Poses
        Pose startPoseRightAdded;
        Pose wallAdded;
        Pose innerSpikeLeftBotAdded;
        Pose targetPoseAdded;
        Pose openAdded;

        //Paths
        PathBuilder toWall;
        PathChain toWallPath;

        @Override
        public void buildPaths(Pose offset) {
            this.offset.add(offset);

            startPoseRightAdded = new Pose(startPose.getX(), innerSpikeLeftMid.getY(), Math.toRadians(90)).addReturn(offset);
            innerSpikeLeftBotAdded = innerSpikeLeftBot.addReturn(offset);
            if (leftWall) {
                wallAdded = pickupWallLeft.addReturn(offset);
            } else {
                wallAdded = pickupWallRight.addReturn(new Pose(-0.5, 0)).addReturn(offset);
            }
            openAdded = new Pose(-48, innerSpikeLeftMid.getY(), Math.toRadians(0)).addReturn(offset);

            switch (pushSpike) {
                case 0:
                    targetPoseAdded = new Pose(-36, innerSpikeLeftMid.getY(), Math.toRadians(90)).addReturn(offset);
                    break;
                case 1:
                    targetPoseAdded = innerSpikeLeftBot.addReturn(offset);
                    break;
                case 2:
                    targetPoseAdded = innerSpikeLeftMid.addReturn(offset);
                    break;
                case 3:
                    targetPoseAdded = innerSpikeLeftTop.addReturn(offset);
                    break;
            }
            toWall = robot.follower.pathBuilder()
                    .addPath(robot.follower.linearPathBuilder(startPoseRightAdded, innerSpikeLeftBotAdded));
            if (leftWall) {
                toWall.addPath(robot.follower.linearPathBuilder(innerSpikeLeftBotAdded, wallAdded, 0.85));
            } else {
                toWall.addPath(robot.follower.linearPathBuilder(innerSpikeLeftBotAdded, openAdded))
                        .addPath(robot.follower.linearPathBuilder(openAdded, wallAdded));
            }


            toWallPath = toWall.build();
        }

        @Override
        public Pose getEndPoseEst() {
            if (leftWall) {
                return pickupWallLeft;
            } else {
                return pickupWallRight;
            }
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    robot.follower.setMaxPower(0.9);
                    robot.follower.followPath(toWallPath);
                    robot.stateMachine.goWall(false, false, false);
                    robot.outtakeSystem.setFlapsSpikeClear();
                    setPathState(1);
                    break;
                case 1:
                    if (robot.follower.getError(targetPoseAdded).getX() < 1) {
                        robot.outtakeSystem.setFlapsWall();
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (robot.follower.getError(openAdded).getX() < 1) {
                        robot.follower.setMaxPower(1);
                        if (park) {
                            robot.outtakeSystem.setArmPos(Constants.Outtake.upArm);
                        }
                        setPathState(3);
                    }
                    break;
                case 3:
                    if (robot.outtakeSystem.seesWall() || pathTimer.getElapsedTimeSeconds() > 2) {
                        setPathState(4);
                    }
                    break;
                case 4:
                    if (pathTimer.getElapsedTimeSeconds() > 0.05) {
                        robot.outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                        setPathState(5);
                    }
                    break;
                case 5:
                    if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                        isFinished = true;
                        setPathState(6);
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
            if (leftWall) {
                return offset.addReturn(new Pose(-0.3, 0.9));
            } else {
                return offset.addReturn(new Pose(-0.3, 0.75));
            }
        }

        public String getName() {
            return "From Bar Left Inner To Wall, " + state;
        }
    }
}
