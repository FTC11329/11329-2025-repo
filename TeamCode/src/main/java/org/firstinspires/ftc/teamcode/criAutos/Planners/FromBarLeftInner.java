package org.firstinspires.ftc.teamcode.criAutos.Planners;

import static org.firstinspires.ftc.teamcode.criAutos.CommonPoses.*;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.PathBuilder;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.PathChain;
import org.firstinspires.ftc.teamcode.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.utility.PlacePosEnum;
import org.firstinspires.ftc.teamcode.utility.Robot;

public class FromBarLeftInner {
    public static class ToWall implements PathPlanner {
        /// Option to push spike 0 is none, 1 is bottom, 2 is middle, 3 is top
        /// Option to go left or right wall
        /// Expects robot.state.whereami to be correct
        /// Ends at left or right wall
        // Variables
        boolean leftWall;
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


        //Poses
        Pose startPoseRightAdded;
        Pose wallAdded;
        Pose targetPoseAdded;
        Pose openAdded;

        //Paths
        PathBuilder toWall;
        PathChain toWallPath;

        @Override
        public void buildPaths(Pose offset) {
            this.offset.add(offset);

            startPoseRightAdded = new Pose(startPose.getX(), innerSpikeLeftMid.getY(), Math.toRadians(90)).addReturn(offset);
            if (leftWall) {
                wallAdded = pickupWallLeft.addReturn(offset);
            } else {
                wallAdded = pickupWallRight.addReturn(offset);
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
                    .addPath(robot.follower.linearPathBuilder(startPoseRightAdded, targetPoseAdded));
            if (leftWall) {
                toWall.addPath(robot.follower.linearPathBuilder(targetPoseAdded, wallAdded, 0.85));
            } else {
                toWall.addPath(robot.follower.linearPathBuilder(targetPoseAdded, openAdded))
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
                    robot.follower.followPath(toWallPath);
                    robot.stateMachine.goWall(false, robot.robotState.whereAmI == PlacePosEnum.lowSpecimen, false);
                    setPathState(1);
                    break;
                case 1:
                    if (robot.follower.getErrorDistance(targetPoseAdded) < 1) {
                        robot.outtakeSystem.setRightFlap(Constants.Outtake.rightFlapSpike);
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (robot.follower.getError(openAdded).getX() < 1) {
                        robot.outtakeSystem.setRightFlap(Constants.Outtake.rightFlapWall);
                        setPathState(3);
                    }
                    break;
                case 3:
                    if ((robot.outtakeSystem.seesWall() && pathTimer.getElapsedTimeSeconds() > 0.7) || pathTimer.getElapsedTimeSeconds() > 2) {
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
                return offset.addReturn(new Pose(-1.2, -0.25));
            } else {
                return offset.addReturn(new Pose(-1.2, -0.25));
            }
        }

        public String getName() {
            return "From Bar Left Inner To Wall, " + state;
        }
    }
}
