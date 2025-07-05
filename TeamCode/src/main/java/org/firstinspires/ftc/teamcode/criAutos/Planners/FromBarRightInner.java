package org.firstinspires.ftc.teamcode.criAutos.Planners;

import static org.firstinspires.ftc.teamcode.criAutos.CommonPoses.*;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.PathBuilder;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.PathChain;
import org.firstinspires.ftc.teamcode.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.utility.PlacePosEnum;
import org.firstinspires.ftc.teamcode.utility.Robot;

public class FromBarRightInner {
    public static class ToWall implements PathPlanner {
        /// Option to push spike 0 is none, 1 is bottom, 2 is middle, 3 is top
        /// Option to go left or right wall
        /// Expects robot.state.whereami to be correct
        /// Ends at left or right wall
        // Variables
        boolean rightWall;
        int pushSpike;
        Pose offset = new Pose();
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        public ToWall(Robot robot, Pose startPose, int pushSpike, boolean rightWall, Pose offset) {
            addToOffset(offset);
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.rightWall = rightWall;
            this.pushSpike = pushSpike;
        }

        public ToWall(Robot robot, Pose startPose, int pushSpike, boolean rightWall) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.rightWall = rightWall;
            this.pushSpike = pushSpike;
        }


        //Poses
        Pose startPoseLeftAdded;
        Pose wallAdded;
        Pose targetPoseAdded;
        Pose openAdded;

        //Paths
        PathBuilder toWall;
        PathChain toWallPath;

        @Override
        public void buildPaths(Pose offset) {
            this.offset.add(offset);

            startPoseLeftAdded = new Pose(startPose.getX(), innerSpikeRightMid.getY(), Math.toRadians(-90)).addReturn(offset);
            if (rightWall) {
                wallAdded = pickupWallRight.addReturn(offset);
            } else {
                wallAdded = pickupWallLeft.addReturn(offset);
            }
            openAdded = new Pose(-48, innerSpikeRightMid.getY(), Math.toRadians(0)).addReturn(offset);



            switch (pushSpike) {
                case 0:
                    targetPoseAdded = new Pose(-36, innerSpikeRightMid.getY(), Math.toRadians(-90)).addReturn(offset);
                    break;
                case 1:
                    targetPoseAdded = innerSpikeRightBot.addReturn(offset);
                    break;
                case 2:
                    targetPoseAdded = innerSpikeRightMid.addReturn(offset);
                    break;
                case 3:
                    targetPoseAdded = innerSpikeRightTop.addReturn(offset);
                    break;
            }
            toWall = robot.follower.pathBuilder()
                    .addPath(robot.follower.linearPathBuilder(startPoseLeftAdded, targetPoseAdded));
            if (rightWall) {
                toWall.addPath(robot.follower.linearPathBuilder(targetPoseAdded, wallAdded, 0.85));
            } else {
                toWall.addPath(robot.follower.linearPathBuilder(targetPoseAdded, openAdded))
                        .addPath(robot.follower.linearPathBuilder(openAdded, wallAdded));
            }

            toWallPath = toWall.build();


        }

        @Override
        public Pose getEndPoseEst() {
            if (rightWall) {
                return pickupWallRight;
            } else {
                return pickupWallLeft;
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
                        robot.outtakeSystem.setLeftFlap(Constants.Outtake.leftFlapSpike);
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (robot.follower.getError(openAdded).getX() < 1) {
                        robot.outtakeSystem.setLeftFlap(Constants.Outtake.leftFlapWall);
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
            if (rightWall) {
                return offset.addReturn(new Pose(0, 1.0));
            } else {
                return offset.addReturn(new Pose(0, 0.9));
            }
        }

        public String getName() {
            return "From Bar Right Inner To Wall, " + state;
        }
    }
}
