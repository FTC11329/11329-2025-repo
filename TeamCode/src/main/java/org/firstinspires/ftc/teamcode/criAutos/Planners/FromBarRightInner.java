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
        /// Option to push spike 0 is none, 1 is bottom, 2 is middle, 3 is NOT WORKING
        /// Option to go left or right wall
        /// Expects robot.state.whereami to be correct
        /// Ends at left or right wall
        // Variables
        boolean rightWall;
        boolean park = false;
        int pushSpike;
        Pose offset = new Pose();
        private Timer pathTimer;
        private int state = -1;
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

        public ToWall(Robot robot, Pose startPose, int pushSpike, boolean rightWall, boolean park) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.rightWall = rightWall;
            this.pushSpike = pushSpike;
            this.park = park;
        }

        //Poses
        Pose startPoseLeftAdded;
        Pose wallAdded;
        Pose innerSpikeRightBotAdded;
        Pose targetPoseAdded;
        Pose openAdded;

        //Paths
        PathBuilder toWall;
        PathChain toWallPath;

        @Override
        public void buildPaths(Pose offset) {
            this.offset.add(offset);

            startPoseLeftAdded = new Pose(startPose.getX(), innerSpikeRightMid.getY(), Math.toRadians(-87)).addReturn(offset);
            innerSpikeRightBotAdded = innerSpikeRightBot.addReturn(offset);
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
                case -1:
                    targetPoseAdded = innerSpikeRightBot.addReturn(new Pose(0, 3)).addReturn(offset);
                    break;
                case 2:
                    targetPoseAdded = innerSpikeRightMid.addReturn(offset);
                    break;
                case -2:
                    targetPoseAdded = innerSpikeRightMid.addReturn(new Pose(0, 3)).addReturn(offset);
                    break;
            }
            targetPoseAdded.setHeading(Math.toRadians(-87));

            toWall = robot.follower.pathBuilder()
                    .addPath(robot.follower.linearPathBuilder(startPoseLeftAdded, targetPoseAdded));
            if (pushSpike < 0) {
                toWall.setZeroPowerAccelerationMultiplier(3);
            }
            if (rightWall) {
                toWall.addPath(robot.follower.linearPathBuilder(targetPoseAdded, wallAdded, 0.2))
                        .setZeroPowerAccelerationMultiplier(9);
            } else {
                toWall.addPath(robot.follower.linearPathBuilder(targetPoseAdded, openAdded, 0.2))
                        .addPath(robot.follower.linearPathBuilder(openAdded, wallAdded))
                        .setZeroPowerAccelerationMultiplier(9);
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
                case -1:
                    robot.follower.setMaxPower(0.8);
                    robot.follower.followPath(toWallPath);
                    robot.stateMachine.goWall(false, false, false);
                    robot.loop();
                    robot.outtakeSystem.setVSlidePos(0);
                    robot.outtakeSystem.setFlapsSpikeClear();
                    setPathState(0);
                    break;
                case 0:
                    if (pathTimer.getElapsedTimeSeconds() > 0) {
                        setPathState(1);
                    }
                    break;
                case 1:
                    robot.outtakeSystem.setVSlidePos(0);
                    if (robot.follower.getError(targetPoseAdded).getX() < 9.75) {
                        if (park) {
                            robot.outtakeSystem.setArmPos(Constants.Outtake.upArm);
                        }
                        robot.outtakeSystem.setFlapsWall();
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (robot.follower.getError(openAdded).getX() < 1) {
                        robot.follower.setMaxPower(1);
                        robot.outtakeSystem.setVSlidePos(Constants.Outtake.intakeWallSlides);
                        setPathState(3);
                    }
                    break;
                case 3:
                    if ((robot.outtakeSystem.seesWall() && robot.follower.getVelocity().getMagnitude() < 3) || pathTimer.getElapsedTimeSeconds() > 2) {
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
                    if (pathTimer.getElapsedTimeSeconds() > 0.4) {
                        robot.outtakeSystem.setVSlidePos(Constants.Outtake.safeFromWallSlides + 50);
                        setPathState(6);
                    }
                    break;
                case 6:
                    if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                        isFinished = true;
                        setPathState(7);
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
                return offset.addReturn(new Pose(1.3, 0.7));
            } else {
                return offset.addReturn(new Pose(1.3, 1));
            }
        }

        public String getName() {
            return "From Bar Right Inner To Wall, " + state;
        }
    }
}
