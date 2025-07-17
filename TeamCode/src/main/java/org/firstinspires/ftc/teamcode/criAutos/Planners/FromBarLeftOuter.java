package org.firstinspires.ftc.teamcode.criAutos.Planners;

import static org.firstinspires.ftc.teamcode.criAutos.CommonPoses.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.BezierCurve;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Path;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.PathBuilder;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.PathChain;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Point;
import org.firstinspires.ftc.teamcode.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.utility.PlacePosEnum;
import org.firstinspires.ftc.teamcode.utility.Robot;
import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

public class FromBarLeftOuter {
    public static class ToWall implements PathPlanner {
        /// Option to pickup and drop one
        /// Option to go left or right wall
        /// Expects robot.state.whereAmI to be correct
        /// Ends at left or right wall
        // Variables
        boolean leftWall;
        boolean park = false;
        boolean superCycle;
        Pose offset = new Pose();
        Pose2D visionResult;
        private Timer pathTimer;
        private int state = -1;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;

        public ToWall(Robot robot, Pose startPose, boolean superCycle, boolean leftWall, Pose offset) {
            addToOffset(offset);
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.leftWall = leftWall;
            this.superCycle = superCycle;
        }

        public ToWall(Robot robot, Pose startPose, boolean superCycle, boolean leftWall) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.leftWall = leftWall;
            this.superCycle = superCycle;
        }

        public ToWall(Robot robot, Pose startPose, boolean superCycle, boolean leftWall, boolean park) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.leftWall = leftWall;
            this.superCycle = superCycle;
            this.park = park;
        }

        //Poses
        Pose controlPoint = new Pose(-36, 110);
        Pose midPoint = new Pose(-48, 48, Math.toRadians(75));


        Pose startPoseAdded;
        Pose controlPointAdded;
        Pose midPointAdded;
        Pose pickupWallLeftAdded;
        Pose pickupWallRightAdded;


        //Paths
        PathChain toWall;
        PathBuilder toWallBuilder;

        @Override
        public void buildPaths(Pose offset) {
            this.offset.add(offset);

            robot.blockVision.switchPipeline(robot.robotState.robotSide);

            startPoseAdded = startPose.addReturn(offset);
            controlPointAdded = controlPoint.addReturn(offset);
            midPointAdded = midPoint.addReturn(offset);
            pickupWallLeftAdded = pickupWallLeft.addReturn(offset);
            pickupWallRightAdded = pickupWallRight.addReturn(offset);
            if (park) {
                pickupWallLeftAdded.setHeading(Math.toRadians(-90));
            }

            toWallBuilder = robot.follower.pathBuilder()
                    .addPath(new Path(new BezierCurve(new Point(startPoseAdded), new Point(controlPointAdded), new Point(midPointAdded))))
                    .setLinearHeadingInterpolation(startPoseAdded.getHeading(), pickupWallLeftAdded.getHeading(), 0.6);

            if (leftWall) {
                toWallBuilder.addPath(robot.follower.linearPathBuilder(midPointAdded, pickupWallLeftAdded))
                        .setConstantHeadingInterpolation(pickupWallLeftAdded.getHeading());
            } else {
                toWallBuilder.addPath(robot.follower.linearPathBuilder(midPointAdded, pickupWallRightAdded))
                        .setConstantHeadingInterpolation(pickupWallRightAdded.getHeading());
            }
            toWall = toWallBuilder.build();
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
                case -1:
                    if (!superCycle) {
                        setPathState(6);
                        break;
                    }
                    robot.follower.setMaxPower(0.7);
                    robot.follower.turn(Math.toRadians(90), false);
                    setPathState(0);
                case 0:
                    visionResult = robot.blockVision.getBlockPosition(true);

                    if (visionResult.getHeading(AngleUnit.DEGREES) != -1) {
                        robot.follower.setMaxPower(1);
                        if (!park) {
                            robot.stateMachine.goStore();
                        }
                        robot.stateMachine.setBringSlidesIn(false);
                        robot.outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                        robot.intakeSystem.setHSlidesInches(robot.follower.followYourHead(visionResult));
                        setPathState(1);

                    } else if (pathTimer.getElapsedTimeSeconds() > 1) {
                        robot.follower.setMaxPower(1);
                        robot.outtakeSystem.placePos(PlacePosEnum.wall);
                        robot.outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                        robot.intakeSystem.storePos();
                        robot.follower.followPath(toWall);
                        setPathState(6);
                    }
                    break;
                case 1:
                    if ((pathTimer.getElapsedTimeSeconds() > 0.2 && Math.abs(robot.intakeSystem.getHSlideTargetPos() - robot.intakeSystem.getHSlidePos()) < 250 && robot.follower.getHeadingError() < Math.toRadians(2)) || pathTimer.getElapsedTimeSeconds() > 0.5) {
                        robot.intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
                        if (robot.robotState.whereAmI == PlacePosEnum.highSpecimen) {
                            robot.outtakeSystem.setArmPos(Constants.Outtake.intakeArm);
                        }
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                        robot.doDriveShake = true;
                        robot.follower.startTeleopDrive();
                        robot.intakeSystem.intakeUntilColor();
                        setPathState(3);
                    }
                    break;
                case 3:
                    robot.intakeSystem.update();
                    if (robot.intakeSystem.intakeUntilColor()) {

                        robot.intakeSystem.storeOutPos();
                        robot.intakeSystem.setIntakePower(Constants.Intake.unjamSpeed);
                        robot.doDriveShake = false;
                        robot.follower.breakFollowing();
                        robot.robotState.hasInIntake = true;

                        robot.follower.followPath(toWall);
                        setPathState(4);

                    } else if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                        robot.intakeSystem.storeOutPos();
                        robot.intakeSystem.setIntakePower(Constants.Intake.spitSpeed);
                        robot.follower.breakFollowing();
                        robot.doDriveShake = false;

                        setPathState(6);
                    }
                    break;
                case 4:
                    if (pathTimer.getElapsedTimeSeconds() > Constants.Intake.unjamTimeMillisAuto / 1000) {
                        robot.intakeSystem.setIntakePower(Constants.Intake.intakeSpeed);
                        setPathState(5);
                    }
                    break;
                case 5:
                    if (robot.intakeSystem.intakeUntil() || pathTimer.getElapsedTimeSeconds() > Constants.Intake.unjamTimeMillisAuto / 1000) {
                        robot.intakeSystem.setIntakePower(0);
                        robot.stateMachine.goWall(true, false, robot.robotState.whereAmI == PlacePosEnum.intake);
                        setPathState(7);
                    }
                    break;
                //Failed pickup
                case 6:
                    robot.follower.followPath(toWall);
                    robot.stateMachine.goWall(false, false, robot.robotState.whereAmI == PlacePosEnum.intake);
                    setPathState(7);
                    break;
                case 7:
                    if (robot.follower.getError(pickupWallLeftAdded).getY() < 20) {
                        robot.intakeSystem.setIntakePower(0);
                        robot.outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                        robot.outtakeSystem.setFlapsWall();
                        setPathState(8);
                    }
                    break;
                case 8:
                    if ((robot.outtakeSystem.seesWall() && pathTimer.getElapsedTimeSeconds() > 0.3) || pathTimer.getElapsedTimeSeconds() > 1) {
                        setPathState(9);
                    }
                    break;
                case 9:
                    if (pathTimer.getElapsedTimeSeconds() > 0.05) {
                        robot.outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                        setPathState(10);
                    }
                    break;
                case 10:
                    if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                        isFinished = true;
                        setPathState(11);
                    }
                    break;
            }

            return isFinished;
        }

        public void setPathState(int state) {
            this.state = state;
            pathTimer.resetTimer();
        }

        public Pose getOffset() {
            return offset.addReturn(new Pose(-2, 0.3));
        }

        public void addToOffset(Pose offset) {
            this.offset = offset;
        }


        public String getName() {
            return "From Bar Left Outer To Wall, " + state;
        }
    }

    public static class ToBasket implements PathPlanner {
        ///
        // Variables
        boolean preExtend;
        Pose offset = new Pose();
        Pose2D visionResult;
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;

        public ToBasket(Robot robot, Pose startPose, boolean preExtend, Pose offset) {
            addToOffset(offset);
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.preExtend = preExtend;
        }

        public ToBasket(Robot robot, Pose startPose, boolean preExtend) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.preExtend = preExtend;
        }

        //Poses
        Pose controlPoint = new Pose(-36, 110);
        Pose midPoint = new Pose(-48, 48, Math.toRadians(75));


        Pose startPoseAdded;
        Pose basketAdded;


        //Paths
        PathChain toBasket;

        @Override
        public void buildPaths(Pose offset) {
            this.offset.add(offset);

            robot.blockVision.switchPipeline(0);

            startPoseAdded = startPose.addReturn(offset);
            basketAdded = redBasket.addReturn(offset);


            toBasket = robot.follower.linearPathChainBuilder(startPose, basketAdded, 0.9);
        }


        @Override
        public Pose getEndPoseEst() {
            return redBasket;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    visionResult = robot.blockVision.getBlockPosition(true);
                    if (visionResult.getHeading(AngleUnit.DEGREES) != -1) {
                        robot.stateMachine.goStore();
                        robot.stateMachine.setBringSlidesIn(false);
                        robot.outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                        robot.intakeSystem.setHSlidesInches(robot.follower.followYourHead(visionResult));
                        setPathState(1);

                    } else if (pathTimer.getElapsedTimeSeconds() > 1) {
                        robot.outtakeSystem.placePos(PlacePosEnum.wall);
                        robot.outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                        robot.intakeSystem.storePos();
                        robot.stateMachine.setBringSlidesIn(false);
                        robot.follower.followPath(toBasket);
                        setPathState(6);
                    }
                    break;
                case 1:
                    if ((pathTimer.getElapsedTimeSeconds() > 0.2 && Math.abs(robot.intakeSystem.getHSlideTargetPos() - robot.intakeSystem.getHSlidePos()) < 250 && robot.follower.getHeadingError() < Math.toRadians(2)) || pathTimer.getElapsedTimeSeconds() > 0.5) {
                        robot.intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
                        if (robot.robotState.whereAmI == PlacePosEnum.highSpecimen) {
                            robot.outtakeSystem.setArmPos(Constants.Outtake.intakeArm);
                        }
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                        robot.doDriveShake = true;
                        robot.follower.startTeleopDrive();
                        robot.intakeSystem.intakeUntilColor();
                        setPathState(3);
                    }
                    break;
                case 3:
                    robot.intakeSystem.update();
                    if (robot.intakeSystem.intakeUntilColor()) {

                        robot.intakeSystem.storeOutPos();
                        robot.intakeSystem.setIntakePower(Constants.Intake.unjamSpeed);
                        robot.doDriveShake = false;
                        robot.follower.breakFollowing();
                        robot.robotState.hasInIntake = true;

                        robot.follower.followPath(toBasket);
                        setPathState(4);

                    } else if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                        robot.intakeSystem.storeOutPos();
                        robot.intakeSystem.setIntakePower(Constants.Intake.spitSpeed);
                        robot.follower.breakFollowing();
                        robot.doDriveShake = false;

                        setPathState(6);
                    }
                    break;
                case 4:
                    if (pathTimer.getElapsedTimeSeconds() > Constants.Intake.unjamTimeMillisAuto / 1000) {
                        robot.intakeSystem.setIntakePower(Constants.Intake.intakeSpeed);
                        setPathState(5);
                    }
                    break;
                case 5:
                    if (robot.intakeSystem.intakeUntil() || pathTimer.getElapsedTimeSeconds() > Constants.Intake.unjamTimeMillisAuto / 1000) {
                        robot.intakeSystem.setIntakePower(0);
                        robot.stateMachine.goHighBasket(true, false, false, robot.robotState.whereAmI == PlacePosEnum.intake);
                        robot.outtakeSystem.setFlapsUp();
                        setPathState(7);
                    }
                    break;
                //Failed pickup
                case 6:
                    robot.follower.followPath(toBasket);
                    robot.stateMachine.goWall(false, false, robot.robotState.whereAmI == PlacePosEnum.intake);
                    setPathState(7);
                    break;
                case 7:
                    if (robot.follower.getErrorDistance(basketAdded) < 1 && !robot.stateMachine.doHighBasket()) {
                        robot.intakeSystem.setIntakePower(0);
                        if (preExtend) {
                            robot.intakeSystem.setHSlidePos(Constants.Intake.autoPreExtendSlides - 100);
                        }
                        setPathState(8);
                    }
                    break;
                case 8:
                    if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                        robot.outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                        setPathState(9);
                    }
                    break;
                case 9:
                    if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                        isFinished = true;
                        setPathState(10);
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

        public void addToOffset(Pose offset) {
            this.offset = offset;
        }


        public String getName() {
            return "From Bar Left Outer To Basket, " + state;
        }
    }
}
