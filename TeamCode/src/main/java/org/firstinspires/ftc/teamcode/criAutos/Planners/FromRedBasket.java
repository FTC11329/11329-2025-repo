package org.firstinspires.ftc.teamcode.criAutos.Planners;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;
import static org.firstinspires.ftc.teamcode.criAutos.CommonPoses.*;
import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.BezierCurve;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Path;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.PathChain;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Point;
import org.firstinspires.ftc.teamcode.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.utility.PlacePosEnum;
import org.firstinspires.ftc.teamcode.utility.Robot;

public class FromRedBasket {
    // has option to pickup and place spikes, pickup and place yellow,
    // pickup color from sub and pickup wall left or right, or pickup wall left or right
    public static class ToPickupAndPlaceSpike1 implements PathPlanner {
        /// Expects intake pre extended
        /// Ends at basket place pos or store with outake if we missed the spike and
        /// Fine with being at store preset
        /// has the intake extended if pre extend

        // Variables
        Pose offset;
        private Timer pathTimer;
        private int state = 0;
        private int transferState = -1;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        private boolean preExtend;
        public ToPickupAndPlaceSpike1(Robot robot, Pose startPose, boolean preExtend) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.preExtend = preExtend;
        }
        //Poses
        //todo:
        private final Pose spike1 = new Pose();

        //Paths
        private Path toSpike;
        private Path toBasket;

        @Override
        public void buildPaths(Pose offset) {
            this.offset = offset;
            toSpike = robot.follower.linearPathBuilder(startPose, spike1);
            toBasket = robot.follower.linearPathBuilder(spike1, redBasket);
        }

        @Override
        public Pose getEndPose() {
            return redBasket;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    robot.follower.followPath(toSpike, false);

                    robot.outtakeSystem.setArmPos(Constants.Outtake.intakeArm);

                    robot.intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
                    setPathState(1);
                    break;
                case 1:
                    if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                        robot.outtakeSystem.setVSlidePos(Constants.Outtake.intakeSlides);
                        robot.outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                        robot.intakeSystem.setHSlidePos(Constants.Intake.maxSlidePos);
                        setPathState(2);
                    }
                    break;
                case 2:
                    robot.doDriveShake = true;
                    robot.intakeSystem.update();
                    if (pathTimer.getElapsedTimeSeconds() > 1.1) {
                        robot.follower.startTeleopDrive();
                    }
                    if (robot.intakeSystem.intakeUntil()){
                        robot.doDriveShake = false;
                        robot.follower.followPath(toBasket);
                        robot.intakeSystem.storeOutPos();
                        robot.intakeSystem.setIntakePower(Constants.Intake.unjamSpeed);
                        robot.stateMachine.goHighBasket(true, false, false);
                        setPathState(3);
                    } else if (pathTimer.getElapsedTimeSeconds() > 2) {
                        robot.atStorePreset = true;
                        robot.doDriveShake = false;
                        robot.follower.followPath(toBasket);
                        robot.intakeSystem.storeOutPos();
                        isFinished = true;
                        setPathState(5);
                    }
                    break;
                case 3:
                    if (!robot.stateMachine.goingHighBasket()) {
                        robot.atStorePreset = false;
                        if (preExtend) {
                            robot.intakeSystem.setHSlidePos(Constants.Intake.autoPreExtendSlides - 100);
                        }
                        setPathState(4);
                    }
                    break;
                case 4:
                    if (pathTimer.getElapsedTimeSeconds() > 0.4) {
                        robot.outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                        robot.intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
                        setPathState(5);
                    }
                    break;
                case 5:
                    if (pathTimer.getElapsedTimeSeconds() > 0.25) {
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

        //todo
        public Pose getOffset() {
            return offset.addReturn(new Pose());
        }
    }
    public static class ToPickupAndPlaceSpike2 implements PathPlanner {
        /// Expects intake pre extended
        /// Ends at basket place pos or store with outake if we missed the spike and
        /// Fine with being at store preset
        /// has the intake extended if pre extend

        // Variables
        Pose offset;
        private Timer pathTimer;
        private int state = 0;
        private int transferState = -1;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        private boolean preExtend;
        public ToPickupAndPlaceSpike2(Robot robot, Pose startPose, boolean preExtend) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.preExtend = preExtend;
        }
        //Poses
        //todo:
        private final Pose spike2 = new Pose();

        //Paths
        private Path toSpike;
        private Path toBasket;

        @Override
        public void buildPaths(Pose offset) {
            this.offset = offset;
            toSpike = robot.follower.linearPathBuilder(startPose, spike2);
            toBasket = robot.follower.linearPathBuilder(spike2, redBasket);
        }

        @Override
        public Pose getEndPose() {
            return redBasket;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    robot.follower.followPath(toSpike, false);

                    robot.outtakeSystem.setArmPos(Constants.Outtake.intakeArm);

                    robot.intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
                    setPathState(1);
                    break;
                case 1:
                    if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                        robot.outtakeSystem.setVSlidePos(Constants.Outtake.intakeSlides);
                        robot.outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                        robot.intakeSystem.setHSlidePos(Constants.Intake.maxSlidePos);
                        setPathState(2);
                    }
                    break;
                case 2:
                    robot.doDriveShake = true;
                    robot.intakeSystem.update();
                    if (pathTimer.getElapsedTimeSeconds() > 1.1) {
                        robot.follower.startTeleopDrive();
                    }
                    if (robot.intakeSystem.intakeUntil()){
                        robot.doDriveShake = false;
                        robot.follower.followPath(toBasket);
                        robot.intakeSystem.storeOutPos();
                        robot.intakeSystem.setIntakePower(Constants.Intake.unjamSpeed);
                        robot.stateMachine.goHighBasket(true, false, false);
                        setPathState(3);
                    } else if (pathTimer.getElapsedTimeSeconds() > 2) {
                        robot.atStorePreset = true;
                        robot.doDriveShake = false;
                        robot.follower.followPath(toBasket);
                        robot.intakeSystem.storeOutPos();
                        isFinished = true;
                        setPathState(5);
                    }
                    break;
                case 3:
                    if (!robot.stateMachine.goingHighBasket()) {
                        robot.atStorePreset = false;
                        if (preExtend) {
                            robot.intakeSystem.setHSlidePos(Constants.Intake.autoPreExtendSlides - 100);
                        }
                        setPathState(4);
                    }
                    break;
                case 4:
                    if (pathTimer.getElapsedTimeSeconds() > 0.4) {
                        robot.outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                        robot.intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
                        setPathState(5);
                    }
                    break;
                case 5:
                    if (pathTimer.getElapsedTimeSeconds() > 0.25) {
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

        //todo
        public Pose getOffset() {
            return offset.addReturn(new Pose());
        }
    }

    public static class ToPickupAndPlaceSpike3 implements PathPlanner {
        /// Expects intake pre extended
        /// Ends at basket place pos or store with outake if we missed the spike and
        /// Fine with being at store preset
        /// has the intake extended if pre extend

        // Variables
        Pose offset;
        private Timer pathTimer;
        private int state = 0;
        private int transferState = -1;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        public ToPickupAndPlaceSpike3(Robot robot, Pose startPose) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
        }
        //Poses
        //todo:
        private final Pose spike3 = new Pose();

        //Paths
        private Path toSpike;
        private Path toBasket;

        @Override
        public void buildPaths(Pose offset) {
            this.offset = offset;
            toSpike = robot.follower.linearPathBuilder(startPose, spike3);
            toBasket = robot.follower.linearPathBuilder(spike3, redBasket);
        }

        @Override
        public Pose getEndPose() {
            return redBasket;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    robot.follower.followPath(toSpike, false);

                    robot.outtakeSystem.setArmPos(Constants.Outtake.intakeArm);

                    robot.intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
                    setPathState(1);
                    break;
                case 1:
                    if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                        robot.outtakeSystem.setVSlidePos(Constants.Outtake.intakeSlides);
                        robot.outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                        robot.intakeSystem.setHSlidePos(Constants.Intake.maxSlidePos);
                        setPathState(2);
                    }
                    break;
                case 2:
                    robot.doDriveShake = true;
                    robot.intakeSystem.update();
                    if (pathTimer.getElapsedTimeSeconds() > 1.1) {
                        robot.follower.startTeleopDrive();
                    }
                    if (robot.intakeSystem.intakeUntil()){
                        robot.doDriveShake = false;
                        robot.follower.followPath(toBasket);
                        robot.intakeSystem.storeOutPos();
                        robot.intakeSystem.setIntakePower(Constants.Intake.unjamSpeed);
                        robot.stateMachine.goHighBasket(true, false, false);
                        setPathState(3);
                    } else if (pathTimer.getElapsedTimeSeconds() > 2) {
                        robot.atStorePreset = true;
                        robot.doDriveShake = false;
                        robot.follower.followPath(toBasket);
                        robot.intakeSystem.storeOutPos();
                        isFinished = true;
                        setPathState(5);
                    }
                    break;
                case 3:
                    if (!robot.stateMachine.goingHighBasket()) {
                        robot.atStorePreset = false;
                        setPathState(4);
                    }
                    break;
                case 4:
                    if (pathTimer.getElapsedTimeSeconds() > 0.4) {
                        robot.outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                        robot.intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
                        setPathState(5);
                    }
                    break;
                case 5:
                    if (pathTimer.getElapsedTimeSeconds() > 0.25) {
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

        //todo
        public Pose getOffset() {
            return offset.addReturn(new Pose());
        }
    }



    public static class ToPickupAndPlaceSubYellow implements PathPlanner {
        /// Starts at basket with intake NOT extended
        /// Ends at basket with outake at basket
        /// Fine with being at store preset
        /// If we run out of time, we will go park and not continue other steps
        // Variables
        Pose offset;
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;
        private Pose2D target2D;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        public ToPickupAndPlaceSubYellow(Robot robot, Pose startPose) {
            this.robot = robot;
            this.startPose = startPose;
        }
        //Poses
        //todo:
        private final Pose toSubControlPoint = new Pose();
        private final Pose toBasketControlPoint = new Pose();

        //Paths
        Path toSub;
        Path toBasket;

        @Override
        public void buildPaths(Pose offset) {
            this.offset = offset;
            toSub = new Path(new BezierCurve(new Point(startPose), new Point(toSubControlPoint), new Point(intakeSubLeftOuter)));
            toSub.setLinearHeadingInterpolation(startPose.getHeading(), intakeSubLeftOuter.getHeading());

            toBasket = new Path(new BezierCurve(new Point(intakeSubLeftOuter), new Point(toBasketControlPoint), new Point(redBasket)));
            toBasket.setLinearHeadingInterpolation(intakeSubLeftOuter.getHeading(), redBasket.getHeading());
        }

        @Override
        public Pose getEndPose() {
            return redBasket;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    robot.follower.followPath(toBasket);
                    robot.outtakeSystem.setArmPos(Constants.Outtake.intakeArm);
                case 1:
                    if (robot.follower.getErrorDistance(intakeSubLeftOuter) < 2) {
                        robot.outtakeSystem.setVSlidePos(Constants.Outtake.intakeSlides);
                        robot.outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (pathTimer.getElapsedTimeSeconds() > 0.4) {
                        target2D = robot.blockVision.getBlockPosition(true);
                        if (target2D.getHeading(AngleUnit.DEGREES) != -1) {
                            robot.intakeSystem.setHSlidesInches(robot.follower.followYourHead(target2D));
                            setPathState(3);
                        }
                        if (robot.opmodeTimer.getElapsedTimeSeconds() > 28) {
                            //Break loop
                            setPathState(9);
                        }
                    }
                    break;
                case 3:
                    if ((pathTimer.getElapsedTimeSeconds() > 0.2 && Math.abs(robot.intakeSystem.getHSlideTargetPos() - robot.intakeSystem.getHSlidePos()) < 250 && robot.follower.getHeadingError() < Math.toRadians(2)) || pathTimer.getElapsedTimeSeconds() > 0.5) {
                        robot.intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
                        setPathState(4);
                    }
                    break;
                case 4:
                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                        robot.doDriveShake = true;
                        robot.intakeSystem.intakeUntilColor();
                        setPathState(5);
                    }
                    break;
                case 5:
                    if (pathTimer.getElapsedTimeSeconds() > 1) {
                        robot.follower.startTeleopDrive();
                    }
                    robot.intakeSystem.update();
                    if (robot.opmodeTimer.getElapsedTimeSeconds() > 28) {
                        //Break loop
                        setPathState(9);
                    }
                    if (robot.intakeSystem.intakeUntil()) {
                        robot.doDriveShake = false;
                        robot.follower.followPath(toBasket);
                        robot.intakeSystem.storeOutPos();
                        robot.intakeSystem.setIntakePower(Constants.Intake.unjamSpeed);
                        robot.stateMachine.goHighBasket(true, false, false);
                        setPathState(6);
                    }
                    break;
                case 6:
                    if (!robot.stateMachine.goingHighBasket() && robot.follower.getError(redBasket).getX() < 1 && robot.follower.getError(redBasket).getY() < 1 || pathTimer.getElapsedTimeSeconds() > 5) {
                        setPathState(7);
                    }
                    break;
                case 7:
                    if (pathTimer.getElapsedTimeSeconds() > 0.45) {
                        robot.outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                        if (robot.opmodeTimer.getElapsedTimeSeconds() > 25) {
                            //Break loop
                            setPathState(9);
                        } else {
                            setPathState(8);
                        }
                    }
                    break;
                case 8:
                    if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                        isFinished = true;
                    }
                    break;

                    //Parking if out of time
                case 9:
                    if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                        robot.doIntakeWhilePark = true;
                        robot.follower.followPath(toSub);
                        robot.outtakeSystem.setVSlidePos(Constants.Outtake.safeFromClimberBar);
                        setPathState(10);
                    }
                    break;
                case 10:
                    if (robot.outtakeSystem.getVSlidePos() > Constants.Outtake.safeFromClimberBar - 50) {
                        robot.outtakeSystem.setArmPos(Constants.Outtake.parkArm);

                        setPathState(11);
                    }
                    break;
                case 11:
                    if (pathTimer.getElapsedTimeSeconds() > 0.05) {
                        robot.outtakeSystem.setVSlidePos(Constants.Outtake.safeFromHSlides + 200);

                        setPathState(12);
                    }
                    break;
                case 12:
                    if (pathTimer.getElapsedTimeSeconds() > 0.1 && robot.follower.getError(intakeSubLeftOuter).getX() < 2) {
                        robot.outtakeSystem.setVSlidePos(100);

                        setPathState(13);
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


    public static class ToPickupColorFromSubToPickupWall implements PathPlanner {
        /// Starts at basket with intake NOT extended
        /// Ends at pickup wall with block in claw
        /// Fine with being at store preset
        /// +1 color and -1 piece to human player
        /// Has an option to go left or right wall
        // Variables
        Pose offset;
        private Timer wristTimer;
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;
        private Pose2D visionResult;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        private boolean leftWall;
        public ToPickupColorFromSubToPickupWall(Robot robot, Pose startPose, boolean leftWall) {
            pathTimer = new Timer();
            wristTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.leftWall = leftWall;
        }
        //Poses
        //todo
        private final Pose toSubControlPoint = new Pose();

        private final Pose toRightWallControlPoint1 = new Pose();
        private final Pose toRightWallControlPoint2 = new Pose();

        private final Pose toLeftWallControlPoint1 = new Pose();
        private final Pose toLeftWallControlPoint2 = new Pose();

        private Pose tartgetPose;

        //Paths
        Path toSub;
        Path toWall;

        @Override
        public void buildPaths(Pose offset) {
            this.offset = offset;
            toSub = new Path(new BezierCurve(startPose, toSubControlPoint, intakeSubLeftMiddle));
            toSub.setLinearHeadingInterpolation(startPose, intakeSubLeftMiddle);

            if (leftWall) {
                toWall = new Path(new BezierCurve(startPose, toLeftWallControlPoint1, toLeftWallControlPoint2, intakeWallLeft));
                toWall.setLinearHeadingInterpolation(startPose.getHeading() + Math.toRadians(45), intakeWallLeft.getHeading(), 0.75);
                tartgetPose = intakeWallLeft;
            } else {
                toWall = new Path(new BezierCurve(startPose, toRightWallControlPoint1, toRightWallControlPoint2, intakeWallRight));
                toWall.setLinearHeadingInterpolation(startPose.getHeading() + Math.toRadians(45), intakeWallRight.getHeading(), 0.68);
                tartgetPose = intakeWallRight;
            }
        }

        @Override
        public Pose getEndPose() {
            return intakeSubLeftMiddle;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    robot.follower.followPath(toSub);
                    setPathState(1);
                    break;
                case 1:
                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                        if (robot.atStorePreset) {
                            robot.stateMachine.goWall(false, false, true);
                        } else {
                            robot.outtakeSystem.placePos(PlacePosEnum.wallAuto);
                        }
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (robot.follower.getErrorDistance(intakeSubLeftMiddle) < 1.5) {
                        setPathState(3);
                    }
                    break;
                case 3:
                    if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                        robot.intakeSystem.setHSlidePos(0);
                        visionResult = robot.blockVision.getBlockPosition(true);
                        if (visionResult.getHeading(AngleUnit.DEGREES) != -1) {
                            robot.outtakeSystem.placePos(PlacePosEnum.wallAuto);
                            robot.outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                            robot.intakeSystem.setHSlidesInches(robot.follower.followYourHead(visionResult));
                            setPathState(4);

                        } else if (pathTimer.getElapsedTimeSeconds() > 1) {
                            robot.outtakeSystem.placePos(PlacePosEnum.wallAuto);
                            robot.outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                            robot.intakeSystem.storePos();
                            robot.follower.followPath(toWall);
                            setPathState(100);
                        }
                    }
                    break;
                case 4:
                    if ((pathTimer.getElapsedTimeSeconds() > 0.2 && Math.abs(robot.intakeSystem.getHSlideTargetPos() - robot.intakeSystem.getHSlidePos()) < 250 && robot.follower.getHeadingError() < Math.toRadians(2)) || pathTimer.getElapsedTimeSeconds() > 0.5) {
                        robot.intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
                        setPathState(5);
                    }
                    break;
                case 5:
                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                        robot.doDriveShake = true;
                        robot.follower.startTeleopDrive();
                        robot.intakeSystem.intakeUntilColor();
                        setPathState(6);
                    }
                    break;
                case 6:
                    robot.intakeSystem.update();
                    if (robot.intakeSystem.intakeUntilColor()) {
                        wristTimer.resetTimer();

                        robot.intakeSystem.storePos();
                        robot.intakeSystem.setIntakePower(Constants.Intake.unjamSpeed);
                        robot.doDriveShake = false;
                        robot.follower.breakFollowing();

                        robot.follower.followPath(toWall);
                        setPathState(7);

                    } else if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                        robot.intakeSystem.storePos();
                        robot.intakeSystem.setIntakePower(Constants.Intake.spitSpeed);
                        robot.follower.breakFollowing();
                        robot.doDriveShake = false;

                        setPathState(100);
                    }
                    break;
                case 7:
                    if (pathTimer.getElapsedTimeSeconds() > Constants.Intake.unjamTimeMillisAuto / 1000) {
                        robot.intakeSystem.setIntakePower(Constants.Intake.intakeSpeed);
                        setPathState(8);
                    }
                    break;
                case 8:
                    if (robot.intakeSystem.intakeUntil() || pathTimer.getElapsedTimeSeconds() > Constants.Intake.unjamTimeMillisAuto / 1000){
                        robot.intakeSystem.setIntakePower(0);
                        setPathState(9);
                    }
                    break;
                case 9:
                    if (wristTimer.getElapsedTimeSeconds() > 0.65) {
                        robot.intakeSystem.setIntakePower(Constants.Intake.transferSpeed);

                        setPathState(10);
                    }
                    break;

                    //Missed first Intake
                case 100:
                    if (robot.intakeSystem.getHSlideTargetPos() < 100 || pathTimer.getElapsedTimeSeconds() > 2) {
                        robot.follower.followPath(toWall);
                        setPathState(10);
                    }
                    break;
                case 10:
                    if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                        robot.outtakeSystem.placePos(PlacePosEnum.wallAuto);
                        setPathState(11);
                    }
                    break;
                //change to be transfer and dump
                case 11:
                    if (robot.follower.getErrorDistance(tartgetPose) < 12) {
                        robot.intakeSystem.setDepoServoPos(Constants.Intake.depoDepo);

                        setPathState(12);
                    }
                    break;
                case 12:
                    if ((robot.follower.getErrorDistance(tartgetPose) < 2 && robot.outtakeSystem.seesWall()) || pathTimer.getElapsedTimeSeconds() > 1) {
                        setPathState(13);
                    }
                    break;
                case 13:
                    if (pathTimer.getElapsedTimeSeconds() > 0.05) {
                        robot.intakeSystem.setDepoServoPos(Constants.Intake.depoStore);
                        robot.outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                        setPathState(14);
                    }
                    break;
                case 14:
                    if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                        isFinished = true;
                        setPathState(15);
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

    public static class ToPickupWall implements PathPlanner {
        /// Starts at basket with intake NOT extended
        /// Ends at pickup wall with block in claw
        /// -1 piece to human player
        /// Has an option to go left or right wall
        // Variables
        Pose offset;
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;
        private Pose2D visionResult;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        private boolean leftWall;

        public ToPickupWall(Robot robot, Pose startPose, boolean leftWall) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.leftWall = leftWall;
        }

        //Poses
        private Pose tartgetPose;

        //Paths
        PathChain toWall;

        @Override
        public void buildPaths(Pose offset) {
            this.offset = offset;
            if (leftWall) {
                Pose halfWayWallPose = new Pose(redBasket.getX(), (startPose.getY() + intakeWallLeft.getY()) / 2, Math.toRadians(-90));
                toWall = robot.follower.pathBuilder()
                        .addPath(new BezierCurve(startPose, halfWayWallPose))
                        .setLinearHeadingInterpolation(startPose.getHeading(), halfWayWallPose.getHeading(), 0.175)
                        .addPath(new BezierCurve(halfWayWallPose, intakeWallLeft))
                        .setLinearHeadingInterpolation(halfWayWallPose.getHeading(), intakeWallLeft.getHeading(), 0.5)
                        .build();

                tartgetPose = intakeWallLeft;
            } else {
                Pose halfWayWallPose = new Pose(redBasket.getX(), (startPose.getY() + intakeWallRight.getY()) / 2, Math.toRadians(-90));
                toWall = robot.follower.pathBuilder()
                        .addPath(new BezierCurve(startPose, halfWayWallPose))
                        .setLinearHeadingInterpolation(startPose.getHeading(), halfWayWallPose.getHeading(), 0.175)
                        .addPath(new BezierCurve(halfWayWallPose, intakeWallRight))
                        .setLinearHeadingInterpolation(halfWayWallPose.getHeading(), intakeWallRight.getHeading(), 0.5)
                        .build();
                tartgetPose = intakeWallRight;
            }
        }

        @Override
        public Pose getEndPose() {
            return intakeSubLeftMiddle;
        }


        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    robot.follower.followPath(toWall);
                    setPathState(1);
                    break;
                case 1:
                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                        if (robot.atStorePreset) {
                            robot.stateMachine.goWall(false, false, true);
                        } else {
                            robot.outtakeSystem.placePos(PlacePosEnum.wallAuto);
                        }
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (robot.follower.getErrorDistance(tartgetPose) < 1.5 && !robot.stateMachine.goingHighBasket()) {
                        setPathState(3);
                    }
                    break;
                case 3:
                    if ((robot.follower.getErrorDistance(tartgetPose) < 2 && robot.outtakeSystem.seesWall()) || pathTimer.getElapsedTimeSeconds() > 2) {
                        setPathState(4);
                    }
                    break;
                case 4:
                    if (pathTimer.getElapsedTimeSeconds() > 0.15) {
                        robot.outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                        setPathState(5);
                    }
                    break;
                case 5:
                    if (pathTimer.getElapsedTimeSeconds() > 0.25) {
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

        //todo
        public Pose getOffset() {
            return offset.addReturn(new Pose());
        }
    }
}
