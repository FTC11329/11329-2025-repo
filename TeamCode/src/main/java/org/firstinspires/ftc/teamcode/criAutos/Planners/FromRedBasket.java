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
        Pose offset = new Pose();
        private Timer pathTimer;
        private int state = 0;
        private int transferState = -1;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        private boolean preExtend;
        public ToPickupAndPlaceSpike1(Robot robot, Pose startPose, boolean preExtend, Pose offset) {
            addToOffset(offset);
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.preExtend = preExtend;
        }
        public ToPickupAndPlaceSpike1(Robot robot, Pose startPose, boolean preExtend) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.preExtend = preExtend;
        }
        //Poses
        Pose startPoseAdded;
        Pose spike1Added;
        Pose redBasketAdded;
        //Paths
        private Path toSpike;
        private Path toBasket;

        @Override
        public void buildPaths(Pose offset) {
            this.offset.add(offset);

            startPoseAdded = startPose.addReturn(offset);
            spike1Added = leftOuterSpike1.addReturn(offset);
            redBasketAdded = redBasket.addReturn(offset);

            toSpike = robot.follower.linearPathBuilder(startPoseAdded, spike1Added);
            toBasket = robot.follower.linearPathBuilder(spike1Added, redBasketAdded);
        }


        @Override
        public Pose getEndPoseEst() {
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
                        robot.stateMachine.goHighBasket(true, false, false, false);
                        setPathState(3);
                    } else if (pathTimer.getElapsedTimeSeconds() > 2) {
                        robot.doDriveShake = false;
                        robot.follower.followPath(toBasket);
                        robot.intakeSystem.storeOutPos();
                        isFinished = true;
                        setPathState(5);
                    }
                    break;
                case 3:
                    if (!robot.stateMachine.goingHighBasket()) {
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

        public void addToOffset(Pose offset) {
            this.offset = offset;
        }

        //todo low priority
        public Pose getOffset() {
            return offset.addReturn(new Pose());
        }

        public String getName() {
            return "From Red Basket To Spike 1, " + state;
        }

    }
    public static class ToPickupAndPlaceSpike2 implements PathPlanner {
        /// Expects intake pre extended
        /// Ends at basket place pos or store with outake if we missed the spike and
        /// Fine with being at store preset
        /// has the intake extended if pre extend

        // Variables
        Pose offset = new Pose();
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
        Pose startPoseAdded;
        Pose spike2Added;
        Pose redBasketAdded;

        //Paths
        private Path toSpike;
        private Path toBasket;

        @Override
        public void buildPaths(Pose offset) {
            this.offset.add(offset);

            startPoseAdded = startPose.addReturn(offset);
            spike2Added = leftOuterSpike2.addReturn(offset);
            redBasketAdded = redBasket.addReturn(offset);

            toSpike = robot.follower.linearPathBuilder(startPoseAdded, spike2Added);
            toBasket = robot.follower.linearPathBuilder(spike2Added, redBasketAdded);
        }


        @Override
        public Pose getEndPoseEst() {
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
                        robot.stateMachine.goHighBasket(true, false, false, false);
                        setPathState(3);
                    } else if (pathTimer.getElapsedTimeSeconds() > 2) {
                        robot.doDriveShake = false;
                        robot.follower.followPath(toBasket);
                        robot.intakeSystem.storeOutPos();
                        isFinished = true;
                        setPathState(5);
                    }
                    break;
                case 3:
                    if (!robot.stateMachine.goingHighBasket()) {
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

        public void addToOffset(Pose offset) {
            this.offset = offset;
        }

        //todo low priority
        public Pose getOffset() {
            return offset.addReturn(new Pose());
        }

        public String getName() {
            return "From Red Basket To Spike 2, " + state;
        }
    }

    public static class ToPickupAndPlaceSpike3 implements PathPlanner {
        /// Expects intake pre extended
        /// Ends at basket place pos or store with outake if we missed the spike and
        /// Fine with being at store preset
        /// has the intake extended if pre extend

        // Variables
        Pose offset = new Pose();
        private Timer pathTimer;
        private int state = 0;
        private int transferState = -1;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        public ToPickupAndPlaceSpike3(Robot robot, Pose startPose, Pose offset) {
            addToOffset(offset);
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
        }
        public ToPickupAndPlaceSpike3(Robot robot, Pose startPose) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
        }
        //Poses
        //todo low priority
        Pose startPoseAdded;
        Pose spike3Added;
        Pose redBasketAdded;

        //Paths
        private Path toSpike;
        private Path toBasket;

        @Override
        public void buildPaths(Pose offset) {
            this.offset.add(offset);

            startPoseAdded = startPose.addReturn(offset);
            spike3Added = leftOuterSpike3.addReturn(offset);
            redBasketAdded = redBasket.addReturn(offset);

            toSpike = robot.follower.linearPathBuilder(startPoseAdded, spike3Added);
            toBasket = robot.follower.linearPathBuilder(spike3Added, redBasketAdded);
        }


        @Override
        public Pose getEndPoseEst() {
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
                        robot.stateMachine.goHighBasket(true, false, false, false);
                        setPathState(3);
                    } else if (pathTimer.getElapsedTimeSeconds() > 2) {
                        robot.doDriveShake = false;
                        robot.follower.followPath(toBasket);
                        robot.intakeSystem.storeOutPos();
                        isFinished = true;
                        setPathState(5);
                    }
                    break;
                case 3:
                    if (!robot.stateMachine.goingHighBasket()) {
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

        //todo low priority
        public Pose getOffset() {
            return offset.addReturn(new Pose());
        }

        public void addToOffset(Pose offset) {
            this.offset = offset;
        }


        public String getName() {
            return "From Red Basket To Spike 3, " + state;
        }
    }


    public static class ToPickupAndPlaceSubYellow implements PathPlanner {
        /// Starts at basket with intake NOT extended
        /// Ends at basket with outake at basket
        /// Fine with being at store preset
        /// If we run out of time, we will go park and not continue other steps
        // Variables
        Pose offset = new Pose();
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;
        private Pose2D target2D;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        public ToPickupAndPlaceSubYellow(Robot robot, Pose startPose, Pose offset) {
            addToOffset(offset);
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
        }
        public ToPickupAndPlaceSubYellow(Robot robot, Pose startPose) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
        }
        //Poses
        private final Pose toSubControlPoint = new Pose(-51, 51, 0);
        private final Pose toBasketControlPoint = new Pose(-51, 51, 0);

        Pose startPoseAdded;
        Pose toSubControlPointAdded;
        Pose intakeSubLeftMiddleAdded;
        Pose toBasketControlPointAdded;
        Pose redBasketAdded;

        //Paths
        Path toSub;
        Path toBasket;

        @Override
        public void buildPaths(Pose offset) {
            this.offset.add(offset);

            startPoseAdded = startPose.addReturn(offset);
            toSubControlPointAdded = toSubControlPoint.addReturn(offset);
            intakeSubLeftMiddleAdded = intakeSubLeftMiddle.addReturn(offset);
            toBasketControlPointAdded = toBasketControlPoint.addReturn(offset);
            redBasketAdded = redBasket.addReturn(offset);

            toSub = new Path(new BezierCurve(new Point(startPoseAdded), new Point(toSubControlPointAdded), new Point(intakeSubLeftMiddleAdded)));
            toSub.setLinearHeadingInterpolation(startPoseAdded.getHeading(), intakeSubLeftMiddleAdded.getHeading());

            toBasket = new Path(new BezierCurve(new Point(intakeSubLeftMiddleAdded), new Point(toBasketControlPointAdded), new Point(redBasketAdded)));
            toBasket.setLinearHeadingInterpolation(intakeSubLeftMiddleAdded.getHeading(), redBasketAdded.getHeading());
        }


        @Override
        public Pose getEndPoseEst() {
            return redBasket;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    robot.follower.followPath(toBasket);
                    robot.outtakeSystem.setArmPos(Constants.Outtake.intakeArm);
                    setPathState(1);
                    break;
                case 1:
                    if (robot.follower.getErrorDistance(intakeSubLeftMiddle) < 2) {
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
                        robot.stateMachine.goHighBasket(true, false, false, false);
                        setPathState(6);
                    }
                    break;
                case 6:
                    if (!robot.stateMachine.goingHighBasket() && robot.follower.getError(redBasketAdded).getX() < 1 && robot.follower.getError(redBasketAdded).getY() < 1 || pathTimer.getElapsedTimeSeconds() > 5) {
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
                    if (pathTimer.getElapsedTimeSeconds() > 0.1 && robot.follower.getError(intakeSubLeftMiddleAdded).getX() < 2) {
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

        //todo low priority
        public Pose getOffset() {
            return offset.addReturn(new Pose());
        }

        public void addToOffset(Pose offset) {
            this.offset = offset;
        }

        public String getName() {
            return "From Red Basket To Sub Place, " + state;
        }
    }


    public static class ToPickupColorFromSubToPickupWall implements PathPlanner {
        /// Starts at basket with intake NOT extended
        /// Ends at pickup wall with block in claw
        /// Fine with being at store preset
        /// +1 color and -1 piece to human player
        /// Has an option to go left or right wall
        // Variables
        Pose offset = new Pose();
        private Timer wristTimer;
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;
        private Pose2D visionResult;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        private boolean leftWall;
        public ToPickupColorFromSubToPickupWall(Robot robot, Pose startPose, boolean leftWall, Pose offset) {
            addToOffset(offset);
            pathTimer = new Timer();
            wristTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.leftWall = leftWall;
        }
        public ToPickupColorFromSubToPickupWall(Robot robot, Pose startPose, boolean leftWall) {
            pathTimer = new Timer();
            wristTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.leftWall = leftWall;
        }
        //Poses
        //todo low priority
        private final Pose toSubControlPoint = new Pose();

        private final Pose toRightWallControlPoint1 = new Pose();
        private final Pose toRightWallControlPoint2 = new Pose();

        private final Pose toLeftWallControlPoint1 = new Pose();
        private final Pose toLeftWallControlPoint2 = new Pose();


        Pose startPoseAdded;
        Pose toSubControlPointAdded;
        Pose intakeSubLeftMiddleAdded;
        Pose toLeftWallControlPoint1Added;
        Pose toLeftWallControlPoint2Added;
        Pose pickupWallLeftAdded;
        Pose toRightWallControlPoint1Added;
        Pose toRightWallControlPoint2Added;
        Pose pickupWallRightAdded;


        private Pose tartgetPose;

        //Paths
        Path toSub;
        Path toWall;


        @Override
        public void buildPaths(Pose offset) {
            this.offset.add(offset);

            startPoseAdded = startPose.addReturn(offset);
            toSubControlPointAdded = toSubControlPoint.addReturn(offset);
            intakeSubLeftMiddleAdded = intakeSubLeftMiddle.addReturn(offset);
            toLeftWallControlPoint1Added = toLeftWallControlPoint1.addReturn(offset);
            toLeftWallControlPoint2Added = toLeftWallControlPoint2.addReturn(offset);
            pickupWallLeftAdded = pickupWallLeft.addReturn(offset);
            toRightWallControlPoint1Added = toRightWallControlPoint1.addReturn(offset);
            toRightWallControlPoint2Added = toRightWallControlPoint2.addReturn(offset);
            pickupWallRightAdded = pickupWallRight.addReturn(offset);

            toSub = new Path(new BezierCurve(startPoseAdded, toSubControlPointAdded, intakeSubLeftMiddleAdded));
            toSub.setLinearHeadingInterpolation(startPoseAdded, intakeSubLeftMiddleAdded);

            if (leftWall) {
                toWall = new Path(new BezierCurve(startPoseAdded, toLeftWallControlPoint1Added, toLeftWallControlPoint2Added, pickupWallLeftAdded));
                toWall.setLinearHeadingInterpolation(startPoseAdded.getHeading() + Math.toRadians(45), pickupWallLeftAdded.getHeading(), 0.75);
                tartgetPose = pickupWallLeftAdded;
            } else {
                toWall = new Path(new BezierCurve(startPoseAdded, toRightWallControlPoint1Added, toRightWallControlPoint2Added, pickupWallRightAdded));
                toWall.setLinearHeadingInterpolation(startPoseAdded.getHeading() + Math.toRadians(45), pickupWallRightAdded.getHeading(), 0.68);
                tartgetPose = pickupWallRightAdded;
            }
        }


        @Override
        public Pose getEndPoseEst() {
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
                        robot.stateMachine.goWall(false, false, robot.robotState.whereAmI == PlacePosEnum.intake);
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (robot.follower.getErrorDistance(intakeSubLeftMiddleAdded) < 1.5) {
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

        public void addToOffset(Pose offset) {
            this.offset = offset;
        }

        //todo low priority
        public Pose getOffset() {
            return offset.addReturn(new Pose());
        }

        public String getName() {
            return "From Red Basket To Intake Color To Wall, " + state;
        }
    }

    public static class ToPickupWall implements PathPlanner {
        /// Starts at basket with intake NOT extended
        /// Ends at pickup wall with block in claw
        /// -1 piece to human player
        /// Has an option to go left or right wall
        // Variables
        Pose offset = new Pose();
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
            this.offset.add(offset);

            Pose startPoseAdded = startPose.addReturn(offset);
            Pose redBasketAdded = redBasket.addReturn(offset);
            Pose pickupWallLeftAdded = pickupWallLeft.addReturn(offset);
            Pose pickupWallRightAdded = pickupWallRight.addReturn(offset);

            if (leftWall) {
                Pose halfWayWallPose = new Pose(redBasketAdded.getX(), (startPoseAdded.getY() + pickupWallLeftAdded.getY()) / 2, Math.toRadians(0));
                Pose halfWayWallPoseAdded = halfWayWallPose.addReturn(offset);

                toWall = robot.follower.pathBuilder()
                        .addPath(new BezierCurve(startPoseAdded, halfWayWallPoseAdded))
                        .setLinearHeadingInterpolation(startPoseAdded.getHeading(), halfWayWallPoseAdded.getHeading(), 0.175)
                        .addPath(new BezierCurve(halfWayWallPoseAdded, pickupWallLeftAdded))
                        .setLinearHeadingInterpolation(halfWayWallPoseAdded.getHeading(), pickupWallLeftAdded.getHeading(), 0.5)
                        .build();

                tartgetPose = pickupWallLeftAdded;
            } else {
                Pose halfWayWallPose = new Pose(redBasketAdded.getX(), (startPoseAdded.getY() + pickupWallRightAdded.getY()) / 2, Math.toRadians(0));
                Pose halfWayWallPoseAdded = halfWayWallPose.addReturn(offset);

                toWall = robot.follower.pathBuilder()
                        .addPath(new BezierCurve(startPoseAdded, halfWayWallPoseAdded))
                        .setLinearHeadingInterpolation(startPoseAdded.getHeading(), halfWayWallPoseAdded.getHeading(), 0.175)
                        .addPath(new BezierCurve(halfWayWallPoseAdded, pickupWallRightAdded))
                        .setLinearHeadingInterpolation(halfWayWallPoseAdded.getHeading(), pickupWallRightAdded.getHeading(), 0.5)
                        .build();

                tartgetPose = pickupWallRightAdded;
            }
        }


        @Override
        public Pose getEndPoseEst() {
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
                    if (pathTimer.getElapsedTimeSeconds() > 1) {
                        robot.stateMachine.goWall(false, false, robot.robotState.whereAmI == PlacePosEnum.intake);
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
                    if (pathTimer.getElapsedTimeSeconds() > 0.05) {
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

        public void addToOffset(Pose offset) {
            this.offset = offset;
        }


        //todo low priority
        public Pose getOffset() {
            return offset.addReturn(new Pose());
        }

        public String getName() {
            return "From Red Basket To Wall, " + state;
        }
    }
}
