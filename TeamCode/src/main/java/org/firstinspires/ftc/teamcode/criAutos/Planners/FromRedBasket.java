package org.firstinspires.ftc.teamcode.criAutos.Planners;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;
import static org.firstinspires.ftc.teamcode.criAutos.CommonPoses.*;
import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.BezierCurve;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Path;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Point;
import org.firstinspires.ftc.teamcode.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.utility.Robot;
import org.firstinspires.ftc.teamcode.utility.SampleAutoEnum;

public class FromRedBasket {
    public static class ToPickupAndPlaceSpike1 implements PathPlanner {
        /// Expects intake pre extended
        /// Ends at basket place pos or store with outake if we missed the spike and
        /// has the intake extended if pre extend

        // Variables
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
            buildPaths();
        }
        //Poses
        //todo:
        private final Pose spike1 = new Pose();

        //Paths
        private Path toSpike;
        private Path toBasket;
        public void buildPaths() {
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
                        robot.doTransfer = true;
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
                    if (!robot.doTransfer) {
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
    }
    public static class ToPickupAndPlaceSpike2 implements PathPlanner {
        /// Expects intake pre extended
        /// Ends at basket place pos or store with outake if we missed the spike and
        /// has the intake extended if pre extend

        // Variables
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
            buildPaths();
        }
        //Poses
        //todo:
        private final Pose spike2 = new Pose();

        //Paths
        private Path toSpike;
        private Path toBasket;
        public void buildPaths() {
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
                        robot.doTransfer = true;
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
                    if (!robot.doTransfer) {
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
    }

    public static class ToPickupAndPlaceSpike3 implements PathPlanner {
        /// Expects intake pre extended
        /// Ends at basket place pos or store with outake if we missed the spike and
        /// has the intake extended if pre extend

        // Variables
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
            buildPaths();
        }
        //Poses
        //todo:
        private final Pose spike3 = new Pose();

        //Paths
        private Path toSpike;
        private Path toBasket;
        public void buildPaths() {
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
                        robot.doTransfer = true;
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
                    if (!robot.doTransfer) {
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
    }



    public static class ToPickupAndPlaceSubYellow implements PathPlanner {
        /// Starts at basket with intake NOT extended
        /// Ends at basket with outake at basket
        /// If we run out of time, we will go park and not continue other steps
        // Variables
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
            buildPaths();
        }
        //Poses
        //todo:
        private final Pose toSubControlPoint = new Pose();
        private final Pose toBasketControlPoint = new Pose();

        //Paths
        Path toSub;
        Path toBasket;
        public void buildPaths() {
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
                case 1:
                    if (robot.follower.getErrorDistance(intakeSubLeftInner) < 2) {
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
                        robot.doTransfer = true;
                        setPathState(6);
                    }
                    break;
                case 6:
                    if (!robot.doTransfer && robot.follower.getError(redBasket).getX() < 1 && robot.follower.getError(redBasket).getY() < 1 || pathTimer.getElapsedTimeSeconds() > 5) {
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
                    if (pathTimer.getElapsedTimeSeconds() > 0.1 && robot.follower.getError(intakeSubLeftInner).getX() < 2) {
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
    }
}
