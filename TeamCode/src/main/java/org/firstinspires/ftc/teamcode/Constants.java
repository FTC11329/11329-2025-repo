package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

public class Constants {

    @Config
    public static class Drivetrain {
        public static double fastSpeed = 1;
        public static double slowSpeed = 0.5;
    }

    @Config
    public static class PTO {
        public static double PTOServoRelease = 0.28;
        public static double PTOServoClimb = 0.6;
        public static double speed = 1;

        public static int motorClimb = 1700;
        public static int motorDrop = 0;

        public static double p = 0.03;
        public static double i = 0;
        public static double d = 0.00001;
    }
    @Config
    public static class Climber {
        public static final int inPos = -500;
        public static final int hookPos = 6700;
        public static final int outPos  = 8000;
    }

    @Config
    public static class Intake {
        public static int minSlidePos = 0;
        public static int safeTransferSlide = 200;
        public static int minWhileDownPos = 375;
        public static int autoHSlides = 1050;
        public static int autoPreExtendSlides = 500;
        public static int intakeSlidePos = 1325;
        public static int maxSlidePos = 1450;
        public static double manualSlideSpeed = 75;

        public static double wristStore = 0;
        public static double wristClear = 0.25;
        public static double wristDown = 0.504;

        public static double intakeSpeed = 0.9;
        public static double transferSpeed = 0.9;
        public static double unjamSpeed = -0.7;
        public static double spitSpeed = -0.75;
        public static int bitMore = 80;
        public static double unjamTimeMillis = 150; //todo make an auto vs teleop times
    }

    @Config
    public static class Outtake {
        public static double grabClaw = 0.59;
        public static double dropClaw = 0.21;

        public static double intakeArm = 0.949;
        public static double initTeleopArm = 0.43;
        public static double initAutoArm = 0.80;
        public static double preTransferArm = 0.690666;
        public static double upArm = 0.43;
        public static double intakeWallArm = 0.024;
        public static double manualArmSpeed = 0.01;

        public static double specimenArm = 0.71;
        public static double basketArm = 0.242;
        public static double autoArmClimb = 0.628;
        public static double parkArm = 0.609;

        public static int intakeSlides = 210;
        public static int intakeWaitSlides = 450;
        public static int intakeWallSlides = 135;
        public static int intakeWallAutoSlides = 135;

        public static int safeFromWallSlides = 360;
        public static int safeFromHSlides = 650;
        public static int safeFromClimberBar = 1200;
        public static int safeFromSpecBar = 1712;

        public static int highSpecimenSlides = 1050;
        public static int lowBasketSlides = 625;
        public static int highBasketSlides = 1975;

        public static int maxSlides = 1975;
        public static int climbSlides = 1975;

        public static double manualSlideSpeed = 60;

        public static double p = 0.03;
        public static double i = 0;
        public static double d = 0.00001;
        public static double f = 0;

    }
    @Config
    public static class Color {
        public static double[] blue = {0.0018, 0.0041, 0.0127, 0.715};
        public static double[] red = {0.0091, 0.0047, 0.0028, 0.667};
        public static double[] yellow = {0.0135, 0.0192, 0.0047, 0.91};
        public static double[] empty = {0.0004, 0.0012, 0.0023, 0.097};

        public static double hasDistance = 1; //1.44 0.47 0.54 0.62
    }
}
