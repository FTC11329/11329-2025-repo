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
        public static double servoRelease = 0.267;
        public static double servoClimb = 0.427;
        public static double speed = 1;

        public static int motorClimb = 1100;
        public static int motorDrop = 0;

        public static double p = 0.03;
        public static double i = 0;
        public static double d = 0.00001;
    }
    @Config
    public static class Climber {
        public static final int inPos = -8000; //-1000;
        public static final int outPos  = 22000; //todo
    }

    @Config
    public static class Intake {
        public static int minSlidePos = 0;
        public static int safeTransferSlide = 333;
        public static int minWhileDownPos = 477;
        public static int intakeSlidePos = 1325;
        public static int maxSlidePos = 1600;
        public static double manualSlideSpeed = 30;

        public static double wristStore = 0;
        public static double wristSpit = 0.30;
        public static double wristClear = 0.35;
        public static double wristDown = 0.403;

        public static double intakeSpeed = 1;
        public static double transferSpeed = 0.5;
        public static double unjamSpeed = -0.5; //Todo: set
        public static double spitSpeed = -0.75; //Todo: set
        public static int bitMore = 80;
    }

    @Config
    public static class Outtake {
        public static double grabClaw = 0.46;
        public static double halfClaw = 0.4;
        public static double dropClaw = 0.25;

        public static double intakeArm = 0.9928;
        public static double initArm = 0.5;
        public static double upArm = 0.33333333;
        public static double preTransferArm = 0.666666;
        public static double intakeWallArm = 0;
        public static double manualArmSpeed = 0.005;

        public static double specimenArm = 0.50;
        public static double specimenArmEnd = 0.40;
        public static double basketArm = 0.165;

        //435
        public static int intakeSlides = 574;
        public static int intakeWallSlides = 0;
        public static int safeFromWallSlides = 166;
        public static int safeFromHSlides = 947;
        public static int highSpecimenSlides = 680;
        public static int endSpecimenSlides = 750;
        public static int lowBasketSlides = 1236;
        public static int highBasketSlides = 2522;
        public static int maxSlides = 2670;
        public static double manualSlideSpeed = 20;

        /*
        1150
        public static int intakeSlides = 247;
        public static int intakeWallSlides = 0;
        public static int safeFromWallSlides = 100;
        public static int safeFromHSlides = 400; //Todo: set
        public static int highSpecimenSlides = 258;
        public static int lowBasketSlides = 424;
        public static int highBasketSlides = 948;
        public static int maxSlides = 1050;
         */


        public static double p = 0.03;
        public static double i = 0;
        public static double d = 0.00001;
        public static double f = 0;

    }
    @Config
    public static class Color {
        public static double[] blue = {0.0018, 0.0042, 0.0114,0.68};
        public static double[] red = {0.009, 0.0051, 0.003, 0.68};
        public static double[] yellow = {0.0112, 0.0169, 0.0045, 0.88};
        public static double[] empty = {0.0005, 0.0013, 0.0017, 0.079};

        public static double hasDistance = 1; //1.44 0.47 0.54 0.62
    }

    @Config
    public static class RoadRunner {
        public static double inPerTick = 0.000526976391;
        public static double lateralInPerTick = 0.00030531911012706043;
        public static double trackWidthTicks = 20846.185113581167;

        public static double kS = 1.7168128242964098;
        public static double kV = 0.00007007307591705226;
        public static double kA = 0.000024;

        //TODO: tune these values if needed
        public static double axialGain = 5.25;
        public static double lateralGain = 6;
        public static double headingGain = 4; // shared with turn
    }
}
