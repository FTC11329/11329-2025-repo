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
        public static double speed = 1; //Todo: set

        public static int motorClimb = 1020;
        public static int motorDrop = 0;

        public static double p = 0.03;
        public static double i = 0;
        public static double d = 0.00001;
    }

    @Config
    public static class Intake {
        public static int minSlidePos = 0; //Todo: set
        public static int minWhileDownPos = 310;
        public static int intakeSlidePos = 1325;
        public static int maxSlidePos = 1850;

        public static double wristUp = 0;
        public static double wristDown = 0.3737;

        public static double intakeSpeed = 1;
        public static int bitMore = 80;
    }

    @Config
    public static class Outtake {
        public static double grabClaw = 0.46; //Todo: set
        public static double dropClaw = 0.25; //Todo: set

        public static double intakeArm = 0.333; //TODO
        public static double initArm = 0.33333333;
        public static double intakeWallArm = 0.027;

        public static double specimenArm = 0.57;
        public static double specimenArmEnd = 0.40;
        public static double basketArm = 0; //Todo: set

        public static int intakeSlides = 10; //Todo: set
        public static int intakeWallSlides = 33;
        public static int lowSpecimenSlides = 0; //Todo: set
        public static int highSpecimenSlides = 217;
        public static int lowBasketSlides = 0; //Todo: set
        public static int highBasketSlides = 637;
    }
    @Config
    public static class Color {
        public static double[] blue = {0.0012, 0.0027, 0.0085, 0.5291} ;
        public static double[] red = {0.0072, 0.0038, 0.0022, 0.5737} ;
        public static double[] yellow = {0.011, 0.0165, 0.0042, 0.8804} ;

        public static double hasDistance = 1; //0.61 0.47 0.51 1.45
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
