package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

public class Constants {

    @Config
    public static class Drivetrain {
        public static double fastSpeed = 0.7;
        public static double slowSpeed = 0.3;
    }

    @Config
    public static class PTO {
        public static double servoRelease = 0.267;
        public static double servoClimb = 0.427;
        public static double speed = 1; //Todo: set

        public static int motorClimb = 0; //Todo: set
        public static int motorDrop = 0; //Todo: set

        public static double p = 0.03;
        public static double i = 0;
        public static double d = 0;
    }

    @Config
    public static class Intake {
        public static int minSlidePos = 0; //Todo: set
        public static int minWhileDownPos = 0; //Todo: set
        public static int maxSlidePos = 0; //Todo: set

        public static double wristUp = 0; //Todo: set
        public static double wristDown = 0; //Todo: set

        public static double intakeSpeed = 0; //Todo: set
    }

    public static class Outtake {
        public static double grabClaw = 0; //Todo: set
        public static double dropClaw = 0; //Todo: set

        public static double specimenArm = 0; //Todo: set
        public static double basketArm = 0; //Todo: set

        public static int lowSpecimenSlides = 0; //Todo: set
        public static int highSpecimenSlides = 0; //Todo: set
        public static int lowBasketSlides = 0; //Todo: set
        public static int highBasketSlides = 0; //Todo: set
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
