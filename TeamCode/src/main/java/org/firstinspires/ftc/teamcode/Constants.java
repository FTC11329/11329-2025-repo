package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {

    @Config
    public static class Drivetrain {
        public static double fastSpeed = 1;
        public static double slowSpeed = 0.5;
    }

    @Config
    public static class PTO {
        public static double PTOServoReleaseRight = 0.3348;
        public static double PTOServoReleaseLeft = 0.317;
        public static double PTOServoClimbRight = 0.4127;
        public static double PTOServoClimbLeft = 0.4035;
        public static double speed = 1;

        public static int motorClimb = 1700;
        public static int motorDrop = 0;

        public static double p = 0.03;
        public static double i = 0;
        public static double d = 0.00001;
    }
    @Config
    public static class Climber {
        public static final int inPos = -800;
        public static final int hookPos = 8100;
        public static final int outPos  = 9800;
    }

    @Config
    public static class Intake {
        public static int minSlidePos = 0;
        public static int transferSlides = 275;
        public static int minWhileDownPos = 375;
        public static int autoHSlides = 1050;
        public static int autoPreExtendSlides = 500;
        public static int intakeSlidePos = 1325;
        public static int maxSlidePos = 1450;
        public static double inchToTick = 1532.0 / 19.0;
        public static double tickToInch = 19.0 / 1532.0;
        public static double manualSlideSpeed = 75;

        public static double wristClimb = 0;
        public static double wristStore = 0.03;
        public static double wristClear = 0.25;
        public static double wristDown = 0.450;

        public static double depoStore = 0.0645;
        public static double depoDepo = 0.444;

        public static double intakeSpeed = 0.9;
        public static double transferSpeed = 1;
        public static double unjamSpeed = -1;
        public static double spitSpeed = -0.75;
        public static int bitMore = 80;
        public static double unjamTimeMillisTeleop = 125;
        public static double unjamTimeMillisAuto = 100;
        public static double intakeServoSpeedTime = 600;
        public static double transferToTrayMillis = 600;
    }

    @Config
    public static class Outtake {
        public static double grabClaw = 0.59;
        public static double dropClaw = 0.21;

        public static double intakeArm = 0.962;
        public static double initTeleopArm = 0.49;
        public static double initAutoSampArm = 0.861;
        public static double initAutoSpecArm = 0.0326;
        public static double autoArmClear = 0.814;
        public static double preTransferArm = 0.744; // Depreciated
        public static double upArm = 0.49;
        public static double intakeWallArm = 0.0657;
        public static double manualArmSpeed = 0.01;

        public static double specimenArm = 0.76;
        public static double basketArm = 0.2825;
        public static double frontBasketArm = 0.6585; // needs updated
        public static double parkArm = 0.7654;

        public static int intakeSlides = 195;
        public static int intakeWaitSlides = 450; //not used anymore
        public static int intakeWallSlides = 170;
        public static int intakeWallAutoSlides = 180;

        public static int safeFromWallSlides = 360;
        public static int safeFromHSlides = 650;
        public static int safeFromClimberBar = 1300;
        public static int safeFromSpecBar = 1600;

        public static int highSpecimenSlides = 1017;
        public static int lowBasketSlides = 625;
        public static int highBasketSlides = 1975;

        public static int maxSlides = 1975;
        public static int climbSlides = 1975;

        public static double manualSlideSpeed = 60;

        public static double seesWallDistance = 1.35;
        public static double seesTransferDistance = 1.1;
        public static double p = 0.03;
        public static double i = 0;
        public static double d = 0.00001;
        public static double f = 0;

    }
    @Config
    public static class Color {
        public static double[] blue = {0.0014, 0.0035, 0.0094, 0.597};
        public static double[] red = {0.0075, 0.0047, 0.0028, 0.621};
        public static double[] redEdge = {0.0157, 0.0108, 0.0065, 0.8877};
        public static double[] yellow = {0.0123, 0.0197, 0.0046, 0.907};
        public static double[] empty = {0.0005, 0.0014, 0.002, 0.10};

        public static double hasDistance = 0.8; //1.27 0.605 0.495 0.463
    }
}
