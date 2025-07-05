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
        public static final int inPos = -875;
        public static final int hookPos =  8100;
        public static final int outPos  = 10200;
        public static final int prePos  = 8130;
    }

    @Config
    public static class Intake {
        public static int minSlidePos = 0;
        public static int transferSlides = 328;
        public static int minWhileDownPos = 375;
        public static int autoHSlides = 1050;
        public static int autoPreExtendSlides = 900;
        public static int intakeSlidePos = 1325;
        public static int maxSlidePos = 1450;
        public static double inchToTick = 1532.0 / 19.0;
        public static double tickToInch = 19.0 / 1532.0;
        public static double manualSlideSpeed = 75;

        public static double wristClimb = 0;
        public static double wristStore = 0.01;
        public static double wristDepo = 0.12;
        public static double wristClear = 0.25;
        public static double wristDown = 0.450;

        public static double depoStore = 0.075;
        public static double depoDepo = 0.444;

        public static double intakeSpeed = 0.9;
        public static double transferSpeed = 1;
        public static double unjamSpeed = -0.9;
        public static double spitSpeed = -0.75;
        public static int unjamTicksShort = -150;
        public static int unjamTicksLong = -150;
        public static double unjamTimeMillisTeleop = 110;
        public static double unjamTimeMillisAuto = 130;
        public static double intakeServoSpeedTime = 600;
        public static double transferToTrayMillis = 600;
    }

    @Config
    public static class Outtake {
        //todo presets
        public static double initTeleopWrist = 0;
        public static double initAutoUnderBarWrist = 0;
        public static double initAutoNearWallWrist = 0;

        public static double preClipSpecimenWrist = 0;
        public static double postClipSpecimenWrist = 0;

        public static double basketWrist = 0;

        public static double storeWrist = 0;

        public static double wallWrist = 0;
        //                   Walrus?
        public static double manualWristSpeed = 0;


        public static double grabClaw = 0.59;
        public static double dropClaw = 0.21;

        public static double intakeArm = 0.946; //0.95
        public static double initTeleopArm = 0.49;
        public static double initAutoUnderBarArm = 0.861;
        public static double initAutoNearWallArm = 0.032;
        public static double autoArmClear = 0.814;
        public static double preTransferArm = 0.744; // Depreciated
        public static double upArm = 0.49;
        public static double intakeWallArm = 0.0544;
        public static double manualArmSpeed = 0.01;

        public static double downArm = 1;
        public static double lowSpecimenArm = 0.76;
        public static double highSpecimenArm = 0.76;
        public static double basketArm = 0.295;
        public static double basketArmHigh = 0.288;
        public static double frontBasketArm = 0.6585; // needs updated
        public static double parkArm = 0.7654;

        public static int intakeSlides = 175;
        public static int intakeWaitSlides = 450; //not used anymore
        public static int intakeWallSlides = 185;
        public static int intakeWallAutoSlides = 180;

        public static int safeFromWallSlides = 360;
        public static int safeFromHSlides = 650;
        public static int safeFromClimberBar = 1300;
        public static int safeFromSpecBar = 1600;

        public static int lowSpecimenSlides = 1010;
        public static int highSpecimenSlides = 1010;
        public static int lowBasketSlides = 625;
        public static int highBasketSlides = 1975;

        public static int maxSlides = 1975;
        public static int climbSlides = 1975;

        public static double manualSlideSpeed = 60;

        public static double seesWallDistanceBlue = 1.7;
        public static double seesWallDistanceRed = 1.7;
        public static double seesTransferDistance = 1.42;
        //todo
        public static double leftFlapUp = 0;
        public static double leftFlapWall = 0.3482;
        public static double leftFlapSpike = 0.4573;

        public static double rightFlapUp = 0;
        public static double rightFlapWall = 0;
        public static double rightFlapSpike = 0;

        public static double p = 0.03;
        public static double i = 0;
        public static double d = 0.00001;
        public static double f = 0;

    }
    @Config
    public static class Color {
        public static double[] blue = {0.0016, 0.0041, 0.0114, 0.6806};
        public static double[] red = {0.0067, 0.0042, 0.0025, 0.5654};
        public static double[] redEdge = {0.016, 0.0122, 0.0055, 0.886};
        public static double[] yellow = {0.0123, 0.0189, 0.0046, 0.903};
        public static double[] yellowEdge = {0.0207, 0.0277, 0.007, 0.0957};
        public static double[] empty = {0.0004, 0.0013, 0.002, 0.0863};

        public static double hasDistance = 0.75; //1.27 0.605 0.495 0.463
    }
}
