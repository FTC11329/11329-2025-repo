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
        public static final int prePos  = 7517;
    }

    @Config
    public static class Intake {
        public static int minSlidePos = 0;
        public static int transferSlides = 158;
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
        public static double wristClear = 0.23;
        public static double wristTransfer = 0.203;
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
        //Wrist
        public static double minWrist = 0.06;
        public static double maxWrist = 0.894;
        public static double straightWrist = 0.486;

        public static double initTeleopWrist = straightWrist;
        public static double initAutoUnderBarWrist = minWrist;
        public static double initAutoNearWallWrist = 0.555;

        public static double highSpecimenWristAutoPre = 0.237;
        public static double highSpecimenWristAutoPost = 0.25;
        public static double lowSpecimenWristAuto = 0.21;
        public static double lowSpecimenWristAutoPost = 0.52;

        public static double safeHighSpecimenWrist = maxWrist;
        public static double preClipHighSpecimenWrist = 0.237;
        public static double postClipHighSpecimenWrist = 0.39;

        public static double safeLowSpecimenWrist = 0.497;
        public static double preClipLowSpecimenWrist = 0.497;
        public static double postClipLowSpecimenWrist = 0.008;

        public static double basketWrist = 0.542;

        public static double intakeWrist = 0.609;

        public static double wallWrist = 0.26;
        //                   Walrus?

        public static double manualWristSpeed = 0.01;

        //Claw
        public static double grabClaw = 0.6;
        public static double dropClaw = 0.25;

        //Arm
        public static double intakeArm = 0.89;
        public static double initTeleopArm = 0.4446;
        public static double initAutoUnderBarArm = 0.786 + 0.083;
        public static double initAutoNearWallArm = 0.037; // min arm
        public static double upArm = initTeleopArm;
        public static double intakeWallArm = 0.048;
        public static double manualArmSpeed = 0.01;

        public static double downArm = 0.94;

        public static double highSpecimenArmAuto = 0.516;
        public static double lowSpecimenArmAutoPre = 0.754;
        public static double lowSpecimenArmAutoPost = 0.72;

        public static double lowSpecimenArm = 0.666 + 0.083;
        public static double safeLowSpecimenArm = 0.782 + 0.083;
        public static double postLowSpecimenArm = 0.733 + 0.083;
        public static double highSpecimenArm = 0.671;
        public static double safeHighSpecimenArm = 0.75 + 0.083;
        public static double postHighSpecimenArm = 0.7352 + 0.083;

        public static double safeBasketArm = 0.296;
        public static double basketArm = 0.286;
        public static double frontBasketArm = 0.5661 + 0.083;
        public static double parkArm = 0.538 + 0.083;

        //VSlides
        public static int safeAtIntakeSlides = 343;

        public static int intakeSlides = 125;
        public static int intakeWallSlides = 67;
        public static int intakeWallAutoSlides = 67;

        public static int safeFromWallSlides = 200;
        public static int safeFromClimberBar = 597;
        public static int safeFromSpecBar = 710;

        public static int highSpecimenSlidesAutoPre = 5;
        public static int highSpecimenSlidesAutoPost = 315;
        public static int lowSpecimenSlidesAutoPre = 202;
        public static int lowSpecimenSlidesAutoPost = 202;

        public static int lowSpecimenSlides = 0;
        public static int postClipLowSpecimenSlides = 182;
        public static int safeLowSpecimenSlides = 271;
        public static int highSpecimenSlides = 440;
        public static int postClipHighSpecimenSlides = 800;
        public static int safeHighSpecimenSlides = 650;
        public static int lowBasketSlides = 228;
        public static int highBasketSlides = 840;

        public static int maxSlides = 840;
        public static int climbSlides = 840;

        public static double manualSlideSpeed = 25;

        public static int intakeWaitSlides = 450; //not used anymore
        public static int safeFromHSlides = 650; //not used anymore
        // Outtake Sensor
        public static double seesWallDistanceBlue = 1.3;
        public static double seesWallDistanceRed = 1.1;
        public static double seesTransferDistance = 1.42;
        //Back Flaps
        public static double flapsUp = 0.044;
        public static double flapsSpikeClear = 0.202;
        public static double flapsWall = 0.352;

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
