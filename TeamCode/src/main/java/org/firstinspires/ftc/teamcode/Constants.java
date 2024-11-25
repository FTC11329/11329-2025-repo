package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

public class Constants {

    @Config
    public static class Drivetrain {
        public static double fastSpeed = 0.7;
        public static double slowSpeed = 0.3;
    }

    @Config
    public static class RoadRunner {
        public static double inPerTick = 0.000526976391;

        public static double kS = 1.7168128242964098;
        public static double kV = 0.00007007307591705226;
        public static double kA = 0.000024;
    }
}
