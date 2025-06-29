package org.firstinspires.ftc.teamcode.criAutos.Planners;

import static org.firstinspires.ftc.teamcode.criAutos.CommonPoses.*;
import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.BezierCurve;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Path;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Point;
import org.firstinspires.ftc.teamcode.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.utility.Robot;

public class ParkPaths {
    //todo
    private static class ParkFrom implements PathPlanner {

        /// Goes to park without intaking
        // Variables
        Pose offset;
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;

        public ParkFrom(Robot robot, Pose startPose) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
        }
        //Poses
        //todo
        private final Pose toSubControlPoint = new Pose();

        //Paths
        Path toSub;


        @Override
        public void buildPaths(Pose offset) {
            this.offset = offset;
            toSub = new Path(new BezierCurve(new Point(startPose), new Point(toSubControlPoint), new Point(intakeSubLeftOuter)));
            toSub.setLinearHeadingInterpolation(startPose.getHeading(), intakeSubLeftOuter.getHeading());
        }

        @Override
        public Pose getEndPoseEst() {
            //todo
            return new Pose();
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
            }

            return isFinished;
        }

        public void setPathState(int state) {
            this.state = state;
            pathTimer.resetTimer();
        }

        //todo
        public Pose getOffset() {
            return offset.addReturn(new Pose());
        }
    }
}
