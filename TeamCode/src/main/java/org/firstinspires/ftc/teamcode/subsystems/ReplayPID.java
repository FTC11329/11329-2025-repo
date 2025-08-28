package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.autos.AutoReplayAllenTest;
import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;

public class ReplayPID {
    AutoReplayAllenTest autoReplay;

    public double[] replayPIDMotorValues(double currentTime, Pose currentPose){
        double[] targetList = autoReplay.getTargets(currentTime); //xTarget 0, yTarget 1, headingTarget 2, vxTarget 3, vyTarget 4, omegaTargeta 5, xTarget 6, ayTarget 7, alphaTarget 8
        //first for the feed forward system that will predict motor powers to reach target acceleration and velocity
        double kV = 1.0; //random needs tuning
        double kA = .1; //acceleration
        double ffX = kV * targetList[3] + kA * targetList[6];
        double ffY = kV * targetList[4] + kA * targetList[7];
        double ffTheta = kV * targetList[5] + kA * targetList[8];

        double[] errorList = autoReplay.getError(currentTime, currentPose);
        double kP = 1.0, kI = 0.0, kD = 0.0;
        double pidX = kP * errorList[0] + kI * errorList[0] + kD * errorList[0];
        double pidY = kP * errorList[1] + kI * errorList[1] + kD * errorList[1];
        double pidTheta = kP * errorList[2] + kI * errorList[2] + kD * errorList[2];

        double controlX = ffX +pidX;
        double controlY = ffY +pidY;
        double controlTheta = ffX +pidTheta;

        double cosH = Math.cos(currentPose.getHeading());
        double sinH = Math.sin(currentPose.getHeading());

        double forward = controlX * sinH + controlY * cosH; // robot-forward
        double strafe  = controlX * cosH - controlY * sinH; // robot-left

        double fl = forward + strafe + controlTheta;
        double fr = forward - strafe - controlTheta;
        double bl = forward - strafe + controlTheta;
        double br = forward + strafe - controlTheta;

        //normalize to one
        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

        fl /= max;
        fr /= max;
        bl /= max;
        br /= max;

        return new double[]{fl, fr, bl, br};
    }
}
