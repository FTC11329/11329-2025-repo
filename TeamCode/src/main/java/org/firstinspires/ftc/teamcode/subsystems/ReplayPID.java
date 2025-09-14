package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.autos.AutoReplayAllenTest;
import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;

public class ReplayPID {
    AutoReplayAllenTest autoReplay;


    double integralHistoryX, integralHistoryY, integralHistoryTheta, previousTime;

    public ReplayPID(AutoReplayAllenTest autoReplay){
        this.autoReplay = autoReplay;
    }

    public double[] replayPIDMotorValues(double currentTime, Pose currentPose){
        double[] targetList;
        targetList = autoReplay.getTargets(currentTime); //xTarget 0, yTarget 1, headingTarget 2, vxTarget 3, vyTarget 4, omegaTargeta 5, xTarget 6, ayTarget 7, alphaTarget 8\

        // first for the feed forward system that will predict motor powers to reach target acceleration and velocity
        double kVxy = .015; //random needs tuning
        double kAxy = 0; //acceleration
        double kVtheta = .01; //random needs tuning
        double kAtheta = 0; //acceleration
        double ffX = kVxy * targetList[3] + kAxy * targetList[6];
        double ffY = kVxy * targetList[4] + kAxy * targetList[7];
        double ffTheta = kVtheta * targetList[5] + kAtheta * targetList[8];

        double[] errorList = autoReplay.getError(currentTime, currentPose);
        double dT = currentTime - previousTime;
        double kP = .14, kI = 0, kD = 0.0, integralGain = 0.5;
        double kPtheta = 1.1, kItheta = 0, kDtheta = 0.0, integralGaintheta = 0.5;
        double pidX = kP * errorList[0] + kI * (integralHistoryX + integralGain * errorList[0] * dT) + kD * errorList[0];
        double pidY = kP * errorList[1] + kI * (integralHistoryY + integralGain * errorList[1] * dT) + kD * errorList[1];
        double pidTheta = kPtheta * errorList[2] + kItheta * (integralHistoryTheta + integralGaintheta * errorList[2] * dT) + kDtheta * errorList[2];

        double controlX = ffX +pidX;
        double controlY = ffY +pidY;
        double controlTheta = ffTheta +pidTheta;

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
