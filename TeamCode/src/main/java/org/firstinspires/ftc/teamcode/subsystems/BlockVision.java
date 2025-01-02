package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

import java.util.List;

public class BlockVision {
    //right of robot +X
    //left of robot -X
    //In front of robot +Y

    private final double cameraXOffset = Math.PI; //close enough
    private final double cameraYOffset = 1.1;

    public BlockVision(HardwareMap hardwareMap, RobotSideEnum robotSide) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
    }

    public Limelight3A limelight;

    //Thanks Adrian
    public Pose getBestBlockPos() {
        LLResult result = getResult();
        Pose finalVector = null;

        if (result != null) {

            if (result.isValid()) {

                double runningMinX = 40000;
                double runningMinY = 40000;

                //define constants
                double height = 11.4;

                int c = 0;

                while (c <= 0) {
                    double champX = 10000.0;
                    double champY = 10000.0;

                    List<LLResultTypes.ColorResult> colorResults = result.getColorResults();

                    for (LLResultTypes.ColorResult cr : colorResults) {
                        //calculate distance from block

                        // calculate the angle from camera to block
                        double trialAngleY = cr.getTargetYDegrees();
                        double y_angle_radians = Math.toRadians(trialAngleY + 60.0);

                        // calculate the distance from the robot
                        double trialY = height * Math.tan(y_angle_radians);

                        //get the angle distance from the center of the screen
                        double trialAngleX = cr.getTargetXDegrees();
                        double x_angle_radians = Math.toRadians(trialAngleX);

                        //calculate the difference in x
                        double trialX = trialY * Math.tan(x_angle_radians);
                        if (trialY <= 28) {
                            if (Math.abs(trialX) < Math.abs(champX)) {
                                champX = trialX;
                                champY = trialY;
                            }
                        }


                    }
                    if (Math.abs(champX) < Math.abs(runningMinX)) {
                        runningMinX = champX;
                        runningMinY = champY;
                    }
                    c++;
                }

                if ((Math.abs(runningMinY) + Math.abs(runningMinX)) <= 100){
                    // returns relative position of the block to front of the robot
                    finalVector = new Pose(runningMinX - cameraXOffset, runningMinY - cameraYOffset, 0);
                }
                return finalVector;

            }
        } else {
            return null;
        }
        return null;
    }

    public LLResult getResult() {
        return limelight.getLatestResult();
    }
}
