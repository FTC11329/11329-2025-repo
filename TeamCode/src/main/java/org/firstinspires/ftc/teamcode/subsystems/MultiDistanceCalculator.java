package org.firstinspires.ftc.teamcode.subsystems;

import java.util.Arrays;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

import java.util.List;

public class MultiDistanceCalculator {

    private Limelight3A limelight;

    private final double cameraYOffset = 1.1;
    private final double cameraXOffset = Math.PI; //close enough



    //define the height that the center of the camera lens is off the ground
    double height = 10.75;
    //define the angle that the camera is pointing (90 deg = directly forward)
    double cameraAngle = Math.toRadians(60.0);
    // define the range of blocks that the robot can grab inches
    double yMaxExtension = 28.0;
    double xMaxTurn = 18.0;

    //find the corresponding angles that the camera would need to get to go outside the acceptable range
    double maxYangle = (Math.atan(yMaxExtension / height) - cameraAngle);
    //the xMaxRotation cannot be calculated because we do not have the distance of the block

    public MultiDistanceCalculator(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        // 0 is yellow; 1 is blue; 2 is red
        limelight.pipelineSwitch(1);
        limelight.start();
    }

    public Pose getBlockPosition(HardwareMap hardwareMap) {

        double[][][] distanceArray = fillImageArray();

        double[][] newDistanceArray = findAverageOfFullFrames(distanceArray);

        // Call the method to find the smallest non-zero values
        double[] finalValues = MinimizeTime(newDistanceArray);

        //find the closest non-zero distance block

        return new Pose(finalValues[0], finalValues[1], 0.0);
    }

    public double[][][] fillImageArray() {

        LLResult result = limelight.getLatestResult();

        if (result != null) {
            if (result.isValid()) {

                //Creating a 3d array to store the distances of each block for comparison
                double[][][] distanceArray;
                distanceArray = new double[2][7][5];

                int c = 0;
                while (c <= 4) {
                    int blockNum = 0;
                    List<LLResultTypes.ColorResult> colorResults = result.getColorResults();

                    for (LLResultTypes.ColorResult cr : colorResults) {
                        //put the angles that are being used in a better variable
                        double cameraAngleY = (Math.toRadians(cr.getTargetYDegrees()) + cameraAngle);
                        double trialAngleX = Math.toRadians(cr.getTargetXDegrees());

                        //Cut out all blocks that are out of the range of the intake
                        if (maxYangle >= cameraAngleY) {
                            //calculate the real distances X&Y away from the camera
                            double distanceY = (height * (Math.tan(cameraAngleY)));
                            double distanceX = (distanceY * (Math.tan(trialAngleX)));
                            if (distanceX <= xMaxTurn) {
                                //store the distance to the array
                                distanceArray[0][blockNum][c] = distanceX - cameraXOffset;
                                distanceArray[1][blockNum][c] = distanceY - cameraYOffset;
                            }
                        }
                        blockNum++;
                    }

                    c++;
                }
                return distanceArray;
            }
        }
        return null;
    }

    public double[][] findAverageOfFullFrames(double[][][] distanceArray) {
        // this class is here to deal with cases where the camera misses a block
        // this will cut out all frames that have too few blocks and average the remaining

        int maxBlockNumber = 0;
        double[] blockNumbers = new double[5];
        // counts how many blocks are in each frame
        for (int frame = 0; frame < 5; frame++) {
            int number = 0;
            for (int block = 0; block < 7; block++) {
                if (distanceArray[0][block][frame] != 0 || distanceArray[1][block][frame] != 0) {
                    number += 1;
                }
            }
            blockNumbers[frame] = number;
            maxBlockNumber = Math.max(maxBlockNumber, number);
        }

        int validFrames = 0;
        for (int frame = 0; frame < 5; frame++) {
            if (blockNumbers[frame] == maxBlockNumber) {
                validFrames += 1;
            }
        }

        double[][] newDistanceArray = new double[2][maxBlockNumber];
        // average all of the blocks in the frames that share the greatest number of blocks
        if (maxBlockNumber > 0) {
            int q = 0;
            for (int block = 0; block < maxBlockNumber; block++) {
                double[] pos = {0, 0};
                for (int frame = 0; frame < 5; frame++) {
                    if (blockNumbers[frame] == maxBlockNumber) {
                        pos[0] += distanceArray[0][block][frame] / validFrames;
                        pos[1] += distanceArray[1][block][frame] / validFrames;
                    }
                }
                if (pos[0] != 0 || pos[1] != 0) {
                    newDistanceArray[0][q] = pos[0];
                    newDistanceArray[1][q] = pos[1];
                    q++;
                }
            }
        }
        return newDistanceArray;
    }

    public double[] MinimizeTime(double[][] distanceArray) {
        int axis = distanceArray.length;
        int blocks = distanceArray[0].length;

        // Array to store the smallest coordinates for each plane
        double[] smallestCoordinates = new double[2];
        double minTime = Double.MAX_VALUE;

        // Iterate through each plane
        for (int c = 0; c < blocks; c++) {
            double[] value = {distanceArray[0][c], distanceArray[1][c]};
            double time = fastestSearchTime(value);
            if (time < minTime) {
                minTime = time;
                smallestCoordinates = value;
            }
        }

        return smallestCoordinates;
    }

    public double fastestSearchTime(double[] coords){
        return Math.hypot(coords[0], coords[1]);
    }

    public void stopLimelight() {
        limelight.stop();
    }

//    public Pose getBestColoredBlock(int pipelineNum) {
//        // 0 is yellow, 1 is blue, 2 is red
//        limelight.pipelineSwitch(pipelineNum);
//        return getBlockPosition();
//    }
//
//    public Pose getBestBlockGeneral(int pipelineNum) {
//        limelight.pipelineSwitch(pipelineNum);
//        Pose colorPose = getBlockPosition();
//        limelight.pipelineSwitch(0);
//        Pose yellowPose = getBlockPosition();
//
//        double[] colorCoords = {colorPose.getX(), colorPose.getY()};
//        double[] yellowCoords = {yellowPose.getX(), yellowPose.getY()};
//
//        double colorTime = fastestSearchTime(colorCoords);
//        double yellowTime = fastestSearchTime(yellowCoords);
//
//        if (colorTime < yellowTime) {
//            return colorPose;
//        } else {
//            return yellowPose;
//        }
//    }
}
