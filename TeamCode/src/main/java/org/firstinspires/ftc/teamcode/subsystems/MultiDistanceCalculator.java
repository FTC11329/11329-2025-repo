package org.firstinspires.ftc.teamcode.subsystems;

import java.util.Arrays;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

import java.util.List;

public class MultiDistanceCalculator {

    private Limelight3A limelight;

    private final double cameraYOffset = 1.1;
    private final double cameraXOffset = Math.PI; //close enough

    public MultiDistanceCalculator(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        // 0 is yellow; 1 is blue; 2 is red
        limelight.pipelineSwitch(1);
        limelight.start();
    }

    public Pose getBlockPosition() {

        LLResult result = limelight.getLatestResult();

        if (result != null) {
            if (result.isValid()) {

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

                // Call the method to find the smallest non-zero values
                double[][] finalResult = findSmallestNonZeroInEachPlane(distanceArray);

                //find the closest non-zero distance block
                double[] finalValue = findSmallestNonZeroInFinalResult(finalResult);

                return new Pose((finalValue[0] - cameraXOffset), (finalValue[1] - cameraYOffset),0.0);
            }
        }
        return null;
    }
    public static double[][] findSmallestNonZeroInEachPlane(double[][][] distanceArray) {
        int planes = distanceArray.length;
        int rows = distanceArray[0].length;
        int columns = distanceArray[0][0].length;

        // Array to store the smallest coordinates for each plane
        double[][] smallestCoordinates = new double[planes][columns];

        // Iterate through each plane
        for (int p = 0; p < planes; p++) {
            for (int c = 0; c < columns; c++) {
                double smallestValue = Double.MAX_VALUE;

                // Iterate through each row in the plane
                for (int r = 0; r < rows; r++) {
                    double value = distanceArray[p][r][c];
                    if (value > 0 && value < smallestValue) {
                        smallestValue = value;
                    }
                }

                // Store the smallest value and its coordinates for this plane
                smallestCoordinates[p][c] = smallestValue;
            }
        }

        return smallestCoordinates;
    }

    public static double[] findSmallestNonZeroInFinalResult(double[][] result) {
        // Sort the array by smallest values
        Arrays.sort(result, (a, b) -> Double.compare(a[0], b[0]));

        /**
         * based on the length of the array that is passed through it needs to do different things to maximize results
         */
        switch (result.length) {
            case (5):
                // Remove the first and last values
                double[][] trimmedResult = Arrays.copyOfRange(result, 1, result.length - 1);

                // Check if the coordinates are within 3 units and return the result
                double xDiff = Math.abs(trimmedResult[0][0] - trimmedResult[0][2]);
                double yDiff = Math.abs(trimmedResult[1][0] - trimmedResult[1][2]);
                if (xDiff <= 3 && yDiff <= 3) {
                    double avgX = (trimmedResult[0][0] + trimmedResult[1][0]) / 2;
                    double avgY = (trimmedResult[0][1] + trimmedResult[1][1]) / 2;
                    return new double[]{avgX, avgY};
                } else {
                    return trimmedResult[1]; // Return the middle value
                }
            case (4):
                // Remove the first and last values
                double[][] trimmedResults = Arrays.copyOfRange(result, 1, result.length - 1);

                // Check if the coordinates are within 3 units and return the result
                double xDiffz = Math.abs(trimmedResults[0][0] - trimmedResults[0][1]);
                double yDiffz = Math.abs(trimmedResults[1][0] - trimmedResults[1][1]);

                if (xDiffz <= 3 && yDiffz <= 3) {
                    double avgX = (trimmedResults[0][0] + trimmedResults[1][0]) / 2;
                    double avgY = (trimmedResults[0][1] + trimmedResults[1][1]) / 2;
                    return new double[]{avgX, avgY};
                } else {
                    return trimmedResults[1]; // Return the middle value
                }
            case (3):
                // Check if the coordinates are within 3 units and return the result
                double xDifft = Math.abs(result[0][0] - result[0][2]);
                double yDifft = Math.abs(result[1][0] - result[1][2]);

                if (xDifft <= 3 && yDifft <= 3) {
                    double avgX = (result[0][0] + result[1][0]) / 2;
                    double avgY = (result[0][1] + result[1][1]) / 2;
                    return new double[]{avgX, avgY};
                } else {
                    return result[1]; // Return the middle value
                }
            case (2):
                // Check if the coordinates are within 3 units and return the result
                double xDiffs = Math.abs(result[0][0] - result[0][1]);
                double yDiffs = Math.abs(result[1][0] - result[1][1]);
                if (xDiffs <= 3 && yDiffs <= 3) {
                    double avgX = (result[0][0] + result[1][0]) / 2;
                    double avgY = (result[0][1] + result[1][1]) / 2;
                    return new double[]{avgX, avgY};
                } else {
                    return result[0]; // Return the first value
                }
            case (1):
                return result[0];
            case (0):
                return null;
        }
        return null;
    }

    public void stopLimelight() {
        limelight.stop();
    }
}
