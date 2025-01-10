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

    private Telemetry telemetry;
    private Limelight3A limelight;

    public MultiDistanceCalculator(HardwareMap hardwareMap, Telemetry telemetry) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        // 0 is yellow; 1 is blue; 2 is red
        limelight.pipelineSwitch(1);
        limelight.start();
        this.telemetry = telemetry;
    }

    public double[] getBlockPosition() {

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
                distanceArray = new double[2][5][5];

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
                                distanceArray[0][blockNum][c] = distanceX;
                                distanceArray[1][blockNum][c] = distanceY;
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

                return finalValue;

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
                double correspondingX = 0.0; // X-coordinate for smallest value
                double correspondingY = 0.0; // Y-coordinate for smallest value

                // Iterate through each row in the plane
                for (int r = 0; r < rows; r++) {
                    double value = distanceArray[p][r][c];

                    if (value > 0 && value < smallestValue) {
                        smallestValue = value;
                        correspondingX = distanceArray[p][r][0]; // Assuming X is in column 0
                        correspondingY = distanceArray[p][r][1]; // Assuming Y is in column 1
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
        System.out.println("Sorted result:");
        for (double[] row : result) {
            System.out.println(Arrays.toString(row));
        }

        // Remove the first and last values
        double[][] trimmedResult = Arrays.copyOfRange(result, 1, result.length - 1);

        // Check if the coordinates are within 3 units and return the result
        double xDiff = Math.abs(trimmedResult[0][0] - trimmedResult[2][0]);
        double yDiff = Math.abs(trimmedResult[0][1] - trimmedResult[2][1]);

        if (xDiff <= 3 && yDiff <= 3) {
            double avgX = (trimmedResult[0][0] + trimmedResult[1][0]) / 2;
            double avgY = (trimmedResult[0][1] + trimmedResult[1][1]) / 2;
            return new double[]{avgX, avgY};
        } else {
            return trimmedResult[1]; // Return the middle value
        }
    }

    public void stopLimelight() {
        limelight.stop();
    }
}
