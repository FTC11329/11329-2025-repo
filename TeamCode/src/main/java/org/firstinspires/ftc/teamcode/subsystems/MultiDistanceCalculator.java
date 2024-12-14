package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
                distanceArray = new double[2][7][5];

                int c = 0;
                while (c <= 5) {
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
                double[][] finalResult = findSmallestNonZeroInColumns(distanceArray);

                //find the closest non-zero distance block
                double[] finalValue = findSmallestNonZeroInFinalResult(finalResult);

                return finalValue;

            }
        } 
        return null;
    }

    public static double[][] findSmallestNonZeroInColumns(double[][][] distanceArray) {
        int rows = distanceArray[1].length; // Number of rows in distanceArray[1]
        int columns = distanceArray[1][0].length; // Number of columns in distanceArray[1]

        // Initialize the result array
        double[][] smallestValues = new double[columns][2]; // [][0] for distanceArray[1], [][1] for distanceArray[0]

        // Iterate through each column
        for (int c = 0; c < columns; c++) {
            double smallestNonZero = Double.MAX_VALUE; // Start with a very large value
            double correspondingValue = 0; // To store the value from distanceArray[0][x][c]

            // Iterate through each row in the column
            for (int x = 0; x < rows; x++) {
                double value = distanceArray[1][x][c];

                // Check if the value is non-zero and smaller than the current smallest
                if (value > 0 && value < smallestNonZero) {
                    smallestNonZero = value;
                    correspondingValue = distanceArray[0][x][c];
                }
            }

            // If no non-zero value was found, set to 0
            smallestValues[c][1] = (smallestNonZero == Double.MAX_VALUE) ? 0 : smallestNonZero;
            smallestValues[c][0] = (smallestNonZero == Double.MAX_VALUE) ? 0 : correspondingValue;
        }

        return smallestValues;
    }

    public static double[] findSmallestNonZeroInFinalResult(double[][] result) {
        //to sort the result of the first sort lol
        int columns = result.length;
        double[] smallestInResult = new double[columns];

        for (int c = 0; c < columns; c++) {
            smallestInResult[c] = result[c][0] > 0 ? result[c][0] : Double.MAX_VALUE;
        }

        return smallestInResult;
    }


    public void stopLimelight() {
        limelight.stop();
    }
}