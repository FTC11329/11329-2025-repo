package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

import java.util.List;

public class BlockVision {
    public Limelight3A limelight; // Limelight camera instance for vision processing
    //right of robot +X
    //left of robot -X
    //In front of robot +Y

    private final double cameraXOffset = Math.PI; //close enough
    private final double cameraYOffset = 1.1;

    // Constructor to initialize the Limelight camera and set the pipeline
    public BlockVision(HardwareMap hardwareMap, RobotSideEnum robotSide) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1); // Switch to pipeline 1
    }

    /**
     * Calculates the best block position (x, y) using vision data from Limelight.
     * @return A Vector2d object representing the x and y coordinates of the closest block.
     */
    public Pose getBestBlockPos() {
        LLResult result = getResult(); // Fetch the latest Limelight result
        Pose finalVector = null;

        if (result != null) { // Ensure result is not null
            if (result.isValid()) { // Check if the result is valid
                double runningMinX = Double.MAX_VALUE; // Initialize running minimum x-distance
                double runningMinY = Double.MAX_VALUE; // Initialize running minimum y-distance
                double height = 11.4; // Height of the camera in inches
                int c = 0; // Loop counter
                double cameraAngle = 60; //degrees from face down

                // Iterate to refine the closest block's position
                while (c <= 4) {
                    double champX = Double.MAX_VALUE; // Temporary best x-distance
                    double champY = Double.MAX_VALUE; // Temporary best y-distance

                    List<LLResultTypes.ColorResult> colorResults = result.getColorResults(); // Get color-based detection results

                    for (LLResultTypes.ColorResult cr : colorResults) {
                        // Calculate the angle from the camera to the block
                        double trialAngleY = cr.getTargetYDegrees();
                        double y_angle_radians = Math.toRadians(trialAngleY + cameraAngle); // Offset by camera angle

                        // Calculate the distance in the y-direction (depth)
                        double trialY = height * Math.tan(y_angle_radians);

                        // Calculate the angle in the x-direction
                        double trialAngleX = cr.getTargetXDegrees();
                        double x_angle_radians = Math.toRadians(trialAngleX);

                        // Calculate the distance in the x-direction
                        double trialX = trialY * Math.tan(x_angle_radians);

                        // Check if the block is within a valid range
                        if (trialY <= 28) {
                            if (Math.abs(trialX) < Math.abs(champX)) { // Find the closest block in x-direction
                                champX = trialX;
                                champY = trialY;
                            }
                        }
                    }

                    // Update the running minimum if a closer block is found
                    if (Math.abs(champX) < Math.abs(runningMinX)) {
                        runningMinX = champX;
                        runningMinY = champY;
                    }
                    c++; // Increment counter
                }

                // Validate and assign the final vector
                if ((Math.abs(runningMinY) + Math.abs(runningMinX)) <= 100) {
                    // returns relative position of the block to front of the robot
                    finalVector = new Pose((runningMinX - cameraXOffset), (runningMinY - cameraYOffset),0.0);
                }
                return finalVector;
            }
        }
        return null; // Return null as a fallback
    }

    /**
     * Fetches the latest result from the Limelight camera.
     * @return An LLResult object containing the latest vision processing data.
     */
    public LLResult getResult() {
        return limelight.getLatestResult();
    }
}
