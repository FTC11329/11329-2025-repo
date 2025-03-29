package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptAprilTag;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

import java.lang.reflect.Array;
import java.lang.reflect.Type;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.ArrayList;
import java.util.Set;

public class Attempt89 {

    RobotSideEnum robotSideEnum;

    private Limelight3A limelight;

    private final double cameraYOffset = 1.1;
    private final double cameraXOffset = Math.PI - 1;

    //define the height that the center of the camera lens is off the ground
    double height = 10;
    //define the angle that the camera is pointing (90 deg = directly forward)
    double cameraAngle = Math.toRadians(62.6);
    // define the range of blocks that the robot can grab inches
    double yMaxExtension = 28.0;
    double xMaxTurn = 18.0;

    //find the corresponding angles that the camera would need to get to go outside the acceptable range
    double maxYangle = (Math.atan(yMaxExtension / height) - cameraAngle);

    //the xMaxRotation cannot be calculated because we do not have the distance of the block

    public Attempt89(HardwareMap hardwareMap, RobotSideEnum robotSideEnum) {
        this.robotSideEnum = robotSideEnum;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        // 0 is yellow; 1 is blue; 2 is red
        if (robotSideEnum == RobotSideEnum.Red) {
            switchPipeline(2);
        } else {
            switchPipeline(1);
        }
        limelight.start();
    }

    public Pose2D getBlockPosition() {

        List<Pose2D> distanceArray = fillImageArray();
        //Returns an empty pose2d if nothing is there
        if (distanceArray == null || distanceArray.isEmpty()){
            return new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, -1);
        }

        //Returns an empty pose2d if nothing is there
        Pose2D finalResult = getClosestBlock(distanceArray);
        if (finalResult == null){
            return new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, -1);
        }

        //Returns an empty pose2d if 0,0,0
        if (finalResult.getX(DistanceUnit.INCH) == 0 && finalResult.getY(DistanceUnit.INCH) == 0) {
            return new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, -1);
        }

        return finalResult;
    }

    public List<Pose2D> fillImageArray() {
        //Creating a 3d array to store the distances of each block for comparison
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                boolean isWorking = true;
                List<Pose2D> frameDistances = new ArrayList<>();
                int i = 0;
                for (LLResultTypes.ColorResult cr : colorResults) {
                    //put the angles that are being used in a better variable
                    double cameraAngleY = (Math.toRadians(cr.getTargetYDegrees()) + cameraAngle);
                    double trialAngleX = Math.toRadians(cr.getTargetXDegrees());

                    //calculate the real distances X&Y away from the camera
                    double distanceY = (height * (Math.tan(cameraAngleY)));
                    double distanceX = (distanceY * (Math.tan(trialAngleX)));
                    //Added distanceY - 1 so we will hit the block with the front rollers
                    Pose2D distance = new Pose2D(DistanceUnit.INCH, (1.25 * distanceX) - cameraXOffset, distanceY - cameraYOffset, AngleUnit.DEGREES, 0);
                    frameDistances.add(distance);
                    i++;
                }
                return frameDistances;
            }
        }
        return null;
    }

    public Pose2D getClosestBlock(List<Pose2D> distanceArray) {
        int blocks = distanceArray.size();

        // Array to store the smallest coordinates for each plane
        Pose2D smallestCoordinates = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        double minTime = Double.MAX_VALUE;

        // Iterate through each block
        for (int block = 0; block < blocks; block++) {
            Pose2D pos = distanceArray.get(block);
            //if in range
            if (pos.getY(DistanceUnit.INCH) < (Constants.Intake.maxSlidePos + 100) * (1/Constants.Intake.inchToTick)) {
                double time = getReachTime(pos);
                if (time < minTime) {
                    minTime = time;
                    smallestCoordinates = pos;
                }
            }
        }

        return smallestCoordinates;
    }

    public double getReachTime(Pose2D coords){
        return Math.hypot(coords.getX(DistanceUnit.INCH), coords.getY(DistanceUnit.INCH));
    }

    public List<Pose> findAverageOfFullFrames(List<List<Pose>> distanceArray) {
        // this class is here to deal with cases where the camera misses a block
        // this will cut out all frames that have too few blocks and average the remaining

        int maxBlockNumber = 0;
        int[] blockNumbers = new int[distanceArray.size()];
        // counts how many blocks are in each frame
        for (int frame = 0; frame < distanceArray.size(); frame++) {
            int number = distanceArray.get(frame).size();
            blockNumbers[frame] = number;
            maxBlockNumber = Math.max(maxBlockNumber, number);
        }

        List<Pose> newDistanceArray = new ArrayList<>();
        // average all of the blocks in the frames that share the greatest number of blocks
        if (maxBlockNumber > 0) {
            for (int block = 0; block < maxBlockNumber; block++) {
                Pose pos = new Pose(0.0, 0.0, 0.0);
                int q = 0;
                for (int frame = 0; frame < distanceArray.size(); frame++) {
                    if (blockNumbers[frame] == maxBlockNumber) {
                        pos.add(distanceArray.get(frame).get(block));
                        q += 1;
                    }
                }
                pos.scalarDivide(q);
                newDistanceArray.add(pos);
            }
        }
        return newDistanceArray;
    }


    public List<Pose> averageBlocks(List<List<Pose>> poses) {
        // Initialize the cluster centers with the first frame's detections.
        Pose[] clusterCenters = poses.get(0).toArray(new Pose[0]);
        Pose[] lastIteration = new Pose[clusterCenters.length];

        while (!Arrays.equals(lastIteration, clusterCenters)) {
            lastIteration = clusterCenters.clone();

            // Keep track of which blocks have been used from each frame
            List<Set<Pose>> usedBlocks = new ArrayList<>();
            for (List<Pose> frame : poses) {
                usedBlocks.add(new HashSet<>());
            }

            for (int i = 0; i < clusterCenters.length; i++) {
                Pose mainPose = clusterCenters[i];
                List<Pose> mainDistances = new ArrayList<>();

                // Iterate over frames and select the closest unused block
                for (int frameIdx = 0; frameIdx < poses.size(); frameIdx++) {
                    List<Pose> frames = poses.get(frameIdx);
                    Set<Pose> used = usedBlocks.get(frameIdx);

                    double closestDistance = Double.MAX_VALUE;
                    Pose closestBlock = null;

                    for (Pose block : frames) {
                        if (!used.contains(block)) {  // Ensure block hasn't been used
                            double dst = Math.hypot(block.getX() - mainPose.getX(), block.getY() - mainPose.getY());
                            if (dst < closestDistance) {
                                closestDistance = dst;
                                closestBlock = block;
                            }
                        }
                    }

                    if (closestBlock != null) {
                        mainDistances.add(closestBlock);
                        used.add(closestBlock); // Mark block as used
                    }
                }

                // Compute the average of all selected blocks
                Pose av = new Pose(0.0, 0.0, 0.0);
                for (Pose pos : mainDistances) {
                    av.add(pos);
                }
                av.scalarDivide(mainDistances.size());
                clusterCenters[i] = av;
            }
        }

        return new ArrayList<>(Arrays.asList(clusterCenters));
    }

    public void stopLimelight() {
        limelight.stop();
    }

    public void switchPipeline(int pipelineNum){
        limelight.pipelineSwitch(pipelineNum);
    }

    public void switchPipeline(RobotSideEnum robotSide){
        if (robotSide == RobotSideEnum.Blue){
            limelight.pipelineSwitch(1);
        } else {
            limelight.pipelineSwitch(2);
        }
    }

    public Pose2D getBestBlock(int pipelineNum) {
        // 0 is yellow, 1 is blue, 2 is red
        switchPipeline(pipelineNum);
        return getBlockPosition();
    }

    public Pose2D getBestSample() {
        return getBestSample(robotSideEnum);
    }

    public Pose2D getBestSample(RobotSideEnum robotSide) {
        Pose2D colorPose;
        if (robotSide == RobotSideEnum.Red) {
            colorPose = getBestBlock(2);
        } else {
            colorPose = getBestBlock(1);
        }
        Pose2D yellowPose = getBestBlock(0);

        double colorTime = getReachTime(colorPose);
        double yellowTime = getReachTime(yellowPose);

        if (colorTime < yellowTime) {
            return colorPose;
        } else {
            return yellowPose;
        }
    }

    public Pose2D getBestSpecimen() {
        return getBestSpecimen(robotSideEnum);
    }

    public Pose2D getBestSpecimen(RobotSideEnum robotSide) {
        if (robotSide == RobotSideEnum.Red) {
            return getBestBlock(2);
        } else {
            return getBestBlock(1);
        }
    }
}
