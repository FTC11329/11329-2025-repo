package org.firstinspires.ftc.teamcode.pedropathing.localization.localizers;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedropathing.localization.Localizer;
import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.MathFunctions;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Vector;

import static org.firstinspires.ftc.teamcode.pedropathing.localization.constants.OTOSConstants.*;

import java.util.ArrayDeque;
import java.util.Deque;

public class OTOSLocalizer extends Localizer {
    private HardwareMap hardwareMap;
    private SparkFunOTOS otos;
    private SparkFunOTOS.Pose2D otosPose;
    private SparkFunOTOS.Pose2D otosVel;
    private SparkFunOTOS.Pose2D otosAcc;
    private double previousHeading;
    private double totalHeading;

    // Rolling history
    private static final int SAMPLE_COUNT = 2;
    private final Deque<Pose> poseHistory = new ArrayDeque<>();
    private final Deque<Long> timeHistory = new ArrayDeque<>();
    private final Deque<Pose> velocityHistory = new ArrayDeque<>();

    private Pose calculatedVelocity = new Pose();
    private Pose calculatedAcceleration = new Pose();

    public OTOSLocalizer(HardwareMap map) {
        this(map, new Pose());
    }

    public OTOSLocalizer(HardwareMap map, Pose setStartPose) {
        hardwareMap = map;

        if (useCorrectedOTOSClass && false) {
            // otos = hardwareMap.get(SparkFunOTOSCorrected.class, hardwareMapName);
        } else {
            otos = hardwareMap.get(SparkFunOTOS.class, hardwareMapName);
        }

        otos.setLinearUnit(linearUnit);
        otos.setAngularUnit(angleUnit);
        otos.setOffset(offset);
        otos.setLinearScalar(1);
        otos.setAngularScalar(angularScalar);

        otos.calibrateImu();
        otos.resetTracking();

        setStartPose(setStartPose);

        otosPose = new SparkFunOTOS.Pose2D();
        otosVel = new SparkFunOTOS.Pose2D();
        otosAcc = new SparkFunOTOS.Pose2D();

        totalHeading = 0;
        previousHeading = otos.getPosition().h;

        resetOTOS();
    }

    @Override
    public Pose getPose() {
        Pose pose = new Pose(otosPose.x * linearScalar, otosPose.y * linearScalar, otosPose.h);
        Vector vec = pose.getVector();
        return new Pose(vec.getXComponent(), vec.getYComponent(), pose.getHeading());
    }

    @Override
    public Pose getVelocity() {
        return calculatedVelocity;
    }

    @Override
    public Vector getVelocityVector() {
        return calculatedVelocity.getVector();
    }

    public Pose getAcceleration() {
        return calculatedAcceleration;
    }

    public Vector getAccelerationVector() {
        return calculatedAcceleration.getVector();
    }

    @Override
    public void setStartPose(Pose setStart) {
        otos.setPosition(new SparkFunOTOS.Pose2D(
                setStart.getX() * (1 / linearScalar),
                setStart.getY() * (1 / linearScalar),
                setStart.getHeading()
        ));
    }

    @Override
    public void setPose(Pose setPose) {
        resetOTOS();
        otos.setPosition(new SparkFunOTOS.Pose2D(setPose.getX(), setPose.getY(), setPose.getHeading()));
    }

    @Override
    public void update() {
        otos.getPosVelAcc(otosPose, otosVel, otosAcc);

        // Update heading tracking
        totalHeading += MathFunctions.getSmallestAngleDifference(otosPose.h, previousHeading);
        previousHeading = otosPose.h;

        // Current position and time
        Pose currentPose = new Pose(otosPose.x * linearScalar, otosPose.y * linearScalar, otosPose.h);
        long currentTime = System.nanoTime();

        // Update history
        poseHistory.addLast(currentPose);
        timeHistory.addLast(currentTime);

        if (poseHistory.size() > SAMPLE_COUNT) {
            poseHistory.removeFirst();
            timeHistory.removeFirst();
        }

        // Calculate velocity if enough samples
        if (poseHistory.size() >= 2) {
            Pose firstPose = poseHistory.getFirst();
            Pose lastPose = poseHistory.getLast();

            long firstTime = timeHistory.getFirst();
            long lastTime = timeHistory.getLast();
            double deltaTime = (lastTime - firstTime) / 1e9;

            if (deltaTime > 0) {
                double dx = lastPose.getX() - firstPose.getX();
                double dy = lastPose.getY() - firstPose.getY();
                double dh = MathFunctions.getSmallestAngleDifference(lastPose.getHeading(), firstPose.getHeading());

                calculatedVelocity = new Pose(dx / deltaTime, dy / deltaTime, dh / deltaTime);

                // Update velocity history
                velocityHistory.addLast(calculatedVelocity);
                if (velocityHistory.size() > SAMPLE_COUNT) {
                    velocityHistory.removeFirst();
                }

                // Calculate acceleration
                if (velocityHistory.size() >= 2) {
                    Pose firstVel = velocityHistory.getFirst();
                    Pose lastVel = velocityHistory.getLast();

                    double ddx = lastVel.getX() - firstVel.getX();
                    double ddy = lastVel.getY() - firstVel.getY();
                    double ddh = lastVel.getHeading() - firstVel.getHeading();

                    calculatedAcceleration = new Pose(ddx / deltaTime, ddy / deltaTime, ddh / deltaTime);
                }
            }
        }
    }

    public void resetOTOS() {
        otos.resetTracking();
        poseHistory.clear();
        timeHistory.clear();
        velocityHistory.clear();
    }

    public double getTotalHeading() {
        return totalHeading;
    }

    public double getForwardMultiplier() {
        return otos.getLinearScalar();
    }

    public double getLateralMultiplier() {
        return otos.getLinearScalar();
    }

    public double getTurningMultiplier() {
        return otos.getAngularScalar();
    }

    public void resetIMU() {
    }

    public boolean isNAN() {
        return Double.isNaN(getPose().getX()) ||
                Double.isNaN(getPose().getY()) ||
                Double.isNaN(getPose().getHeading());
    }

    public SparkFunOTOS.Status getStatus() {
        return otos.getStatus();
    }
}
