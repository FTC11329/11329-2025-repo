package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizers;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Point;
import com.pedropathing.pathgen.Vector;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.pedropathing.util.KalmanFilterParameters;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
public class FConstants { // This is how we change Follower Constants.
    static {
        FollowerConstants.localizers = Localizers.OTOS;
        FollowerConstants.leftFrontMotorName = "leftFront";
        FollowerConstants.leftRearMotorName = "leftBack";
        FollowerConstants.rightFrontMotorName = "rightFront";
        FollowerConstants.rightRearMotorName = "rightBack";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.motorCachingThreshold = 0.01;

        FollowerConstants.xMovement = 73.2;
        FollowerConstants.yMovement = 53.863;
        double[] convertToPolar = Point.cartesianToPolar(FollowerConstants.xMovement, -FollowerConstants.yMovement);
        FollowerConstants.frontLeftVector = MathFunctions.normalizeVector(new Vector(convertToPolar[0], convertToPolar[1]));


        // Translational PIDF coefficients (don't use integral)
//        FollowerConstants.translationalPIDFCoefficients = new CustomPIDFCoefficients(
//                0.14,
//                0,
//                0.01,
//                0);
        FollowerConstants.translationalPIDFCoefficients = new CustomPIDFCoefficients(
                0,
                0,
                0,
                0);

        // Translational Integral
        FollowerConstants.translationalIntegral = new CustomPIDFCoefficients(
                0,
                0.0002,
                0,
                0);

        // Feed forward constant added on to the translational PIDF
        FollowerConstants.translationalPIDFFeedForward = 0.015;


        // Heading error PIDF coefficients
        FollowerConstants.headingPIDFCoefficients = new CustomPIDFCoefficients(
                1.1,
                0,
                0.09,
                0);

        // Feed forward constant added on to the heading PIDF
        FollowerConstants.headingPIDFFeedForward = 0.01;


        // Drive PIDF coefficients
        FollowerConstants.drivePIDFCoefficients = new CustomFilteredPIDFCoefficients(
                0.006,
                0.001,
                0.000095,
                0.6,
                0);

        // Feed forward constant added on to the drive PIDF
        FollowerConstants.drivePIDFFeedForward = 0.01;

        // Kalman filter parameters for the drive error Kalman filter
        FollowerConstants.driveKalmanFilterParameters = new KalmanFilterParameters(
                6,
                1);


        // Mass of robot in kilograms
        FollowerConstants.mass = 14;

        // Centripetal force to power scaling
        FollowerConstants.centripetalScaling = 0.0013;


        // Acceleration of the drivetrain when power is cut in inches/second^2 (should be negative)
        // if not negative, then the robot thinks that its going to go faster under 0 power
        FollowerConstants.forwardZeroPowerAcceleration = -28.786;

        // Acceleration of the drivetrain when power is cut in inches/second^2 (should be negative)
        // if not negative, then the robot thinks that its going to go faster under 0 power
        FollowerConstants.lateralZeroPowerAcceleration = -73.6813;

        // A multiplier for the zero power acceleration to change the speed the robot decelerates at
        // the end of paths.
        // Increasing this will cause the robot to try to decelerate faster, at the risk of overshoots
        // or localization slippage.
        // Decreasing this will cause the deceleration at the end of the Path to be slower, making the
        // robot slower but reducing risk of end-of-path overshoots or localization slippage.
        // This can be set individually for each Path, but this is the default.
        FollowerConstants.zeroPowerAccelerationMultiplier = 1;


        // When the robot is at the end of its current Path or PathChain and the velocity goes below
        // this value, then end the Path. This is in inches/second.
        // This can be custom set for each Path.
        FollowerConstants.pathEndVelocityConstraint = 0.1;

        // When the robot is at the end of its current Path or PathChain and the translational error
        // goes below this value, then end the Path. This is in inches.
        // This can be custom set for each Path.
        FollowerConstants.pathEndTranslationalConstraint = 0.1;

        // When the robot is at the end of its current Path or PathChain and the heading error goes
        // below this value, then end the Path. This is in radians.
        // This can be custom set for each Path.
        FollowerConstants.pathEndHeadingConstraint = 0.007;

        // When the t-value of the closest point to the robot on the Path is greater than this value,
        // then the Path is considered at its end.
        // This can be custom set for each Path.
        FollowerConstants.pathEndTValueConstraint = 0.995;

        // When the Path is considered at its end parametrically, then the Follower has this many
        // milliseconds to further correct by default.
        // This can be custom set for each Path.
        FollowerConstants.pathEndTimeoutConstraint = 500;

        // This is how many steps the BezierCurve class uses to approximate the length of a BezierCurve.
        FollowerConstants.APPROXIMATION_STEPS = 1000;

        // This is scales the translational error correction power when the Follower is holding a Point.
        FollowerConstants.holdPointTranslationalScaling = 0.45;

        // This is scales the heading error correction power when the Follower is holding a Point.
        FollowerConstants.holdPointHeadingScaling = 0.35;

        // This is the number of times the velocity is recorded for averaging when approximating a first
        // and second derivative for on the fly centripetal correction. The velocity is calculated using
        // half of this number of samples, and the acceleration uses all of this number of samples.
        FollowerConstants.AVERAGED_VELOCITY_SAMPLE_NUMBER = 8;

        // This is the number of steps the binary search for closest point uses. More steps is more
        // accuracy, and this increases at an exponential rate. However, more steps also does take more
        // time.
        FollowerConstants.BEZIER_CURVE_SEARCH_LIMIT = 10;


        // These activate / deactivate the secondary PIDs. These take over at errors under a set limit for
        // the translational, heading, and drive PIDs.
        FollowerConstants.useSecondaryTranslationalPID = true;
        FollowerConstants.useSecondaryHeadingPID = true;
        FollowerConstants.useSecondaryDrivePID = false;


        // the limit at which the translational PIDF switches between the main and secondary translational PIDFs,
        // if the secondary PID is active
        FollowerConstants.translationalPIDFSwitch = 3;

        // Secondary translational PIDF coefficients (don't use integral)
        FollowerConstants.secondaryTranslationalPIDFCoefficients = new CustomPIDFCoefficients(
                0.3,
                0,
                0.01,
                0);

        // Secondary translational Integral value
        FollowerConstants.secondaryTranslationalIntegral = new CustomPIDFCoefficients(
                0,
                0,
                0,
                0);

        // Feed forward constant added on to the small translational PIDF
        FollowerConstants.secondaryTranslationalPIDFFeedForward = 0.015;


        // the limit at which the heading PIDF switches between the main and secondary heading PIDFs
        FollowerConstants.headingPIDFSwitch = Math.PI / 20;

        // Secondary heading error PIDF coefficients
        FollowerConstants.secondaryHeadingPIDFCoefficients = new CustomPIDFCoefficients(
                5,
                0,
                0.08,
                0);

        // Feed forward constant added on to the secondary heading PIDF
        FollowerConstants.secondaryHeadingPIDFFeedForward = 0.01;


        // the limit at which the heading PIDF switches between the main and secondary drive PIDFs
        FollowerConstants.drivePIDFSwitch = 20;

        // Secondary drive PIDF coefficients
        FollowerConstants.secondaryDrivePIDFCoefficients = new CustomFilteredPIDFCoefficients(
                0.02,
                0,
                0.000005,
                0.6,
                0);

        // Feed forward constant added on to the secondary drive PIDF
        FollowerConstants.secondaryDrivePIDFFeedForward = 0.01;

        FollowerConstants.useBrakeModeInTeleOp = false;
        FollowerConstants.automaticHoldEnd = true;
        FollowerConstants.useVoltageCompensationInAuto = false;
        FollowerConstants.useVoltageCompensationInTeleOp = false;
        FollowerConstants.nominalVoltage = 12.0;
        FollowerConstants.cacheInvalidateSeconds = 0.5;
    }
}
