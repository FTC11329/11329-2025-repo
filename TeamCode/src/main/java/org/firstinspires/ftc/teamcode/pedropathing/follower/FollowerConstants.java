package org.firstinspires.ftc.teamcode.pedropathing.follower;


import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.pedropathing.localization.Localizers;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.MathFunctions;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Vector;
import org.firstinspires.ftc.teamcode.pedropathing.util.CustomFilteredPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedropathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedropathing.util.KalmanFilterParameters;

/**
 * This is the FollowerConstants class. It holds many constants and parameters for various parts of
 * the Follower. This is here to allow for easier tuning of Pedro Pathing, as well as concentrate
 * everything tunable for the Paths themselves in one place.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 3/4/2024
 */

@Config
public class FollowerConstants {

    /** The Localizer that the Follower & Pose Updater will use
     *  Default Value: Localizers.THREE_WHEEL */
    public static Localizers localizers = Localizers.OTOS;

    // This section is for configuring your motors
    public static String leftFrontMotorName = "leftFront";
    public static String leftRearMotorName = "leftBack";
    public static String rightFrontMotorName = "rightFront";
    public static String rightRearMotorName = "rightBack";

    /** The direction of the left front motor
     *  Default Value: DcMotorSimple.Direction.REVERSE */
    public static DcMotorSimple.Direction leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;

    /** The direction of the right front motor
     *  Default Value: DcMotorSimple.Direction.REVERSE */
    public static DcMotorSimple.Direction rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;

    /** The direction of the left rear motor
     *  Default Value: DcMotorSimple.Direction.FORWARD */
    public static DcMotorSimple.Direction leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;

    /** The direction of the right rear motor
     *  Default Value: DcMotorSimple.Direction.FORWARD */
    public static DcMotorSimple.Direction rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

    /** The motor caching threshold
     *  Default Value: 0.01 */
    public static double motorCachingThreshold = 0.01;

    public static double xMovement = 70;

    public static double yMovement = 48.8;



    private static double[] convertToPolar = Point.cartesianToPolar(xMovement, -yMovement);

    /** The actual drive vector for the front left wheel, if the robot is facing a heading of 0 radians with the wheel centered at (0,0)
     *  Default Value: new Vector(convertToPolar[0], convertToPolar[1])
     * @implNote This vector should not be changed, but only accessed.
     */
    public static Vector frontLeftVector = MathFunctions.normalizeVector(new Vector(convertToPolar[0], convertToPolar[1]));

    /** Global Max Power (can be overridden, just a default)
     *  Default Value: 1 */
    public static double maxPower = 1;

    public static CustomPIDFCoefficients translationalPIDFCoefficients = new CustomPIDFCoefficients(
            0.14,
            0,
            0.01,
            0);

    // Translational Integral
    public static CustomPIDFCoefficients translationalIntegral = new CustomPIDFCoefficients(
            0,
            0,
            0,
            0);

    // Feed forward constant added on to the translational PIDF
    public static double translationalPIDFFeedForward = 0.015;


    // Heading error PIDF coefficients
    public static CustomPIDFCoefficients headingPIDFCoefficients = new CustomPIDFCoefficients(
            1.1,
            0,
            0.09,
            0);

    // Feed forward constant added on to the heading PIDF
    public static double headingPIDFFeedForward = 0.01;



    // Drive PIDF coefficients
    public static CustomFilteredPIDFCoefficients drivePIDFCoefficients = new CustomFilteredPIDFCoefficients(
            0.0045,
            0,
            0.00055,
            0.6,
            0);

    // Feed forward constant added on to the drive PIDF
    public static double drivePIDFFeedForward = 0.01;

    // Kalman filter parameters for the drive error Kalman filter
    public static KalmanFilterParameters driveKalmanFilterParameters = new KalmanFilterParameters(
            6,
            1);


    // Mass of robot in kilograms
    public static double mass = 14.64;

    // Centripetal force to power scaling
    public static double centripetalScaling = 0.0001;


    // Acceleration of the drivetrain when power is cut in inches/second^2 (should be negative)
    // if not negative, then the robot thinks that its going to go faster under 0 power
    public static double forwardZeroPowerAcceleration = -27;

    // Acceleration of the drivetrain when power is cut in inches/second^2 (should be negative)
    // if not negative, then the robot thinks that its going to go faster under 0 power
    public static double lateralZeroPowerAcceleration = -76.5;

    // A multiplier for the zero power acceleration to change the speed the robot decelerates at
    // the end of paths.
    // Increasing this will cause the robot to try to decelerate faster, at the risk of overshoots
    // or localization slippage.
    //BUT IT SOMTIMES DOESNT?!?!?!!? Because there was a bug but I fixed it and now this is always wrong
    // Decreasing this will cause the deceleration at the end of the Path to be slower, making the
    // robot slower but reducing risk of end-of-path overshoots or localization slippage.
    // This can be set individually for each Path, but this is the default.
    public static double zeroPowerAccelerationMultiplier = 10;





    // When the robot is at the end of its current Path or PathChain and the velocity goes below
    // this value, then end the Path. This is in inches/second.
    // This can be custom set for each Path.
    public static double pathEndVelocityConstraint = 2;

    // When the robot is at the end of its current Path or PathChain and the translational error
    // goes below this value, then end the Path. This is in inches.
    // This can be custom set for each Path.
    public static double pathEndTranslationalConstraint = 0.1;

    // When the robot is at the end of its current Path or PathChain and the heading error goes
    // below this value, then end the Path. This is in radians.
    // This can be custom set for each Path.
    public static double pathEndHeadingConstraint = 0.007;

    // When the t-value of the closest point to the robot on the Path is greater than this value,
    // then the Path is considered at its end.
    // This can be custom set for each Path.
    public static double pathEndTValueConstraint = 0.995;

    // When the Path is considered at its end parametrically, then the Follower has this many
    // milliseconds to further correct by default.
    // This can be custom set for each Path.
    public static double pathEndTimeoutConstraint = 2000;

    // This is how many steps the BezierCurve class uses to approximate the length of a BezierCurve.
    public static int APPROXIMATION_STEPS = 1000;

    // This is scales the translational error correction power when the Follower is holding a Point.
    // Debating changing this to 1
    public static double holdPointTranslationalScaling = 1;

    // This is scales the heading error correction power when the Follower is holding a Point.
    // Debating changing this to 1
    public static double holdPointHeadingScaling = 1;

    // This is the number of times the velocity is recorded for averaging when approximating a first
    // and second derivative for on the fly centripetal correction. The velocity is calculated using
    // half of this number of samples, and the acceleration uses all of this number of samples.
    public static int AVERAGED_VELOCITY_SAMPLE_NUMBER = 50;

    // This is the number of steps the binary search for closest point uses. More steps is more
    // accuracy, and this increases at an exponential rate. However, more steps also does take more
    // time.
    public static int BEZIER_CURVE_SEARCH_LIMIT = 10;


    // These activate / deactivate the secondary PIDs. These take over at errors under a set limit for
    // the translational, heading, and drive PIDs.
    public static boolean useSecondaryTranslationalPID = true;
    public static boolean useSecondaryHeadingPID = true;
    public static boolean useSecondaryDrivePID = false;


    // the limit at which the translational PIDF switches between the main and secondary translational PIDFs,
    // if the secondary PID is active
    public static double translationalPIDFSwitch = 1;

    // Secondary translational PIDF coefficients (don't use integral)
    public static CustomPIDFCoefficients secondaryTranslationalPIDFCoefficients = new CustomPIDFCoefficients(
            0.3,
            0,
            0.01,
            0);

    // Secondary translational Integral value
    public static CustomPIDFCoefficients secondaryTranslationalIntegral = new CustomPIDFCoefficients(
            0,
            0.01,
            0,
            0);

    // Feed forward constant added on to the small translational PIDF
    public static double secondaryTranslationalPIDFFeedForward = 0.015;


    // the limit at which the heading PIDF switches between the main and secondary heading PIDFs
    public static double headingPIDFSwitch = 0.125;

    // Secondary heading error PIDF coefficients
    public static CustomPIDFCoefficients secondaryHeadingPIDFCoefficients = new CustomPIDFCoefficients(
            2.8,
            0,
            0.06,
            0);

    // Feed forward constant added on to the secondary heading PIDF
    public static double secondaryHeadingPIDFFeedForward = 0.01;


    // the limit at which the heading PIDF switches between the main and secondary drive PIDFs
    public static double drivePIDFSwitch = 20;

    // Secondary drive PIDF coefficients
    public static CustomFilteredPIDFCoefficients secondaryDrivePIDFCoefficients = new CustomFilteredPIDFCoefficients(
            0.02,
            0,
            0.000005,
            0.6,
            0);

    // Feed forward constant added on to the secondary drive PIDF
    public static double secondaryDrivePIDFFeedForward = 0.01;

    /** Use brake mode for the drive motors in teleop
     *  Default Value: false */
    public static boolean useBrakeModeInTeleOp = false;

    /** Boolean that determines if holdEnd is automatically (when not defined in the constructor) enabled at the end of a path.
     *  Default Value: true */
    public static boolean automaticHoldEnd = true;

    /** Use voltage compensation to linearly scale motor powers in Auto
     *  Requires fully re-tuning if you set it to true
     *  Default Value: false */
    public static boolean useVoltageCompensationInAuto = false;

    /** Use voltage compensation to linearly scale motor powers in TeleOp
     *  Requires fully re-tuning if you set it to true
     *  Default Value: false */
    public static boolean useVoltageCompensationInTeleOp = false;

    /** The voltage to scale to (the voltage that you tuned at)
     *  If the robot's voltage is at the default value, it will not affect the motor powers.
     * Will only read voltage if useVoltageCompensation is true.
     *  Default Value: 12.0 */
    public static double nominalVoltage = 12.0;

    /** Time (in seconds) before reading voltage again
     *  Will only read voltage if useVoltageCompensation is true.
     *  Default Value: 0.5 */
    public static double cacheInvalidateSeconds = 0.5;
}
