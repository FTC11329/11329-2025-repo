package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.BezierCurve;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Path;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Point;
import org.firstinspires.ftc.teamcode.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.PowerTakeOff;
import org.firstinspires.ftc.teamcode.subsystems.PressHold;
import org.firstinspires.ftc.teamcode.utility.DriveSpeedEnum;
import org.firstinspires.ftc.teamcode.utility.PlacePosEnum;
import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;
import org.firstinspires.ftc.teamcode.utility.StateMachine;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.FileWriter;
import java.io.IOException;
import android.os.Environment;

import org.json.JSONTokener;

import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;
import java.io.*;

public class NewTeleopBlueSecret {
    //Delete me
    DcMotorEx motor1, motor2, motor3, motor4, motor5, motor6, motor7, motor8;
    Climber climber;
    Follower follower;
    Drivetrain driveTrain;
    PowerTakeOff powerTakeOff;
    IntakeSystem intakeSystem;
    OuttakeSystem outtakeSystem;

    StateMachine stateMachine;
    ElapsedTime elapsedTime = new ElapsedTime();

    FtcDashboard dashboard;

    //Debug Variables
    boolean debugAll = false;
    boolean debugState = false;
    boolean debugStateMachine = false;
    boolean debugPos = false;
    boolean debugClimber = false;
    boolean debugAuto = false;
    boolean debugMisc = false;

    //Auto Variables
    private final Pose frontWall  = new Pose(38, -54, Math.toRadians(90));
    private final Pose pickupWall = new Pose(38, -61, Math.toRadians(90));
    private final Pose placeSub = new Pose(0, -48, Math.toRadians(90));

    private final Pose controlPointForSubPlace = new Pose(-10, -63, 0);
    private final Pose controlPointForWall1 = new Pose(12, -50, 0);
    private final Pose controlPointForWall2 = new Pose(42, -18, 0);

    private Path toFrontWall;
    private Path frontWallToWall;
    private Path placeSubPath;

    //Input Variables
    double driveForward;
    double driveStrafe;
    double driveRotation;
    DriveSpeedEnum driveSpeed;
    boolean climbToggButton;

    double manualVSlide;
    double manualHSlide;
    double manualArm;
    double manualClimber;

    boolean sideDepo;

    boolean highSpecimen;
    boolean highBasket;
    boolean lowBasket;
    boolean frontBasket;
    boolean wallPreset;
    boolean storePos;
    boolean transfer;

    boolean clawToggleButton;

    boolean intake;
    boolean intakeColor;
    boolean unJam;

    //State Machine Variables
    boolean hasInIntake = false;;
    boolean atStorePos = false;;
    boolean hasInOuttake = false;
    boolean hasInTray = false;
    PlacePosEnum whereAmI = PlacePosEnum.wall;

    //Various Variables
    int climberStage = 0;
    Timer climberTimer = new Timer();
    int climberPos = 0;

    Timer pathTimer = new Timer();
    boolean autoMovement = false;
    boolean autoMovementOnce = true;
    boolean autoToWall = false;
    boolean autoToSub = false;
    int autoState = 0;

    boolean climberActive = false;
    boolean climbPause = false;
    boolean climbDebounce = false;
    boolean lastCurrentTrip = false;
    double lastCurrentTripTime = 2000000000;
    boolean climbStopPause = false;
    boolean climbStopPauseOnce = true;

    boolean onceTime = true;
    boolean onceState = true;
    boolean wallOnce = true;
    double storeTime = 2000000000;
    double transferTime = 2000000000;
    double unStoringTime = 2000000000;
    double wallTime = 2000000000;
    double lastTime = 2000000000;

    boolean transferFirstTime = true;

    boolean clawToggle = true;
    boolean clawDebounce = false;

    boolean grabbingOffWall = false;
    double grabbingOffWallTime = 2000000000;
    boolean droppingBasket = false;
    double droppingBasketTime = 2000000000;
    double sideDepoTime = 2000000000;
    boolean sideDepoFirst = false;
    boolean sideDepoDebounce = false;

    boolean sideDepoing = false;

    boolean intakeingColor = false;
    boolean intakeing = false;
    boolean intakeingDebounce = false;
    double intakeWristTime = 2000000000;
    int extendHSlide = Constants.Intake.intakeSlidePos;

    boolean unjamming = false;
    double unjammingTime = 2000000000;
    boolean unjamAfterIntake = false;
    double unjamAfterIntakeTime = 2000000000;

    PressHold recording;
    PressHold replay;
    double lastTimer = 0;
    double deltaTime = 0.5;
    double deltaError = 0.4;
    int replayIndex = 0;


    //this is here because I have to have a teleop blue and teleop red
    HardwareMap hardwareMap;
    Telemetry telemetry;
    Gamepad gamepad1;
    Gamepad gamepad2;
    RobotSideEnum robotSide;

    public NewTeleopBlueSecret(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, RobotSideEnum robotSide) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.robotSide = robotSide;
    }

    public void init() {
        //delete me
        motor1 = hardwareMap.get(DcMotorEx.class, "leftFront");
        motor2 = hardwareMap.get(DcMotorEx.class, "rightFront");
        motor3 = hardwareMap.get(DcMotorEx.class, "rightBack");
        motor4 = hardwareMap.get(DcMotorEx.class, "leftBack");
        motor5 = hardwareMap.get(DcMotorEx.class, "hSlides");
        motor6 = hardwareMap.get(DcMotorEx.class, "vSlides");
        motor7 = hardwareMap.get(DcMotorEx.class, "climber");
        motor8 = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        //uncomment if you want telemetry on dashboard
//        dashboard = FtcDashboard.getInstance();
//        telemetry = dashboard.getTelemetry();

        climber = new Climber(hardwareMap);
        follower = new Follower(hardwareMap);
        driveTrain = new Drivetrain(hardwareMap);

        stateMachine = new StateMachine();
        recording = new PressHold();
        replay = new PressHold();


        //Building paths
        toFrontWall = new Path(new BezierCurve(new Point(placeSub), new Point(controlPointForWall1), new Point(controlPointForWall2), new Point(frontWall)));
        toFrontWall.setConstantHeadingInterpolation(placeSub.getHeading());

        frontWallToWall = follower.linearPathBuilder(frontWall, pickupWall);

        placeSubPath = new Path(new BezierCurve(new Point(pickupWall), new Point(controlPointForSubPlace), new Point(placeSub)));
        placeSubPath.setConstantHeadingInterpolation(placeSub.getHeading());

    }

    public void start() {
        intakeSystem = new IntakeSystem(hardwareMap, robotSide);
        powerTakeOff = new PowerTakeOff(hardwareMap);
        outtakeSystem = new OuttakeSystem(hardwareMap, true);
        outtakeSystem.setArmPos(Constants.Outtake.upArm);
        elapsedTime.reset();
    }

    public void recordPosition(double time, Pose pos) {
        File dir = AppUtil.getInstance().getSettingsFile("TeamCodeLogs").getParentFile();
        File file = new File(dir, "movement.json");
        JSONArray log;
        telemetry.addData("Recording, time: ", time);
        try {
            if (file.exists()) {
                FileReader reader = new FileReader(file);
                BufferedReader bufferedReader = new BufferedReader(reader);
                StringBuilder content = new StringBuilder();
                String line;
                while ((line = bufferedReader.readLine()) != null) {
                    content.append(line);
                }
                bufferedReader.close();
                log = new JSONArray(new JSONTokener(content.toString()));
                reader.close();
            } else {
                log = new JSONArray();
            }

            // Create position JSON
            JSONObject posJson = new JSONObject();
            posJson.put("x", pos.getX());
            posJson.put("y", pos.getY());
            posJson.put("heading", pos.getHeading());

            JSONObject entry = new JSONObject();
            entry.put("t", time);
            entry.put("pos", posJson);

            log.put(entry);
            BufferedWriter writer = new BufferedWriter(new FileWriter(file));
            writer.write(log.toString(2));
            writer.close();

            telemetry.addData("Drive log written", true);
        } catch (Exception e) {
            telemetry.addData("Drive log error", e.getMessage());
        }
    }

    public Pose loadPoses() {
        Pose pose = null;

        File dir = AppUtil.getInstance().getSettingsFile("TeamCodeLogs").getParentFile();
        File file = new File(dir, "movement.json");

        if (!file.exists()) return null;

        try (FileReader reader = new FileReader(file)) {
            BufferedReader bufferedReader = new BufferedReader(reader);
            StringBuilder content = new StringBuilder();
            String line;
            while ((line = bufferedReader.readLine()) != null) {
                content.append(line);
            }
            bufferedReader.close();

            JSONArray log = new JSONArray(new JSONTokener(content.toString()));
            if (replayIndex < log.length()) {
                JSONObject entry = log.getJSONObject(replayIndex);
                double t = entry.getDouble("t");

                if (follower.driveError > deltaError) {  // your logic
                    JSONObject posJson = entry.getJSONObject("pos");
                    double x = posJson.getDouble("x");
                    double y = posJson.getDouble("y");
                    double heading = posJson.getDouble("heading");

                    pose = new Pose(x, y, heading);
                    replayIndex += 1;
                }
            }
        } catch (Exception e) {
            telemetry.addData("Failed to Load", true);
        }

        return pose;
    }

    public void clearLog() {
        File dir = AppUtil.getInstance().getSettingsFile("TeamCodeLogs").getParentFile();
        if (!dir.exists()) dir.mkdirs();  // Ensure the directory exists

        File file = new File(dir, "movement.json");

        telemetry.addData("Clearing log file at: ", file.getAbsolutePath());
        telemetry.addData("Does folder exist: ", dir.exists());

        try (FileWriter writer = new FileWriter(file)) {
            writer.write("[]");  // Reset the file with an empty JSON array
            telemetry.addData("Log cleared successfully", true);
        } catch (IOException e) {
            telemetry.addData("Error clearing log", e.getMessage());
        }
    }


    public void loop() {
        // Inputs
        debugAll = false;
        driveForward = -gamepad1.left_stick_y;
        driveStrafe = -gamepad1.left_stick_x;
        driveRotation = -gamepad1.right_stick_x;
        recording.checkStatus(gamepad1.a);
        replay.checkStatus(gamepad1.b);
//        telemetry.addData("record time in loop: ", recording.time.seconds());
//        telemetry.addData("record start in loop: ", recording.startPress);
//        telemetry.addData("record is Pressed in loop: ", recording.isPressed);
//        telemetry.addData("record is on in loop: ", recording.isOn);
//        telemetry.addData("record last timer in loop: ", lastTimer);
//        telemetry.addData("record should record in loop: ", (recording.time.seconds() - lastTimer) > deltaTime);
        if(recording.startPress){
            clearLog();
            lastTimer = 0;
        }
        if (recording.isOn) {
            telemetry.addData("Hello World", 1);
            if (recording.time.seconds() - lastTimer > deltaTime) {
                recordPosition(recording.time.seconds(), follower.getPose());
                telemetry.addData("Hello World", 2);
                telemetry.update();
                lastTimer = recording.time.seconds();
            }
        } else if (replay.isOn){
            Pose target = loadPoses();
            if (target != null){
                Path next = follower.linearPathBuilder(follower.getPose(), target);
                follower.followPath(next);
                telemetry.addData("Hello World", target.getX());
            }
            follower.update();
        }
        telemetry.addData("Hello World", true);

        if (gamepad1.right_bumper) {
            driveSpeed = DriveSpeedEnum.Fast;
        } else {
            driveSpeed = DriveSpeedEnum.Slow;
        }

        climbToggButton = gamepad1.back;

        manualVSlide = -gamepad2.right_stick_y;
        if (!climberActive) {
            manualHSlide = gamepad2.right_trigger + (0.6 *(-gamepad2.left_trigger + gamepad1.right_trigger - gamepad1.left_trigger));
            if (debugAll) {
                manualHSlide = manualHSlide * 2;
            }
            manualArm = gamepad2.left_stick_y;
            manualClimber = gamepad1.right_trigger - gamepad1.left_trigger;
        } else {
            manualHSlide = 0;
            manualArm = 0;
            manualClimber = 0;
        }

        sideDepo = gamepad2.touchpad && gamepad2.touchpad_finger_1_x > 0;

        highSpecimen = gamepad2.dpad_up;
        highBasket = gamepad2.dpad_right;
        lowBasket = gamepad2.dpad_down;
        frontBasket = gamepad2.touchpad && gamepad2.touchpad_finger_1_x < 0;
        wallPreset = gamepad2.dpad_left;
        storePos = gamepad2.triangle;//y
        transfer = gamepad2.left_bumper;

        clawToggleButton = gamepad1.left_bumper || gamepad2.right_bumper;

        intake = gamepad2.cross;//a
        intakeColor = gamepad2.square;//x
        unJam = gamepad2.circle;//b

        //Drivetrain ******************************************************************************~
        if (!climberActive && !autoMovement && !climbPause) {
            if (climbToggButton) {
                climberActive = true;
                climbDebounce = true;
            }
            //Drive time
            driveTrain.drive(driveForward, driveStrafe, driveRotation, driveSpeed);

            if (climberActive) {
                driveTrain.setRunToPos();
                climberTimer.resetTimer();
            }
            //Autos
            if (gamepad1.x) {
                autoMovement = true;
                autoMovementOnce = true;

                autoToSub = true;
                autoToWall = false;
            }
            if (gamepad1.y) {
                autoMovement = true;
                autoMovementOnce = true;

                autoToSub = false;
                autoToWall = true;
            }
        }
        if (gamepad1.b) {
            autoMovement = false;
            autoMovementOnce = true;
            follower.breakFollowing();
        }
        //Auto drive ******************************************************************************~
        if (autoMovement) {
            if (autoMovementOnce) {
                if (autoToSub) {
                    follower.setPose(new Pose(pickupWall.getY() + 1.5, pickupWall.getX(), pickupWall.getHeading()));
                    autoState = 3;
                }
                if (autoToWall) {
                    autoState = 0;
                }
                autoMovementOnce = false;
            }
            //back to front wall
            switch (autoState) {
                case 0:
                    follower.followPath(toFrontWall);

                    pathTimer.resetTimer();
                    autoState = 1;
                    break;
                case 1:
                    if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                        outtakeSystem.placePos(PlacePosEnum.wallAuto);
                        whereAmI = PlacePosEnum.wall;

                        pathTimer.resetTimer();
                        autoState = 2;
                    }
                    break;
                //front Wall To Wall
                case 2:
                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                        follower.followPath(frontWallToWall);

                        pathTimer.resetTimer();
                        autoState = 3;
                    }
                    break;
                //grab off wall and go to sub
                case 3:
                    if (follower.getError(pickupWall).getX() < 1 && follower.getError(pickupWall).getY() < 1 || pathTimer.getElapsedTimeSeconds() > 1) {
                        outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                        clawToggle = true;
                        hasInOuttake = true;

                        pathTimer.resetTimer();
                        autoState = 4;
                    }
                    break;
                //outtake to specimen place pos
                case 4:
                    if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                        follower.followPath(placeSubPath);
                        outtakeSystem.placePos(PlacePosEnum.highSpecimen);
                        whereAmI = PlacePosEnum.highSpecimen;

                        pathTimer.resetTimer();
                        autoState = 5;
                    }
                    break;
                //drop specimen
                case 5:
                    if (follower.getError(placeSub).getY() < 1) {
                        follower.breakFollowing();
                        autoMovement = false;
                        autoMovementOnce = true;

                        pathTimer.resetTimer();
                        autoState = 6;
                    }
                    break;
            }
            follower.update();
        }
        //Auto Climb ******************************************************************************~
        if (climberActive && !climbPause) {
            if (climbToggButton && !climbDebounce) {
                climbPause = true;
                climber.setPos(climber.getPos());
                driveTrain.setPTOPos(driveTrain.getPTOPos());
            }
            if (!climbToggButton) {
                climbDebounce = false;
            }

            double current;
            switch (climberStage) {
                case 0:
                    //puts arm to safe space
                    if (outtakeSystem.getArmPos() > 0.6) {
                        outtakeSystem.setVSlidePos(Constants.Outtake.maxSlides);
                        outtakeSystem.setArmPos(Constants.Outtake.intakeArm);
                    } else {
                        outtakeSystem.setVSlidePos(Constants.Outtake.maxSlides);
                        outtakeSystem.setArmPos(Constants.Outtake.intakeWallArm);
                    }

                    intakeSystem.storePos();
                    intakeSystem.setIntakeServoPos(Constants.Intake.wristClimb);

                    climberPos = Constants.Climber.outPos;

                    //Enable PTO
                    driveTrain.setRunToPos();
                    powerTakeOff.enable();

                    climberTimer.resetTimer();
                    climberStage = 1;
                    break;
                case 1:
                    driveTrain.moveBackWheels();
                    if (climberTimer.getElapsedTimeSeconds() > 1.5 && Math.abs(climber.getPos() - climberPos) < 500) {
                        driveTrain.setPTOPos(Constants.PTO.motorClimb);

                        climberTimer.resetTimer();
                        climberStage = 2;
                    }
                    break;
                case 2:
                    driveTrain.setPTOPower(1);
                    if (climber.getDistance() > 10.5) {
                        climberPos = Constants.Climber.hookPos;
                        outtakeSystem.setVSlidePos(Constants.Outtake.maxSlides);
                        //Prevent pto from drawing too much power
                        driveTrain.setPTOPos(driveTrain.getPTOPos());

                        climberTimer.resetTimer();
                        climberStage = 3;
                    }
                    break;
                case 3:
                    driveTrain.setPTOPower(0.8);
                    if (Math.abs(climber.getPos() - Constants.Climber.hookPos) < 100) {
                        //disable PTO to conserve power
                        driveTrain.setPTOPower(0);

                        climberPos = Constants.Climber.inPos;

                        climberTimer.resetTimer();
                        climberStage = 4;
                    }
                    break;
                case 4:
                    driveTrain.setPTOPower(0);
                    if (Math.abs(climber.getPos() - Constants.Climber.inPos) < 1000) {

                        climberTimer.resetTimer();
                        climberStage = 5;
                    }
                    break;
                case 5:
                    driveTrain.setPTOPower(-0.9);
                    //Does some things to make sure that the current has been tripped for more than 1 second after one one second
                    if (climberTimer.getElapsedTimeSeconds() > 0.25) {
                        current = Math.min(Math.max(driveTrain.getDriveCurrent()[0], driveTrain.getDriveCurrent()[1]), Math.max(driveTrain.getDriveCurrent()[2], driveTrain.getDriveCurrent()[3]));
                    } else {
                        current = 0;
                    }

                    if (current > 4 && !lastCurrentTrip) {
                        lastCurrentTripTime = elapsedTime.milliseconds();
                        lastCurrentTrip = true;
                    } else if (current < 4) {
                        lastCurrentTrip = false;
                    }

                    if (current > 4 && elapsedTime.milliseconds() > lastCurrentTripTime + 300) {
                        //Prevent pto from drawing too much power
                        driveTrain.setPTOPos(driveTrain.getPTOPos());

                        climberTimer.resetTimer();
                        climberStage = 6;
                    }
                    break;
                case 6:
                    driveTrain.setPTOPower(-0.2);
            }

            //manual movement
            climberPos += (int) (20 * (manualClimber));
            climber.setPos(climberPos);
        }
        if (climbPause) {
            if (gamepad1.b) {
                climbStopPause = true;
            }
            if (!climbStopPause) {
                climber.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
                driveTrain.setPTOPower(-gamepad1.left_stick_y);
            }

            if (climbStopPause && climbStopPauseOnce) {
                climbStopPauseOnce = false;
                climber.setPos(climber.getPos());
            }

            if (climbStopPause) {
                driveTrain.setPTOPower(-0.2);
            }
        }
        // Pre-Start Climb
        if (gamepad1.dpad_up || gamepad2.back) {
            climberPos = Constants.Climber.outPos;
            climber.setPos(climberPos);
        }

        //Manual Movements ************************************************************************~
        intakeSystem.manualHSlide(manualHSlide);
        outtakeSystem.manualVSlide(manualVSlide);
        outtakeSystem.manualArm(manualArm);


        intakeSystem.update();
        outtakeSystem.update(Math.abs(manualVSlide) > 0.05 || outtakeSystem.overAmp());

        //Presets
        //Button to State Machine class ***********************************************************~
        if (highSpecimen) {
            if (!stateMachine.doTransfer()) {
                onceTime = true;
            }
            intakeSystem.storePos();
            stateMachine.goHighSpecimen(atStorePos);
        }
        if (lowBasket) {
            if (!stateMachine.doTransfer()) {
                onceTime = true;
            }
            intakeingColor = false;
            intakeing = false;
            stateMachine.goLowBasket(hasInIntake || hasInTray, hasInOuttake, atStorePos);
        }
        if (highBasket) {
            if (!stateMachine.doTransfer()) {
                onceTime = true;
            }
            intakeingColor = false;
            intakeing = false;
            stateMachine.goHighBasket(hasInIntake || hasInTray, hasInOuttake, atStorePos);
        }
        if (frontBasket) {
            if (!stateMachine.doTransfer()) {
                onceTime = true;
            }
            intakeingColor = false;
            intakeing = false;
            stateMachine.goFrontBasket(hasInIntake || hasInTray, hasInOuttake, atStorePos);
        }
        if (wallPreset) {
            if (!stateMachine.doTransfer()) {
                onceTime = true;
            }
            intakeSystem.storePos();
            intakeingColor = false;
            intakeing = false;
            stateMachine.goWall(hasInIntake || hasInTray, hasInOuttake, atStorePos);
        }
        if (storePos) {
            onceTime = true;
            stateMachine.resetValues();
            intakeSystem.storePos();
            intakeingColor = false;
            intakeing = false;
            stateMachine.goStore();
            extendHSlide = Constants.Intake.intakeSlidePos;
        }
        if (transfer) {
            if (!stateMachine.doTransfer() && !stateMachine.doUnStore()) {
                onceTime = true;
            }
            intakeingColor = false;
            intakeing = false;
            stateMachine.goTransfer(atStorePos);
        }


        //State Machine class to movement *********************************************************~
        if (stateMachine.doGoToStore()) {
            if (onceTime) {
                outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                clawToggle = false;
                if (whereAmI == PlacePosEnum.highSpecimen) {
                    outtakeSystem.setVSlidePos(Constants.Outtake.safeFromSpecBar);
                } else {
                    outtakeSystem.setVSlidePos(Constants.Outtake.safeFromClimberBar);
                }
                intakeSystem.setHSlidePos(Constants.Intake.transferSlides);
                onceState = true;

                onceTime = false;
            }
            if (Math.abs(outtakeSystem.getVSlidePos() - Constants.Outtake.safeFromClimberBar) < 200 && onceState) {
                storeTime = elapsedTime.milliseconds();
                outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                outtakeSystem.setArmPos(Constants.Outtake.intakeArm);
                onceState = false;
            }
            if (elapsedTime.milliseconds() > storeTime + 500 && elapsedTime.milliseconds() < storeTime + 600) {
                outtakeSystem.setVSlidePos(Constants.Outtake.intakeSlides);

                atStorePos = true;
                whereAmI = PlacePosEnum.intake;
                onceState = false;
                onceTime = true;
                stateMachine.finishGoToStoreFromSpec();
            }
        }

        if (stateMachine.doTransfer()) {
            if (onceTime) {
                transferTime = elapsedTime.milliseconds();
                onceTime = false;
                transferFirstTime = true;
                if (!intakeSystem.readyToTransfer()) {
                    transferTime = elapsedTime.milliseconds() + Constants.Intake.intakeServoSpeedTime;
                }
                outtakeSystem.placePos(PlacePosEnum.intake);
            }
            if (elapsedTime.milliseconds() > transferTime + 0 && elapsedTime.milliseconds() < transferTime + 100 && transferFirstTime) {
                intakeSystem.setIntakePower(Constants.Intake.transferSpeed);
                outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
            }
            if (outtakeSystem.readyToTransfer() && intakeSystem.readyToTransfer() && transferFirstTime) {
                intakeSystem.setIntakePower(0);
                outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                clawToggle = true;
                hasInTray = false;
                transferFirstTime = false;
                transferTime = elapsedTime.milliseconds();
            }
            if (elapsedTime.milliseconds() > transferTime + 1500 && transferFirstTime) {
                intakeSystem.setIntakePower(0);
                hasInIntake = false;
                hasInTray = true;
                stateMachine.failTransfer();
            }
            if (elapsedTime.milliseconds() > transferTime + 300 && elapsedTime.milliseconds() < transferTime + 400 && !transferFirstTime) {
                hasInOuttake = true;
                hasInIntake = false;
                onceState = true;
                onceTime = true;
                transferFirstTime = true;
                stateMachine.finishTransfer();
            }
        }

        if (stateMachine.doUnStore()) {
            if (onceTime) {
                outtakeSystem.setVSlidePos(Constants.Outtake.safeFromClimberBar);
                intakeSystem.setIntakeServoPos(Constants.Intake.wristClear);
                onceState = true;

                onceTime = false;
            }
            if (Math.abs(outtakeSystem.getVSlidePos() - Constants.Outtake.safeFromClimberBar) < 100 && onceState) {
                atStorePos = false;
                outtakeSystem.setArmPos(Constants.Outtake.initTeleopArm);
                intakeSystem.setIntakeServoPos(Constants.Intake.wristStore);
                intakeSystem.setHSlidePos(0);
                unStoringTime = elapsedTime.milliseconds();
                onceState = false;
            }
            if (elapsedTime.milliseconds() > unStoringTime + 200 && elapsedTime.milliseconds() < unStoringTime + 400) {
                atStorePos = false;
                onceState = true;
                onceTime = true;
                stateMachine.finishUnStore();
            }
        }

        if (stateMachine.doHighSpecimen()) {
            clawToggle = true;
            outtakeSystem.placePos(PlacePosEnum.highSpecimen);
            whereAmI = PlacePosEnum.highSpecimen;
            stateMachine.finishHighSpecimen();
        }

        if (stateMachine.doLowBasket()) {
            if (onceTime) {
                outtakeSystem.setVSlidePos(Constants.Outtake.lowBasketSlides);
                outtakeSystem.setArmPos(Constants.Outtake.upArm);
                whereAmI = PlacePosEnum.lowBasket;
                onceTime = false;
            }
            if (outtakeSystem.getVSlidePos() > outtakeSystem.getVSlideTargetPos() - 50) {
                outtakeSystem.setArmPos(Constants.Outtake.basketArm);
                stateMachine.finishLowBasket();
                onceTime = true;
            }
        }

        if (stateMachine.doHighBasket()) {
            if (onceTime) {
                outtakeSystem.setVSlidePos(Constants.Outtake.highBasketSlides);
                outtakeSystem.setArmPos(Constants.Outtake.upArm);
                whereAmI = PlacePosEnum.highBasket;
                onceTime = false;
            }
            if (outtakeSystem.getVSlidePos() > outtakeSystem.getVSlideTargetPos() - 150) {
                outtakeSystem.setArmPos(Constants.Outtake.basketArm);
                stateMachine.finishHighBasket();
                onceTime = true;
            }
        }

        if (stateMachine.doFrontBasket()) {
            if (onceTime) {
                outtakeSystem.setVSlidePos(Constants.Outtake.highBasketSlides);
                outtakeSystem.setArmPos(Constants.Outtake.upArm);
                whereAmI = PlacePosEnum.highBasket;
                onceTime = false;
            }
            if (outtakeSystem.getVSlidePos() > outtakeSystem.getVSlideTargetPos() - 150) {
                outtakeSystem.setArmPos(Constants.Outtake.frontBasketArm);
                stateMachine.finishFrontBasket();
                onceTime = true;
            }
        }

        if (stateMachine.doWall()) {
            if (onceTime) {
                outtakeSystem.placePos(PlacePosEnum.wall);
                whereAmI = PlacePosEnum.wall;
                if (!hasInOuttake) {
                    outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                    clawToggle = false;
                }

                intakeSystem.storePos();
                wallTime = elapsedTime.milliseconds();
                onceTime = false;
            }
            //transferring into tray
            if (elapsedTime.milliseconds() > wallTime + 0 && elapsedTime.milliseconds() < wallTime + 100 && hasInIntake) {
                intakeSystem.setIntakePower(Constants.Intake.transferSpeed);
            }
            if (elapsedTime.milliseconds() > wallTime + 800 && elapsedTime.milliseconds() < wallTime + 900 && hasInIntake) {
                intakeSystem.setIntakePower(0);
                hasInIntake = false;
                hasInTray = true;
                onceTime = true;
                stateMachine.finishWall();
            }

        }


        // Claw Controls **************************************************************************~
        if (!clawToggle) {
            hasInOuttake = false;
        }
        if (clawToggleButton && !clawDebounce) {
            clawToggle = !clawToggle;
            clawDebounce = true;

            // Grab
            if (clawToggle && whereAmI == PlacePosEnum.wall) {
                grabbingOffWall = true;
                wallOnce = true;
            } else if (clawToggle) {
                outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
            }

            // Drop
            if (!clawToggle && (whereAmI == PlacePosEnum.lowBasket || whereAmI == PlacePosEnum.highBasket)) {
                hasInOuttake = false;
                outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                droppingBasket = true;
                droppingBasketTime = elapsedTime.milliseconds();
            } else if (!clawToggle && whereAmI == PlacePosEnum.highSpecimen) {
                hasInOuttake = false;
                outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
            } else if (!clawToggle) {
                hasInOuttake = false;
                outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
            }
        }
        if (!clawToggleButton) {
            clawDebounce = false;
        }

        if (grabbingOffWall) {
            if ((outtakeSystem.seesWall() || !clawToggleButton) && wallOnce) {
                outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                hasInOuttake = true;

                grabbingOffWallTime = elapsedTime.milliseconds();
                wallOnce = false;
            }
            if (elapsedTime.milliseconds() > grabbingOffWallTime + 300 && !wallOnce) {
                outtakeSystem.setVSlidePos(Constants.Outtake.safeFromWallSlides);
                grabbingOffWall = false;
                wallOnce = true;
            }
        }

        if (droppingBasket) {
            if (clawToggleButton) {
                droppingBasketTime = elapsedTime.milliseconds();
            }
            if (elapsedTime.milliseconds() > droppingBasketTime + 300 && elapsedTime.milliseconds() < droppingBasketTime + 400) {
                outtakeSystem.setArmPos(Constants.Outtake.upArm);
            }
            if (elapsedTime.milliseconds() > droppingBasketTime + 500) {
                stateMachine.goStore();
                droppingBasket = false;
            }
        }

        //Intakes *********************************************************************************~
        if (gamepad1.touchpad_finger_1) {
            extendHSlide = (int) ((gamepad1.touchpad_finger_1_x + 1)/2.0 * Constants.Intake.maxSlidePos);
            if (gamepad1.touchpad) {
                intakeSystem.setHSlidePos(extendHSlide);
            }
        }

        if (intakeColor && !intakeingColor && !intakeingDebounce) {
            intakeSystem.setHSlidePos(extendHSlide);
            intakeSystem.setIntakeServoPos(Constants.Intake.wristClear);
            intakeingColor = true;
            intakeing = false;
            //do we want this?
            hasInTray = false;
            intakeWristTime = elapsedTime.milliseconds();
        }
        if (intake && !intakeing && !intakeingDebounce) {
            intakeSystem.setHSlidePos(extendHSlide);
            intakeSystem.setIntakeServoPos(Constants.Intake.wristClear);
            intakeingColor = false;
            intakeing = true;
            //do we want this?
            hasInTray = false;
            intakeWristTime = elapsedTime.milliseconds();
        }

        if (intakeing) {
            if (intakeSystem.getIntakeServoPos() > Constants.Intake.wristClear && intakeSystem.intakeUntil()) {
                if (atStorePos) {
                    stateMachine.goTransfer(true);
                    intakeSystem.storeOutPos();
                } else {
                    intakeSystem.storePos();
                }
                gamepad1.rumble(0,1,300);
                gamepad2.rumble(0,1,300);
                extendHSlide = Constants.Intake.intakeSlidePos;
                hasInIntake = true;
                intakeing = false;
                unjamAfterIntakeTime = elapsedTime.milliseconds();
                unjamAfterIntake = true;
            }
        }

        if (intakeingColor) {
            if (intakeSystem.getIntakeServoPos() > Constants.Intake.wristClear && intakeSystem.intakeUntilColor()) {
                if (atStorePos) {
                    intakeSystem.storeOutPos();
                } else {
                    intakeSystem.storePos();
                }
                gamepad1.rumble(0, 1, 300);
                gamepad2.rumble(0, 1, 300);
                extendHSlide = Constants.Intake.intakeSlidePos;
                hasInIntake = true;
                intakeingColor = false;
                unjamAfterIntakeTime = elapsedTime.milliseconds();
                unjamAfterIntake = true;
            }
        }

        //makes the intake wrist not hit the robot while coming out
        if (((intakeingColor && !intakeColor) || (intakeing && !intake)) && !hasInOuttake) {
            intakeingDebounce = false;
            intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
            intakeWristTime = 2000000000;
        }

        //SIDE DEPOSIT*****************************************************************************~
        if (sideDepo && !sideDepoDebounce && !intakeing && !intakeingColor && !unjamAfterIntake && !unjamming && !intakeColor && !intake) {
            sideDepoing = true;
            sideDepoFirst = true;
            sideDepoDebounce = true;
            sideDepoTime = elapsedTime.milliseconds();
            if (whereAmI != PlacePosEnum.wall) {
                outtakeSystem.setVSlidePos(Constants.Outtake.safeFromClimberBar);
            }
        }
        if (!sideDepo) {
            sideDepoDebounce = false;
        }

        if (sideDepoing) {
            intakeSystem.setHSlidePos(0);
            if (whereAmI != PlacePosEnum.wall) {
                outtakeSystem.setVSlidePos(Constants.Outtake.safeFromClimberBar);
            }
            if (elapsedTime.milliseconds() < sideDepoTime + 100 && sideDepoFirst) {
                intakeSystem.setIntakePower(Constants.Intake.transferSpeed);
            }
            if (elapsedTime.milliseconds() > sideDepoTime + 400 && elapsedTime.milliseconds() < sideDepoTime + 500 && sideDepoFirst) {
                intakeSystem.setIntakePower(0);
            }
            if (elapsedTime.milliseconds() > sideDepoTime + 400 && !sideDepo && sideDepoFirst) {
                sideDepoTime = elapsedTime.milliseconds();
                sideDepoFirst = false;
            }
            if (elapsedTime.milliseconds() < sideDepoTime + 100 && !sideDepoFirst) {
                intakeSystem.setDepoServoPos(Constants.Intake.depoDepo);
            }
            if (elapsedTime.milliseconds() > sideDepoTime + 400 && !sideDepoFirst) {
                intakeSystem.setDepoServoPos(Constants.Intake.depoStore);
                sideDepoing = false;
            }
        }

        //Unjamming intake ************************************************************************~
        if (unjamAfterIntake) {
            if (elapsedTime.milliseconds() < unjamAfterIntakeTime + Constants.Intake.unjamTimeMillisTeleop) {
                unJam = true;
            }
            if (elapsedTime.milliseconds() > unjamAfterIntakeTime + Constants.Intake.unjamTimeMillisTeleop) {
                unjamAfterIntake = false;
            }
        }

        if (unJam) {
            if (!intakeing && !intakeingColor) {
                intakeSystem.setIntakeServoPos(Constants.Intake.wristClear);
            }
            unjamming = true;
            unjammingTime = elapsedTime.milliseconds();

            intakeSystem.setIntakePower(Constants.Intake.unjamSpeed);
        }

        if (unjamming && !unJam) {
            if (!(intakeing || intakeingColor)) {
                intakeSystem.setIntakePower(0);
                intakeSystem.setIntakeServoPos(Constants.Intake.wristStore);
            }
            if (intakeSystem.intakeUntil()) {
                intakeSystem.setIntakePower(0);
                unjamming = false;
            }
            if (elapsedTime.milliseconds() > unjammingTime + 600) {
                hasInIntake = false;
                intakeSystem.setIntakePower(0);
                unjamming = false;
            }
        }

        if (elapsedTime.seconds() > 90 && elapsedTime.seconds() < 91) {
            gamepad1.rumble(1000);
            gamepad2.rumble(1000);
        }

        //DEBUG ***********************************************************************************~
        if (debugState || debugAll) {
            telemetry.addLine("STATE");
            telemetry.addData("hasInIntake", hasInIntake);
            telemetry.addData("hasInTray", hasInTray);
            telemetry.addData("hasInOuttake", hasInOuttake);
            telemetry.addData("atStore", atStorePos);
            telemetry.addData("onceTime", onceTime);
            telemetry.addData("where am I", whereAmI);
            telemetry.addLine();
        }
        if (debugStateMachine || debugAll) {
            telemetry.addLine("STATE MACHINE");
            telemetry.addData("doGoToStore", stateMachine.doGoToStore());
            telemetry.addData("doTransfer", stateMachine.doTransfer());
            telemetry.addData("doUnStore", stateMachine.doUnStore());
            telemetry.addData("doHighBasket", stateMachine.doHighBasket());
            telemetry.addData("doLowBasket", stateMachine.doLowBasket());
            telemetry.addData("doHighSpecimen", stateMachine.doHighSpecimen());
            telemetry.addData("doWall", stateMachine.doWall());
            telemetry.addData("hasInIntake", stateMachine.debug()[0]);
            telemetry.addData("transferred", stateMachine.debug()[1]);
            telemetry.addData("atStore", stateMachine.debug()[2]);
            telemetry.addData("onceTime", onceTime);
            telemetry.addLine();
        }
        if (debugPos || debugAll) {
            telemetry.addLine("POSITION");
            telemetry.addData("V Slide Tar", outtakeSystem.getVSlideTargetPos());
            telemetry.addData("V Slide Pos", outtakeSystem.getVSlidePos());
            telemetry.addData("H Slide Tar", intakeSystem.getHSlideTargetPos());
            telemetry.addData("H Slide Pos", intakeSystem.getHSlidePos());
            telemetry.addData("Arm Pos", outtakeSystem.getArmPos());
            telemetry.addLine();
        }
        if (debugClimber || debugAll) {
            telemetry.addLine("CLIMBER");
            telemetry.addData("climberActive", climberActive);
            telemetry.addData("climberStage", climberStage);
            telemetry.addData("climbPause", climbPause);
            telemetry.addData("climbDebounce", climbDebounce);
            telemetry.addData("climberPos", climberPos);

            telemetry.addData("PTO Tar", driveTrain.getPTOTPos());
            telemetry.addData("PTO Pos", driveTrain.getPTOPos());
            telemetry.addData("PTO Err", Math.abs(driveTrain.getPTOPos() - driveTrain.getPTOTPos()));
            telemetry.addData("PTO Pow", Math.max(Math.max(driveTrain.getDrivePowers()[0], driveTrain.getDrivePowers()[1]), Math.max(driveTrain.getDrivePowers()[2], driveTrain.getDrivePowers()[3])));
            telemetry.addData("Climber Tar", climber.getTargetPos());
            telemetry.addData("Climber Var", climberPos);
            telemetry.addData("Climber Pos", climber.getPos());
            telemetry.addData("Climber err", Math.abs(climber.getPos() - climber.getTargetPos()));
            telemetry.addData("Climber once", climbStopPauseOnce);
            telemetry.addData("Climber once", climbStopPause);
            telemetry.addData("Current", Math.max(Math.max(driveTrain.getDriveCurrent()[0], driveTrain.getDriveCurrent()[1]), Math.max(driveTrain.getDriveCurrent()[2], driveTrain.getDriveCurrent()[3])));
            telemetry.addData("climberDistance", climber.getDistance());
            telemetry.addLine();
        }
        if (debugAuto || debugAll) {
            follower.updatePose();
            telemetry.addLine("AUTO");
            telemetry.addData("autoMovement", autoMovement);
            telemetry.addData("autoMovementOnce", autoMovementOnce);
            telemetry.addData("autoState", autoState);
            telemetry.addData("Pose", follower.getPose());
            telemetry.addLine();
        }
        if (debugMisc || debugAll) {
            telemetry.addLine("MISCELLANEOUS");
            telemetry.addData("onceTime", onceTime);
            telemetry.addData("transferTime", transferTime);
            telemetry.addData("droppingBasketTime", droppingBasketTime);
            telemetry.addData("storeTime", storeTime);
            telemetry.addData("walltime", wallTime - elapsedTime.milliseconds());
            telemetry.addData("extendHSlide", extendHSlide);
            telemetry.addData("unjamming", unjamming);
            telemetry.addData("unjamAfterIntake", unjamAfterIntake);
            telemetry.addData("outtake distance", outtakeSystem.getClawDistance());
            telemetry.addData("grabbing off wall", grabbingOffWall);
            telemetry.addData("Loop Times ms", elapsedTime.milliseconds() - lastTime);
            telemetry.addData("power", outtakeSystem.getAmp());
            telemetry.addData("outtakeSystem.readyToTransfer", outtakeSystem.readyToTransfer());
            telemetry.addData("intakeSystem.readyToTransfer",  intakeSystem.readyToTransfer());
            telemetry.addData("wallonce",  wallOnce);
            telemetry.addData("thing",  elapsedTime.milliseconds() > grabbingOffWallTime + 250);
            telemetry.addData("grabbingOffWallTime",  grabbingOffWallTime - elapsedTime.milliseconds());
            telemetry.addData("grabbingOffWall",  grabbingOffWall);
            lastTime = elapsedTime.milliseconds();

            //TODO add more Things here
            telemetry.addLine();
        }
        if (debugAll || debugAuto || debugClimber || debugMisc || debugPos || debugStateMachine || debugState) {
            telemetry.update();
        }

//        telemetry.addData("leftFront", motor1.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("rightFront", motor2.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("rightBack", motor3.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("leftBack", motor4.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("hSlides", motor5.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("vSlides", motor6.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("climber", motor7.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("intakeMotor", motor8.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("max", motor1.getCurrent(CurrentUnit.AMPS) + motor2.getCurrent(CurrentUnit.AMPS) + motor3.getCurrent(CurrentUnit.AMPS) + motor4.getCurrent(CurrentUnit.AMPS) + motor5.getCurrent(CurrentUnit.AMPS) + motor6.getCurrent(CurrentUnit.AMPS) + motor7.getCurrent(CurrentUnit.AMPS) + motor8.getCurrent(CurrentUnit.AMPS));
//
//        telemetry.update();
    }

    public void stop() {
        driveTrain.stopDrive();
    }
}