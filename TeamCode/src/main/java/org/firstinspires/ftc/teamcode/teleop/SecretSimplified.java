package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;
import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

@TeleOp(name = "New Secret Simplified", group = "Comp mode")
public class SecretSimplified extends OpMode {
    @Override
    public void init() {
        clearLog();
    }

    public void recordPosition(double time, Pose pos) {
        File dir = AppUtil.getInstance().getSettingsFile("TeamCodeLogs").getParentFile();
        File file = new File(dir, "movement.json");
        JSONArray log;
        telemetry.addData("Recording, time: ", time);
        telemetry.addData("Recording, time: ", dir.toString());
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

            FileWriter writer = new FileWriter(file);
            telemetry.addData("Text: ", log.toString(2));
            writer.write(log.toString(2));
            writer.close();

            telemetry.addData("Drive log written", true);
        } catch (Exception e) {
            telemetry.addData("Drive log error", e.getMessage());
        }
    }

    public void clearLog() {
        File dir = AppUtil.getInstance().getSettingsFile("TeamCodeLogs").getParentFile();
        if (!dir.exists()) dir.mkdirs();  // Ensure the directory exists

        File file = new File(dir, "movement.json");

        telemetry.addData("Clearing log file at: ", file.getAbsolutePath());
        telemetry.addData("Does folder exist: ", dir.exists());

        try (FileWriter writer = new FileWriter(file)) {
            writer.write("[]");  // Reset the file with an empty JSON array
            writer.close();
            telemetry.addData("Log cleared successfully", true);
        } catch (IOException e) {
            telemetry.addData("Error clearing log", e.getMessage());
        }
    }

    @Override
    public void loop() {

    }

    @Override
    public void start() {
        recordPosition(5, new Pose(1, 2));

    }

    @Override
    public void stop() {

    }
}
