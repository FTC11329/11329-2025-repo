package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PressHold {

    public boolean isOn = false;
    public boolean isPressed = false;

    public boolean startPress = false;

    public ElapsedTime time = new ElapsedTime();

    public void checkStatus(boolean pressed){
        if (startPress){
            startPress = false;
        }
        if (pressed && !isPressed){
            if (!isOn){
                startPress = true;
                isOn = true;
                time.reset();
            }else {
                isOn = false;
            }
        }
        isPressed = pressed;
    }
}
