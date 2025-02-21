package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.devices.GamepadKey;

@TeleOp(name = "Blue Tele Op")
class BlueTeleop extends BaseTeleOp {
    @Override
    public void init() {
        super.init();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        super.loop();

        if(gamepadInterface1.isKeyDown(GamepadKey.TOUCHPAD)){
            if(detectSample.cycleColorsBlue()){
                r = 255;
                g = 255;
                b = 0;
            }
            else{
                r = 0;
                g = 0;
                b = 255;
            }
        }
    }
}