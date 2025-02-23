package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.devices.GamepadInterface;
import com.shprobotics.pestocore.devices.GamepadKey;

import org.firstinspires.ftc.teamcode.shplib.TestBaseBot;

@Disabled
@TeleOp
public class ClawTeleOp extends TestBaseBot {
    GamepadInterface gamepadInterface1;

    @Override
    public void init(){
        super.init();

        gamepadInterface1 = new GamepadInterface(gamepad1);
    }

    @Override
    public void loop(){
        super.loop();

        gamepadInterface1.update();

        if(gamepadInterface1.isKeyDown(GamepadKey.DPAD_LEFT)) claw.increment();
        if(gamepadInterface1.isKeyDown(GamepadKey.DPAD_RIGHT)) claw.decrement();
    }
}