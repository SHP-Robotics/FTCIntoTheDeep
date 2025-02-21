package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.devices.GamepadInterface;
import com.shprobotics.pestocore.devices.GamepadKey;

import org.firstinspires.ftc.teamcode.shplib.TestBaseBot;


@TeleOp
public class ClawTeleOp extends TestBaseBot {
    private double driveBias;
    GamepadInterface gamepadInterface1, gamepadInterface2;

    @Override
    public void init(){
        super.init();

        gamepadInterface1 = new GamepadInterface(gamepad1);
        gamepadInterface2 = new GamepadInterface(gamepad2);
//        vision.limelight.start();

    }
    @Override
    public void start(){
        super.start();

    }

    @Override
    public void loop(){
        super.loop();

        gamepadInterface1.update();
        gamepadInterface2.update();

        if(gamepadInterface1.isKeyDown(GamepadKey.DPAD_LEFT))
            claw.open();
        if(gamepadInterface1.isKeyDown(GamepadKey.DPAD_RIGHT))
            claw.close();


    }

}