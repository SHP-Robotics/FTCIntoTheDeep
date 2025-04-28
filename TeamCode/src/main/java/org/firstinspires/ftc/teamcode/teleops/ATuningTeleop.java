package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.devices.GamepadInterface;

import org.firstinspires.ftc.teamcode.shplib.BaseRobot;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;

@Disabled
@TeleOp
public class ATuningTeleop extends BaseRobot {
    GamepadInterface gamepadInterface2;

    @Override
    public void init(){
        super.init();
        drive.setDefaultCommand(
                new RunCommand(
                        () -> drive.mecanum(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x)
                )
        );

        gamepadInterface2 = new GamepadInterface(gamepad2);
    }

    @Override
    public void loop(){
        super.loop();
        drive.update(gamepad2);

//        new Trigger(gamepad1.dpad_up, new RunCommand(()-> pivot.incrementElbowUp()));
//        new Trigger(gamepad1.dpad_down, new RunCommand(()-> pivot.decrementElbowDown()));

        
    }
}
