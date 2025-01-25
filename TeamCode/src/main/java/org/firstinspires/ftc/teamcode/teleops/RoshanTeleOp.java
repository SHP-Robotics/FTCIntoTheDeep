package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.devices.GamepadInterface;
import com.shprobotics.pestocore.devices.GamepadKey;

import org.firstinspires.ftc.teamcode.shplib.BaseRobot;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.Trigger;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSubsystem;

@TeleOp(name = "RoshanTeleOp", group = "TeleOp")
public class RoshanTeleOp extends BaseRobot {
    private double driveBias; // Adjust driving sensitivity
    private GamepadInterface gamepadInterface1; // Wrapper for custom input handling

    @Override
    public void init() {
        super.init();

        // Set default drive command for mecanum driving
        drive.setDefaultCommand(
                new RunCommand(() -> drive.mecanum(
                        -driveBias * gamepad1.left_stick_y, // Forward/backward
                        driveBias * gamepad1.left_stick_x,  // Strafing
                        driveBias * gamepad1.right_stick_x  // Turning
                ))
        );



        // Initialize gamepad interface for enhanced input handling
        gamepadInterface1 = new GamepadInterface(gamepad1);
    }

    @Override
    public void start() {
        super.start();
        // Retrieve the default drive bias from the vertical subsystem
        driveBias = vertical.getDriveBias();
    }

    @Override
    public void loop() {
        super.loop();

        // Update the drive bias dynamically
        driveBias = vertical.getDriveBias();

        // Update gamepad interface to manage input states and debounce
        gamepadInterface1.update();

        // Update the drive system for accurate tracking and control
        drive.update(gamepad1);

        // === Vertical Slide Controls ===
        // Right bumper: Cycle between HIGHBUCKET and LOWBUCKET states
        if (gamepad1.right_bumper) {
            vertical.cycleStates(true);
        }

        // Left bumper: Set the slide to the BOTTOM state
        if (gamepad1.left_bumper) {
            vertical.setState(VerticalSubsystem.State.BOTTOM);
        }

        // B button: Return the slide to the intake position
        if (gamepad1.b) {
            vertical.returnToIntake();
        }

        // Manual slide adjustments with D-Pad
        if (gamepad1.dpad_up) {
            vertical.incrementSlide(); // Increment slide position
        }
        if (gamepad1.dpad_down) {
            vertical.decrementSlide(); // Decrement slide position
        }
//        if (gamepad1.dpad_left) {
//            vertical.incrementWormGear(); // Increment worm gear position
//        }
//
//        if (gamepad1.dpad_right) {
//            vertical.decrementWormGear(); // Decrement worm gear position
//        }

//        gamepad1.dpad_right ? vertical.runIntake(1) : vertical.stopIntake();

        if (gamepad1.dpad_right){
            vertical.runIntake(1);
        } else {
            vertical.stopIntake();
        }

        if (gamepad1.dpad_left){
            vertical.runIntake(-1);
        } else {
            vertical.stopIntake();
        }

//        vertical.intake.setPower((gamepad1.dpad_left ? 1 : 0) - (gamepad1.dpad_right ? 1 : 0));

        vertical.wormGear.setPower(gamepad1.right_trigger - (gamepad1.left_trigger * 0.5));

        // A button: Emergency reset to zero position
        if (gamepad1.a) {
            vertical.resetZeroPosition(); // Reset encoders and position
        }

        // Telemetry to monitor key data during TeleOp
        telemetry.addData("Drive Bias:", driveBias);
        telemetry.addData("Vertical Slide State:", vertical.getState());
        telemetry.addData("Slide Position:", vertical.getSlidePosition());
        telemetry.update();
    }
}