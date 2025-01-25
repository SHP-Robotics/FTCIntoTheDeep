package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.devices.GamepadInterface;
import com.shprobotics.pestocore.devices.GamepadKey;
import org.firstinspires.ftc.teamcode.shplib.BaseRobot;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.Trigger;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSubsystem;

@TeleOp(name = "AOfficialTeleOp", group = "TeleOp")
public class AOfficialTeleOp extends BaseRobot {
    private double driveBias; // Used to adjust driving sensitivity
    private GamepadInterface gamepadInterface1; // Gamepad wrapper for debouncing and custom actions

    @Override
    public void init() {
        super.init();

        // Set default driving command for mecanum drive
        drive.setDefaultCommand(
                new RunCommand(() -> drive.mecanum(
                        -driveBias * gamepad1.left_stick_y, // Forward/backward
                        driveBias * gamepad1.left_stick_x,  // Strafing
                        driveBias * gamepad1.right_stick_x  // Turning
                ))
        );

        // Initialize the gamepad interface for better input handling
        gamepadInterface1 = new GamepadInterface(gamepad1);
    }

    @Override
    public void start() {
        super.start();
        // Initialize drive bias from the vertical subsystem
        driveBias = vertical.getDriveBias();
    }

    @Override
    public void loop() {
        super.loop();

        // Update the drive bias dynamically if required
        driveBias = vertical.getDriveBias();

        // Update gamepad interface for debounce and state management
        gamepadInterface1.update();

        // Update the drive system (important for custom implementations)
        drive.update(gamepad1);

        // === Vertical Slide Controls ===
        // Right bumper: Cycle between HIGHBUCKET and LOWBUCKET states
        new Trigger(gamepad1.right_bumper, new RunCommand(() -> {
            vertical.cycleStates(true);
        }));

        // Left bumper: Set the slide to the BOTTOM state
        new Trigger(gamepad1.left_bumper, new RunCommand(() -> {
//            vertical.setDepositState(VerticalSubsystem.State.BOTTOM);
        }));

        // B button: Return the slide to the intake position
        new Trigger(gamepad1.b, new RunCommand(() -> {
            vertical.returnToIntake();
        }));

        // Manual slide adjustments with D-Pad
        if (gamepad1.dpad_up) {
            vertical.incrementSlide(); // Move the slide up incrementally
        }
        if (gamepad1.dpad_down) {
            vertical.decrementSlide(); // Move the slide down incrementally
        }
        if (gamepad1.dpad_left) {

        }

        // A button: Emergency reset to zero position
        if (gamepad1.a) {
            vertical.resetZeroPosition(); // Reset encoders and position
        }
    }
}