package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

public class FlashCommand extends Command {
    ClawSubsystem claw;
    ClawSubsystem.ColorState color;

    double startTime, endTime;

    public FlashCommand(ClawSubsystem claw, ClawSubsystem.ColorState color) {
        // You MUST call the parent class constructor and pass through any subsystems you use
        super(claw);
        this.color = color;
        endTime = 1;
    }


    // Called once when the command is initially schedule
    public void init() {
        super.init();
        startTime = Clock.now();
    }

    // Called repeatedly until isFinished() returns true
    @Override
    public void execute() {
        claw.setColor(color);
    }

    // Called once after isFinished() returns true
    @Override
    public void end() {
        claw.setColor(ClawSubsystem.ColorState.OFF);
    }

    // Specifies whether or not the command has finished
    // Returning true causes execute() to be called once
    @Override
    public boolean isFinished() {
        return Clock.hasElapsed(startTime, endTime);
    }
}
