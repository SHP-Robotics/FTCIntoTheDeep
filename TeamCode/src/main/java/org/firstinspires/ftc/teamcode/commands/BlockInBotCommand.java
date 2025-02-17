package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HorizSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RotateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSubsystem;

public class BlockInBotCommand extends Command {
    HorizSubsystem horiz;
    double startTime, endTime;

    public BlockInBotCommand(HorizSubsystem horiz) {
        // You MUST call the parent class constructor and pass through any subsystems you use
        super(horiz);
        this.horiz = horiz;
        endTime = 0.5;
    }


    // Called once when the command is initially schedule
    public void init() {
        super.init();
        startTime = Clock.now();
    }

    // Called repeatedly until isFinished() returns true
    @Override
    public void execute() {
        horiz.setState(HorizSubsystem.State.BLOCK_IN_BOT);
    }

    // Called once after isFinished() returns true
    @Override
    public void end() {
        horiz.setState(HorizSubsystem.State.DRIVING);
    }

    // Specifies whether or not the command has finished
    // Returning true causes execute() to be called once
    @Override
    public boolean isFinished() {
        return Clock.hasElapsed(startTime, endTime);
    }
}
