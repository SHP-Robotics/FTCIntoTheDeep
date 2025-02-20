package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem.ColorState.GREEN;
import static org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem.ColorState.OFF;

import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HorizSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RotateSubsystem;

public class SubToDriveCommand extends Command {
    RotateSubsystem rotate;
    ClawSubsystem claw;
    PivotSubsystem pivot;
    HorizSubsystem horiz;

    private double startTime, endTime;

    public SubToDriveCommand(RotateSubsystem rotate, ClawSubsystem claw, PivotSubsystem pivot, HorizSubsystem horiz) {
        // You MUST call the parent class constructor and pass through any subsystems you use
        super(rotate, claw, pivot, horiz);
        this.rotate = rotate;
        this.claw = claw;
        this.pivot = pivot;
        this.horiz = horiz;
        endTime = 0.75;
    }


    // Called once when the command is initially schedule

    public void init() {
        startTime = Clock.now();
    }

    // Called repeatedly until isFinished() returns true
    @Override
    public void execute() {
        pivot.setState(PivotSubsystem.State.INTAKE);

        if(Clock.hasElapsed(startTime, 0.25))
            claw.close();
    }

    // Called once after isFinished() returns true
    @Override
    public void end() {
        if (!claw.isBlockInClaw()) {
            CommandScheduler.getInstance().scheduleCommand(
                    new DriveToSubCommand(rotate, claw, pivot, horiz)
            );
        }
        if (claw.isBlockInClaw()) {
            claw.setColor(GREEN);
            CommandScheduler.getInstance().scheduleCommand(
                    new RunCommand(()->{
                        rotate.setState(RotateSubsystem.State.NEUTRAL);
                        pivot.setState(PivotSubsystem.State.SUB_TO_DRIVING);
                    })
                            .then(new WaitCommand(0.05))
                            .then(new RunCommand(() -> {
                                horiz.setState(HorizSubsystem.State.DRIVING);
                            }))
                            .then(new WaitCommand(0.5))
                            .then(new RunCommand(() -> {
                                pivot.setState(PivotSubsystem.State.DRIVING);
                                claw.setColor(OFF);
                            }))
            );
        }
    }

    // Specifies whether or not the command has finished
    // Returning true causes execute() to be called once
    @Override
    public boolean isFinished() {
        return Clock.hasElapsed(startTime, endTime);
    }
}
