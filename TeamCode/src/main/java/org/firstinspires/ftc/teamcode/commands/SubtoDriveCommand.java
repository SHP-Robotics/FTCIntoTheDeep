package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HorizSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RotateSubsystem;

public class SubtoDriveCommand extends Command {
    RotateSubsystem rotate;
    ClawSubsystem claw;
    PivotSubsystem pivot;
    HorizSubsystem horiz;
    private double startTime;
    private double endTime;

    public SubtoDriveCommand(RotateSubsystem rotate, ClawSubsystem claw, PivotSubsystem pivot, HorizSubsystem horiz) {
        // You MUST call the parent class constructor and pass through any subsystems you use
        super(rotate, claw, pivot, horiz);
        this.rotate = rotate;
        this.claw = claw;
        this.pivot = pivot;
        this.horiz = horiz;
        endTime = 0.25;
    }


    // Called once when the command is initially schedule

    public void init() {
        startTime = Clock.now();
    }

    // Called repeatedly until isFinished() returns true
    @Override
    public void execute() {
        pivot.setState(PivotSubsystem.State.INTAKE);
//        claw.close();
    }

    // Called once after isFinished() returns true
    @Override
    public void end() {
        claw.close();
//        new WaitCommand(0.5);
//        rotate.setState(RotateSubsystem.State.NEUTRAL);
//        pivot.setState(PivotSubsystem.State.DRIVING);
//        horiz.setState(HorizSubsystem.State.DRIVING);



    }

    // Specifies whether or not the command has finished
    // Returning true causes execute() to be called once
    @Override
    public boolean isFinished() {
        return Clock.hasElapsed(startTime, endTime);
    }
}
