package org.firstinspires.ftc.teamcode.subsystems.Arm.ArmCommands;

import org.firstinspires.ftc.teamcode.subsystems.Arm.PivotSubsystem;
import org.firstinspires.ftc.teamcode.utils.BT.BTCommand;

public class SetPickupAngle extends BTCommand{

    PivotSubsystem m_pivot;
    public SetPickupAngle(PivotSubsystem subsystem){
        m_pivot = subsystem;
    }

    public void execute(){
//        m_pivot.set(m_pivot.m_pivotPID.setGoal(PICKUP.Angle));
    }

}