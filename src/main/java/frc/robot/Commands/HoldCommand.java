package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ManipulatorSubsystem;

public class HoldCommand extends Command{
    
    private ManipulatorSubsystem manipulatorSubsystem;
    private boolean m_stop;
    private boolean m_algae;

    public HoldCommand(ManipulatorSubsystem manipulatorSubsystem, boolean stop, boolean algae)
    {
        this.manipulatorSubsystem = manipulatorSubsystem;
        this.m_stop = stop;
        this.m_algae = algae;
        addRequirements(manipulatorSubsystem);
    }

    @Override
    public void execute()
    {
        if (m_stop) { end(false); return; }

        if (m_algae)
        {
            manipulatorSubsystem.holdAlgae();
        }
        else
        {
            manipulatorSubsystem.holdCoral();
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        manipulatorSubsystem.stop();
    }
}
