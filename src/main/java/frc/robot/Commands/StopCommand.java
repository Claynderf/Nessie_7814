package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ManipulatorSubsystem;

public class StopCommand extends Command
{
    private ManipulatorSubsystem m_subsystem;

    public StopCommand(ManipulatorSubsystem subsystem)
    {
        this.m_subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute()
    {
        m_subsystem.stop();
    }
}
