package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ManipulatorSubsystem;

public class ManipulatorCommand extends Command{
    private ManipulatorSubsystem manipulatorSubsystem;
    private boolean isIntake = false;

    public ManipulatorCommand(ManipulatorSubsystem manipulatorSubsystem, boolean isIntake)
    {
        this.manipulatorSubsystem = manipulatorSubsystem;
        this.isIntake = isIntake;

        addRequirements(manipulatorSubsystem);
    }

    @Override
    public void execute()
    {
        if (isIntake)
        {
            manipulatorSubsystem.intake();
        }
        else
        {
            manipulatorSubsystem.placeGamepiece();
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        manipulatorSubsystem.stop();
    }
}
