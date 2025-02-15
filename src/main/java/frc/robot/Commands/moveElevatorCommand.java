package frc.robot.Commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.ElevatorSubsystem;

public class moveElevatorCommand extends Command
{
    private ElevatorSubsystem elevatorSubsystem;
    private DoubleSupplier m_speed;

    public moveElevatorCommand(DoubleSupplier speed, ElevatorSubsystem elevatorSubsystem)
    {
        this.elevatorSubsystem = elevatorSubsystem;
        this.m_speed = speed;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute()
    {
        elevatorSubsystem.moveElevator(m_speed.getAsDouble() * Constants.ElevatorConstants.ELEVATOR_SPEED_MODIFIER);
    }
}
