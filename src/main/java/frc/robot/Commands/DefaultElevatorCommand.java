package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.ElevatorSubsystem;

public class DefaultElevatorCommand extends Command{
    
    private ElevatorSubsystem elevatorSubsystem;

    public DefaultElevatorCommand(ElevatorSubsystem elevatorSubsystem)
    {
        this.elevatorSubsystem = elevatorSubsystem;

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute()
    {
        if (!elevatorSubsystem.getInTolerance())
        {
            elevatorSubsystem.setPostion(Constants.ElevatorConstants.ELEVATOR_PARK_HEIGHT);
        }
        else
        {
            elevatorSubsystem.stopElevator();
        }
    }
}
