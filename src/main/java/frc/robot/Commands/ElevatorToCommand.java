package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.WristSubsystem;

public class ElevatorToCommand extends Command
{
    
    private ElevatorSubsystem elevatorSubsystem;
    private WristSubsystem wristSubsystem;
    private double elevatorHeight;
    private double wristAngle;

    public ElevatorToCommand(ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem, double elevatorHeight, double wristAngle)
    {
        this.elevatorSubsystem = elevatorSubsystem;
        this.wristSubsystem = wristSubsystem;
        this.elevatorHeight = elevatorHeight;
        this.wristAngle = wristAngle;

        addRequirements(elevatorSubsystem, wristSubsystem);
    }

    @Override
    public void initialize()
    {
        wristSubsystem.setPosition(wristAngle);
        elevatorSubsystem.setPostion(elevatorHeight);
    }

    @Override
    public void execute()
    {
        wristSubsystem.setPosition(wristAngle);
        elevatorSubsystem.setPostion(elevatorHeight);
    }

    @Override
    public void end(boolean interrupted)
    {
        wristSubsystem.setPosition(Constants.WristConstants.WRIST_PARK_ANGLE);
        elevatorSubsystem.setPostion(Constants.ElevatorConstants.ELEVATOR_PARK_HEIGHT);
    }
}
