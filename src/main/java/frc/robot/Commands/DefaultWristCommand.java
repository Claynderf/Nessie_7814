package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.WristSubsystem;

public class DefaultWristCommand extends Command{
    
    private WristSubsystem wristSubsystem;

    public DefaultWristCommand(WristSubsystem wristSubsystem)
    {
        this.wristSubsystem = wristSubsystem;

        addRequirements(wristSubsystem);
    }

    @Override
    public void execute()
    {
        wristSubsystem.setPosition(Constants.WristConstants.WRIST_PARK_ANGLE);
    }
}
