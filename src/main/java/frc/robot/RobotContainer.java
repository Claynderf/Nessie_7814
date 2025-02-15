package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

//import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.*;
import frc.robot.Commands.*;
import frc.robot.Subsystems.*;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
;



  // The robot's subsystems are defined here.
   private final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem();
   
   
   private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
   private final WristSubsystem m_wrist = new WristSubsystem();
   private final ManipulatorSubsystem m_manipulator = new ManipulatorSubsystem();
  
   SendableChooser<Command> autoChooser = new SendableChooser<>();

  /*The gamepad provided in the KOP shows up like an XBox controller if the mode switch is set to X mode using the
   * switch on the top.*/
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be accessed via the
   * named factory methods in the Command* classes in edu.wpi.first.wpilibj2.command.button (shown
   * below) or via the Trigger constructor for arbitary conditions
   */
  private void configureBindings() {
    // Set the default command for the drivetrain to drive using the joysticks
     m_drivetrain.setDefaultCommand(
         new RunCommand(
             () ->
                 m_drivetrain.driveCartesian(
                     -m_driverController.getLeftY() , -m_driverController.getLeftX() ,-m_driverController.getRightX() ),
             m_drivetrain));
             m_wrist.setDefaultCommand(new DefaultWristCommand(m_wrist));

    m_driverController.a().whileTrue(new ElevatorToCommand(m_elevator, m_wrist, Constants.ElevatorConstants.L3, Constants.WristConstants.L3)).onFalse(new DefaultWristCommand(m_wrist));

		m_driverController.x().whileTrue(new ElevatorToCommand(m_elevator, m_wrist, Constants.ElevatorConstants.L2, Constants.WristConstants.L2)).onFalse(new DefaultWristCommand(m_wrist));

		m_driverController.b().whileTrue(new ElevatorToCommand(m_elevator, m_wrist, Constants.ElevatorConstants.PLACE_ALGAE, Constants.WristConstants.PLACE_ALGAE)).onFalse(new DefaultWristCommand(m_wrist));

		m_driverController.y().whileTrue(new ElevatorToCommand(m_elevator, m_wrist, Constants.ElevatorConstants.HUMAN_PICKUP, Constants.WristConstants.HUMAN_PICKUP)).onFalse(new DefaultWristCommand(m_wrist));
		
		//these buttons are disabled to for outreach so the buttons at the top of the joystick will not control the wrist or elevator
		/*m_controller.button(3).whileTrue(new ElevatorToCommand(m_elevator, m_wrist, Constants.ElevatorConstants.PICKUP_ALGAE_L1, Constants.WristConstants.PICKUP_ALGAE_L1)).onFalse(new DefaultWristCommand(m_wrist));

		m_controller.button(4).whileTrue(new ElevatorToCommand(m_elevator, m_wrist, Constants.ElevatorConstants.PICKUP_ALGAE_L2, Constants.WristConstants.PICKUP_ALGAE_L2)).onFalse(new DefaultWristCommand(m_wrist));
		*/
		m_driverController.rightTrigger().whileTrue(new ElevatorToCommand(m_elevator, m_wrist, Constants.ElevatorConstants.L1, Constants.WristConstants.L1)).onFalse(new DefaultWristCommand(m_wrist));
		
		// coral intake, algae shoot
		m_driverController.leftTrigger().whileTrue(new ManipulatorCommand(m_manipulator, false)).toggleOnFalse(new HoldCommand(m_manipulator, false, false));

		// algae intake, coral shoot
		m_driverController.rightBumper().whileTrue(new ManipulatorCommand(m_manipulator, true)).toggleOnFalse(new HoldCommand(m_manipulator, false, true));

		m_driverController.povUp().onTrue(new StopCommand(m_manipulator));
    
}
}