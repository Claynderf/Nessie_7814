package frc.robot.Subsystems;

import static frc.robot.Constants.DrivetrainConstants.kCurrentLimit;
import static frc.robot.Constants.DrivetrainConstants.kLeftFrontID;
import static frc.robot.Constants.DrivetrainConstants.kLeftRearID;
import static frc.robot.Constants.DrivetrainConstants.kRightFrontID;
import static frc.robot.Constants.DrivetrainConstants.kRightRearID;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;


import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DrivetrainSubsystem extends SubsystemBase {


/* This class declares the subsystem for the robot drivetrain if controllers are connected via CAN. Make sure to go to
 * RobotContainer and uncomment the line declaring this subsystem and comment the line for PWMDrivetrain.
 *
 * The subsystem contains the objects for the hardware contained in the mechanism and handles low level logic
 * for control. Subsystems are a mechanism that, when used in conjuction with command "Requirements", ensure
 * that hardware is only being used by 1 command at a time.
 */

  /*Class member variables. These variables represent things the class needs to keep track of and use between
  different method calls. */
    SparkMax leftFront = new SparkMax(kLeftFrontID, MotorType.kBrushless);
    SparkMax leftRear = new SparkMax(kLeftRearID, MotorType.kBrushless);
    SparkMax rightFront = new SparkMax(kRightFrontID, MotorType.kBrushless);
    SparkMax rightRear = new SparkMax(kRightRearID, MotorType.kBrushless);
    
     
    //.smartCurrentLimit(50)

    MecanumDrive m_drivetrain;

  /*Constructor. This method is called when an instance of the class is created. This should generally be used to set up
   * member variables and perform any configuration or set up necessary on hardware.
   */
  public DrivetrainSubsystem() {
    /*Sets current limits for the drivetrain motors. This helps reduce the likelihood of wheel spin, reduces motor heating
     *at stall (Drivetrain pushing against something) and helps maintain battery voltage under heavy demand */
    //leftFront.SetSmartCurrentLimit(kCurrentLimit);
    //leftRear.setSmartCurrentLimit(kCurrentLimit);
    //rightFront.setSmartCurrentLimit(kCurrentLimit);
    //rightRear.setSmartCurrentLimit(kCurrentLimit);
    /*public void periodic(){
smartdashboard.putnumber("left front wheel", leftFront.getEncoder().getPosition());
smartdashboard.putnumber("left rear wheel", leftRear.getEncoder().getPosition());
smartdashboard.putnumber("right front wheel", rightFront.getEncoder().getPosition());
smartdashboard.putnumber("right rear wheel", rightRear.getEncoder().getPosition());
}*/
    //SmartDashboard.putNumber("left front wheel", leftFront.getEncoder().getPosition());
    //SmartDashboard.putNumber("left rear wheel", leftRear.getEncoder().getPosition());
    //SmartDashboard.putNumber("right front wheel", rightFront.getEncoder().getPosition());
    //SmartDashboard.putNumber("right rear wheel", rightRear.getEncoder().getPosition());
    // Put the front motors into the differential drive object. This will control all 4 motors with
    // the rears set to follow the fronts
    m_drivetrain = new MecanumDrive(leftFront, leftRear, rightFront, rightRear);
  }

  /*Method to control the drivetrain using arcade drive. Arcade drive takes a speed in the X (forward/back) direction
   * and a rotation about the Z (turning the robot about it's center) and uses these to control the drivetrain motors */
  public void driveCartesian(double xSpeed, double ySpeed, double rotation) {
    m_drivetrain.driveCartesian(xSpeed, ySpeed, rotation);
  }
public double[] getpositions() {
return new double[] {leftFront.getEncoder().getPosition(), leftRear.getEncoder().getPosition(),
   rightFront.getEncoder().getPosition(), rightRear.getEncoder().getPosition()};

}


  @Override
  public void periodic() {
    /*This method will be called once per scheduler run. It can be used for running tasks we know we want to update each
     * loop such as processing sensor data. Our drivetrain is simple so we don't have anything to put here */
  }
}

