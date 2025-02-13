package frc.robot;

public class Constants {
    //constants for the Drivetrain
    public static class DrivetrainConstants {
        // PWM ports/CAN IDs for motor controllers
        public static final int kLeftRearID = 1;
        public static final int kLeftFrontID = 2;
        public static final int kRightRearID = 3;
        public static final int kRightFrontID = 4;
    
        // Current limit for drivetrain motors
        public static final int kCurrentLimit = 60;
      }
      public static class OperatorConstants {
        // Port numbers for driver and operator gamepads. These correspond with the numbers on the USB
        // tab of the DriverStation
        public static final int kDriverControllerPort = 0;
      }
}
