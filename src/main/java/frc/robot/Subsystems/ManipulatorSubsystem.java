package frc.robot.Subsystems;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ManipulatorSubsystem extends SubsystemBase{
    private SparkMax m_topMotor; // Leader
    private SparkMax m_middleMotor; // Follower
    private SparkMax m_bottomMotor; // Follower

    private final double INTAKE_SPEED = Constants.ManipulatorConstants.INTAKE_SPEED;
    private final double HOLD_SPEED = Constants.ManipulatorConstants.HOLD_SPEED;
    private final double PLACE_SPEED = Constants.ManipulatorConstants.PLACE_SPEED;

    public ManipulatorSubsystem() {
        m_topMotor = new SparkMax(Constants.ManipulatorConstants.TOP_MANIPULATOR_MOTOR_ID, MotorType.kBrushless);
        m_middleMotor = new SparkMax(Constants.ManipulatorConstants.MIDDLE_MANIPULATOR_MOTOR_ID, MotorType.kBrushless);
        m_bottomMotor = new SparkMax(Constants.ManipulatorConstants.BOTTOM_MANIPULATOR_MOTOR_ID, MotorType.kBrushless);

        //config top motor

        SparkMaxConfig topMotorConfig = new SparkMaxConfig();
        topMotorConfig.inverted(false);
        topMotorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);

        // config middle motor

        SparkMaxConfig middleMotorConfig = new SparkMaxConfig();
        middleMotorConfig.inverted(false);
        middleMotorConfig.follow(m_topMotor.getDeviceId(), true);
        middleMotorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);

        // config bottom motor

        SparkMaxConfig bottomMotorConfig = new SparkMaxConfig();
        bottomMotorConfig.inverted(false);
        bottomMotorConfig.follow(m_topMotor.getDeviceId());
        bottomMotorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);

        m_topMotor.configure(topMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        m_middleMotor.configure(middleMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        m_bottomMotor.configure(bottomMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // intake ball, spit out pvc pipe
    public void intake() {
        m_topMotor.set(INTAKE_SPEED);
    }
    
    // hold algae, shoot coral
    public void holdAlgae() {
        m_topMotor.set(HOLD_SPEED);
    }
    
    // hold coral, shoot algae
    public void holdCoral() {
        m_topMotor.set(-HOLD_SPEED);
    }

    // intake pvc pipe, shoot out ball
    public void placeGamepiece() {
        m_topMotor.set(PLACE_SPEED);
    }

    // stop motor
    public void stop() {
        m_topMotor.stopMotor();
    }
}
