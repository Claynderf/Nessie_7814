package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase {
    private SparkMax m_wristMotor;
    private PIDController m_wristPidController;
    private ArmFeedforward m_wristFeedForward;
    private DigitalInput m_limitSwitch;
    private boolean inTolerance = false;
    public boolean disableWrist = false; //to disable the wrist for outreach, set this to true
    public double w_kP_tune = Constants.WristConstants.WRIST_kP;
    public double w_PID_Tolerance_tune= 0.1;

    public WristSubsystem() {
        m_wristMotor = new SparkMax(Constants.WristConstants.WRIST_MOTOR_ID, MotorType.kBrushless);

        // config motor
        SparkMaxConfig wristMotorConfig = new SparkMaxConfig();
        wristMotorConfig.inverted(false);
        wristMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        m_wristMotor.configure(wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // Set up PID controller
        m_wristPidController = new PIDController(Constants.WristConstants.WRIST_kP,
        Constants.WristConstants.WRIST_kI, Constants.WristConstants.WRIST_kD);
        m_wristPidController.setTolerance(Constants.WristConstants.WRIST_PID_TOLERANCE);

        // Set up feed forward 
        m_wristFeedForward = new ArmFeedforward(Constants.WristConstants.WRIST_kS, Constants.WristConstants.WRIST_kG, Constants.WristConstants.WRIST_kV);

        m_limitSwitch = new DigitalInput(Constants.WristConstants.limitSwitchPort);

        //SmartDashboard.putBoolean("Disable Wrist", disableWrist);
        // SmartDashboard.putNumber("Wrist feedforward", 9999);
        // SmartDashboard.putNumber("Wrist speed", 9999);
        // SmartDashboard.putNumber("Wrist pidOutput", 9999);
    }

    
    public void periodic() {
        /* add if need to tune pid again 
        w_kP_tune = SmartDashboard.getNumber("Wrist kP", w_kP_tune);
        wristPidController.setP(w_kP_tune);
        w_PID_Tolerance_tune = SmartDashboard.getNumber("Wrist PID Tolerance", w_PID_Tolerance_tune);
        wristPidController.setTolerance(w_PID_Tolerance_tune);
        */
    }
    

    // zero the wrist encoder
    public void zeroWrist() {
        if ( disableWrist == false ){ //safety feature for outreach
            m_wristMotor.getEncoder().setPosition(0);
        }
    }

    // do NOT setPosition outisde of limit, there is nothing stopping it for going out of bounds
    public void setPosition(double position) {

        inTolerance = m_wristPidController.atSetpoint();

        // set the goal for it to -goalPosition, the encoder pose is - and the speed wants a + value
        m_wristPidController.setSetpoint(-position);

        // calculate the pid using feed forward and set the motor to maintain position
        double pidOutput = m_wristPidController.calculate(m_wristMotor.getEncoder().getPosition(), position);

        double feedForward = m_wristFeedForward.calculate(m_wristMotor.getEncoder().getPosition(), m_wristMotor.getEncoder().getVelocity());

        double speed = pidOutput + feedForward;

        // SmartDashboard.putNumber("Wrist pidOutput", pidOutput);
        // SmartDashboard.putNumber("Wrist feedforward", feedForward);
        // SmartDashboard.putNumber("Wrist speed", speed);
        // SmartDashboard.putNumber("Wrist Position", m_wristMotor.getEncoder().getPosition());

        // ensure @param speed is within -1 to 1
        speed = (speed > 1) ? 1 : speed;
        speed = (speed < -1) ? -1 : speed;

        // if the wrist is at its backmost pose, reset zero to compensate for encoder drifting through match
        if (m_limitSwitch.get()) {
            zeroWrist();
        }

        // set the motor speed
        if(disableWrist == false){ //safety feature for outreah
            m_wristMotor.set(speed);
        } else {
            m_wristMotor.set(0);
        }

    }

    // get encoder pose
    public double getPosition() {
        return m_wristMotor.getEncoder().getPosition();
    }

    public boolean getInTolerance() {
        return inTolerance;
    }

    public void stopWrist() {
        m_wristMotor.stopMotor();
    }

    // manual wrist control
    public void setWristSpeed(double speed) {
        // SmartDashboard.putNumber("Wrist Encoder Position", m_wristMotor.getEncoder().getPosition());
        // SmartDashboard.putNumber("Wrist Speed", speed);
        
        if (m_limitSwitch.get()) 
        {
            zeroWrist();
        }

        /*if (wristMotor.getEncoder().getPosition() >= Constants.WristConstants.WRIST_LIMIT_BOTTOM) {
            speed = Math.min(speed, 0);
        }

        if (wristMotor.getEncoder().getPosition() <= Constants.WristConstants.WRIST_LIMIT_TOP) {
            speed = Math.max(speed, 0);
        }*/

        // if ((m_limitSwitch.get() && speed > 0)
        //         || wristMotor.getEncoder().getPosition() > Constants.WristConstants.WRIST_LIMIT_BOTTOM) {
        //     speed = 0;
        // }

        // if ((wristMotor.getEncoder().getPosition() < Constants.WristConstants.WRIST_LIMIT_TOP) && speed > 0) {
        //     speed = 0;
        // }

        // ensure @param speed is within -1 to 1
        speed = (speed > 1) ? 1 : speed;
        speed = (speed < -1) ? -1 : speed;
        if (disableWrist == false){ //safety feature for outreach
            m_wristMotor.set(speed);
        } else {
            m_wristMotor.set(speed);
        }

    }
}
