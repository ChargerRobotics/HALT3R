package frc.robot;

import javax.lang.model.util.ElementScanner6;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot 
{
  /* DRIVETRAIN */
  /********************************************************************************************************************************/
  private PWMSparkMax frontLeftMotor = new PWMSparkMax(0);
  private PWMSparkMax frontRightMotor = new PWMSparkMax(1);
  private PWMSparkMax backLeftMotor = new PWMSparkMax(2);
  private PWMSparkMax backRightMotor = new PWMSparkMax(3);

  private MotorControllerGroup leftDrive = new MotorControllerGroup(frontLeftMotor, backLeftMotor);
  private MotorControllerGroup rightDrive = new MotorControllerGroup(frontRightMotor, backRightMotor);

  private DifferentialDrive drivetrain;
  /********************************************************************************************************************************/

  /* STORAGE AND INTAKE */
  /********************************************************************************************************************************/
  private CANSparkMax intakeMotor = new CANSparkMax(8, MotorType.kBrushed);
  private VictorSPX storageMotor_master = new VictorSPX(10);
  private VictorSPX storageMotor_slave = new VictorSPX(11);
  /********************************************************************************************************************************/

  /* SHOOTER */
  /********************************************************************************************************************************/
  private CANSparkMax shooterMotor_master = new CANSparkMax(20, MotorType.kBrushless);
  private CANSparkMax shooterMotor_slave = new CANSparkMax(21, MotorType.kBrushless);
  /********************************************************************************************************************************/

  /* CLIMB */
  /********************************************************************************************************************************/
  private TalonFX staticClimbMotor_master = new TalonFX(30);
  private TalonFX staticClimbMotor_slave = new TalonFX(31);
  private TalonFXConfiguration config = new TalonFXConfiguration();
  
  private CANSparkMax dynamicClimbMotor_master = new CANSparkMax(40, MotorType.kBrushless);
  private CANSparkMax dynamicClimbMotor_slave = new CANSparkMax(41, MotorType.kBrushless);
  /********************************************************************************************************************************/

  /* JOYSTICKS AND PNEUMATICS */
  /********************************************************************************************************************************/
  private Joystick leftController = new Joystick(0);
  private Joystick rightController = new Joystick(1);

  private Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  private DoubleSolenoid leftIntakePiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  private DoubleSolenoid rightIntakePiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
  /********************************************************************************************************************************/
  
  @Override
  public void robotInit() 
  {
    // BIND SLAVES TO MASTERS(INVERTED IF NEEDED)
    storageMotor_slave.follow(storageMotor_master);
    storageMotor_slave.setInverted(InvertType.FollowMaster);
    shooterMotor_slave.follow(shooterMotor_master, true);
    staticClimbMotor_slave.follow(staticClimbMotor_master, FollowerType.PercentOutput);
    staticClimbMotor_slave.setInverted(InvertType.FollowMaster);
    dynamicClimbMotor_slave.follow(dynamicClimbMotor_master, true);
    
    // SETUP DRIVETRAIN
    rightDrive.setInverted(true);
    drivetrain = new DifferentialDrive(leftDrive, rightDrive);

    // CONFIGURE STATIC CLIMB
    config.supplyCurrLimit.enable = true;
    config.supplyCurrLimit.triggerThresholdCurrent = 30;
    config.supplyCurrLimit.currentLimit = 28;
    config.supplyCurrLimit.triggerThresholdTime = 1.5;
    config.openloopRamp = 1;
    
    staticClimbMotor_master.configAllSettings(config);
    staticClimbMotor_slave.configAllSettings(config);
  }

  @Override
  public void robotPeriodic() 
  {
  /* SMARTDASHBOARD */
  /********************************************************************************************************************************/
  SmartDashboard.putNumber("Pressure", compressor.getPressure());
  /********************************************************************************************************************************/

  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() 
  {
    // START COMPRESSOR
    compressor.enableDigital();
  }

  @Override
  public void teleopPeriodic() 
  {
    /* DRIVETRAIN */
  /********************************************************************************************************************************/
    // DRIVETRAIN
    if(rightController.getRawAxis(2) < -0.5)
    {
      drivetrain.curvatureDrive(0, 0, false);
    }
    else
    {
      drivetrain.curvatureDrive(leftController.getRawAxis(1), rightController.getRawAxis(0), rightController.getRawButton(2));
    }
  /********************************************************************************************************************************/

    /* SHOOTER AND STORAGE AND INTAKE*/
  /********************************************************************************************************************************/
    // SHOOTER
    if (rightController.getRawButton(1))
    {
      shooterMotor_master.set(0.90);
    }
    else
    {
      shooterMotor_master.set(0.0);
    }

    // STORAGE
    if(rightController.getRawButton(2))
    {
      storageMotor_master.set(VictorSPXControlMode.PercentOutput, -0.55);
    }
    else
    {
      storageMotor_master.set(VictorSPXControlMode.PercentOutput, 0.0);
    }

    // INTAKE
    if(leftController.getRawButton(1))
    {
      intakeMotor.set(0.5);
    }
    else
    {
      intakeMotor.set(0);
    }
  /********************************************************************************************************************************/
  
    /* CLIMB */
  /********************************************************************************************************************************/
    // STATIC
    staticClimbMotor_master.set(TalonFXControlMode.PercentOutput, 0);

    if(leftController.getPOV() == 0)
    {
      staticClimbMotor_master.set(TalonFXControlMode.PercentOutput, 0.9);
    }
    if(leftController.getPOV() == 180)
    {
      staticClimbMotor_master.set(TalonFXControlMode.PercentOutput, -0.9);
    }

    // DYNAMIC
    if(rightController.getPOV() == 45 || rightController.getPOV() == 90 || rightController.getPOV() == 135)
    {
      dynamicClimbMotor_master.set(.35);
    }
    if(rightController.getPOV() == 270)
    {
      dynamicClimbMotor_master.set(-.35);
    }
    else
    {
      dynamicClimbMotor_master.set(0.0);
    }
  /********************************************************************************************************************************/
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  /* HELPER METHODS */
  /********************************************************************************************************************************/
  // private void setStorageMotors(double speed)
  // {
  //   rightStorageMotor.set(ControlMode.PercentOutput, speed);
  //   leftStorageMotor.set(ControlMode.PercentOutput, -speed);
  // }
  /********************************************************************************************************************************/
}


