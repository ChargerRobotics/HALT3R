package frc.robot;

import java.util.Collection;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.music.Orchestra;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot 
{
  /* DRIVETRAIN */
  /********************************************************************************************************************************/
  private PWMSparkMax leftFrontMotor = new PWMSparkMax(0);
  private CANSparkMax rightFrontMotor = new CANSparkMax(60, MotorType.kBrushed);
  private PWMSparkMax leftBackMotor = new PWMSparkMax(2);
  private CANSparkMax rightBackMotorCanSparkMax = new CANSparkMax(61, MotorType.kBrushed);

  private MotorControllerGroup leftDrive = new MotorControllerGroup(leftFrontMotor, leftBackMotor);
  private MotorControllerGroup rightDrive = new MotorControllerGroup(rightFrontMotor, rightBackMotorCanSparkMax);

  private DifferentialDrive drivetrain;
  /********************************************************************************************************************************/

  /* STORAGE AND INTAKE */
  /********************************************************************************************************************************/
  private CANSparkMax intakeMotor = new CANSparkMax(8, MotorType.kBrushed);
  private VictorSPX rightStorageMotor = new VictorSPX(10);
  private VictorSPX leftStorageMotor = new VictorSPX(11);

  private DoubleSolenoid leftIntakePiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  private DoubleSolenoid rightIntakePiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
  /********************************************************************************************************************************/

  /* SHOOTER */
  /********************************************************************************************************************************/
  private CANSparkMax rightShooterMotor = new CANSparkMax(20, MotorType.kBrushless);
  private CANSparkMax leftShooterMotor = new CANSparkMax(21, MotorType.kBrushless);
  /********************************************************************************************************************************/

  /* CLIMB */
  /********************************************************************************************************************************/
  private TalonFX rightStaticMotor = new TalonFX(30);
  private TalonFX leftStaticMotor = new TalonFX(31);
  private TalonFXConfiguration talonConfig = new TalonFXConfiguration();
  
  private CANSparkMax rightDynamicMotor = new CANSparkMax(40, MotorType.kBrushless);
  private CANSparkMax leftDynamicMotor = new CANSparkMax(41, MotorType.kBrushless);

  private RelativeEncoder rightDynamicEncoder = rightDynamicMotor.getEncoder(Type.kHallSensor, 42);
  private RelativeEncoder leftDynamicEncoder = leftDynamicMotor.getEncoder(Type.kHallSensor, 42);

  private Orchestra orchestra = new Orchestra();
  
  /********************************************************************************************************************************/

  /* JOYSTICKS AND COMPRESSOR */
  /********************************************************************************************************************************/
  private Joystick leftController = new Joystick(0);
  private Joystick rightController = new Joystick(1);

  private Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  /********************************************************************************************************************************/
  
  @Override
  public void robotInit() 
  {
    // SET MOTOR DIRECTIONS
    leftShooterMotor.follow(rightShooterMotor, true);
    leftStorageMotor.follow(rightStorageMotor);
    leftStorageMotor.setInverted(InvertType.FollowMaster);
    rightStaticMotor.setInverted(TalonFXInvertType.CounterClockwise);
    leftStaticMotor.setInverted(TalonFXInvertType.Clockwise);
    rightDynamicMotor.setInverted(true);

    // SETUP DRIVETRAIN
    rightDrive.setInverted(true);
    drivetrain = new DifferentialDrive(leftDrive, rightDrive);

    // CONFIGURE STATIC CLIMB
    talonConfig.supplyCurrLimit.enable = true;
    talonConfig.supplyCurrLimit.triggerThresholdCurrent = 40;
    talonConfig.supplyCurrLimit.currentLimit = 38;
    talonConfig.supplyCurrLimit.triggerThresholdTime = 3.0;
    talonConfig.openloopRamp = 0.5;
    talonConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    
    rightStaticMotor.configAllSettings(talonConfig);
    leftStaticMotor.configAllSettings(talonConfig);

    // TURN OFF ALL MOTORS
    disableAllMotors();
    
    // CREATE SMARTDASHBOARD
    createSmartDashboardValues();
  }

  @Override
  public void robotPeriodic() 
  {
    SmartDashboard.updateValues();
  }

  @Override
  public void autonomousInit() 
  {
    
  }

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
    if(rightController.getRawAxis(2) < 0.5)
    {
      drivetrain.curvatureDrive(0, 0, false);
    }
    else
    {
      drivetrain.curvatureDrive(leftController.getRawAxis(1), rightController.getRawAxis(0) * -1, rightController.getRawButton(2));
    }
  /********************************************************************************************************************************/

    /* SHOOTER AND STORAGE AND INTAKE*/
  /********************************************************************************************************************************/
    // SHOOTER
    if (rightController.getRawButton(1))
    {
      rightShooterMotor.set(0.90);
    }
    else
    {
      rightShooterMotor.set(0.0);
    }

    // STORAGE
    if(rightController.getRawButton(3))
    {
      rightStorageMotor.set(VictorSPXControlMode.PercentOutput, -0.55);
    }
    else
    {
      rightStorageMotor.set(VictorSPXControlMode.PercentOutput, 0.0);
    }

    // INTAKE
    if(leftController.getRawButton(1))
    {
      intakeMotor.set(0.2);
    }
    else
    {
      intakeMotor.set(0);
    }
  /********************************************************************************************************************************/
  
    /* CLIMB */
  /********************************************************************************************************************************/
    // STATIC CONTROL
    if(leftController.getPOV() == 0)
    {
      rightStaticMotor.set(TalonFXControlMode.PercentOutput, 0.9);
      leftStaticMotor.set(TalonFXControlMode.PercentOutput, 0.9);
    }
    if(leftController.getPOV() == 180)
    {
      rightStaticMotor.set(TalonFXControlMode.PercentOutput, -0.9);
      leftStaticMotor.set(TalonFXControlMode.PercentOutput, -0.9);
    }

    if(leftController.getPOV() == -1)
    {
      rightStaticMotor.set(TalonFXControlMode.PercentOutput, 0.0);
      leftStaticMotor.set(TalonFXControlMode.PercentOutput, 0.0);
    }

    // DYNAMIC CONTROL
    if(rightController.getPOV() == 90)
    {
      rightDynamicMotor.set(0.25);
      leftDynamicMotor.set(0.25);
    }
    if(rightController.getPOV() == 270)
    {
      rightDynamicMotor.set(-0.25);
      leftDynamicMotor.set(-0.25);
    }

    if(rightController.getPOV() == -1)
    {
      rightDynamicMotor.set(0.00);
      leftDynamicMotor.set(0.00);
    }

    // CHECK LIMITS

  /********************************************************************************************************************************/
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() 
  {
    disableAllMotors();

    // CALLIBRATE RIGHT STATIC CLIMB
    while(!rightController.getRawButton(3))
    {
      rightStaticMotor.set(TalonFXControlMode.PercentOutput, rightController.getRawAxis(1) * 0.5);
      Shuffleboard.update();
    }
    
    rightStaticMotor.set(TalonFXControlMode.PercentOutput, 0);
    rightStaticMotor.setSelectedSensorPosition(0);
    Timer.delay(0.5);

    // CALLIBRATE LEFT STATIC CLIMB
    while(!leftController.getRawButton(3))
    {
      leftStaticMotor.set(TalonFXControlMode.PercentOutput, leftController.getRawAxis(1) * 0.5);
      Shuffleboard.update();
    }
    leftStaticMotor.set(TalonFXControlMode.PercentOutput, 0);
    leftStaticMotor.setSelectedSensorPosition(0);
    Timer.delay(0.5);

    // CALLIBRATE RIGHT DYNAMIC CLIMB
    while(!rightController.getRawButton(3))
    {
      rightDynamicMotor.set(rightController.getRawAxis(1) * .2);
      Shuffleboard.update();
    }
    rightDynamicMotor.set(0);
    rightDynamicEncoder.setPosition(0);
    Timer.delay(0.5);

    // CALLIBRATE LEFT DYNAMIC CLIMB
    while(!leftController.getRawButton(3))
    {
      leftDynamicMotor.set(leftController.getRawAxis(1) * .2);
      Shuffleboard.update();
    }
    leftDynamicMotor.set(0);
    leftDynamicEncoder.setPosition(0);
    
    // Shuffleboard.update();
  }

  @Override
  public void testPeriodic() {}

  /* HELPER METHODS */
  /********************************************************************************************************************************/
    private void createSmartDashboardValues()
    {
      SmartDashboard.putNumber("Air Pressure", compressor.getPressure());
      SmartDashboard.putNumber("Static Right Position", rightStaticMotor.getSelectedSensorPosition(0));
      SmartDashboard.putNumber("Static Left Position", leftStaticMotor.getSelectedSensorPosition(0));
      SmartDashboard.putNumber("Dynamic Right Position", rightDynamicEncoder.getPosition());
      SmartDashboard.putNumber("Dynamic Left Position", leftDynamicEncoder.getPosition());
      
    }

    private void disableAllMotors()
    {
      drivetrain.tankDrive(0, 0);
      intakeMotor.set(0);
      rightStorageMotor.set(VictorSPXControlMode.PercentOutput, 0);
      leftStorageMotor.set(VictorSPXControlMode.PercentOutput, 0);
      rightDynamicMotor.set(0);
      leftDynamicMotor.set(0);
      rightStaticMotor.set(TalonFXControlMode.PercentOutput, 0.0);
      leftStaticMotor.set(TalonFXControlMode.PercentOutput, 0.0);
    }
  /********************************************************************************************************************************/
}


