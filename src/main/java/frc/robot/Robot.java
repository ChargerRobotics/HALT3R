package frc.robot;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
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
  private CANSparkMax rightFrontMotor = new CANSparkMax(58, MotorType.kBrushed);
  private PWMSparkMax leftBackMotor = new PWMSparkMax(1);
  private CANSparkMax rightBackMotor = new CANSparkMax(59, MotorType.kBrushed);

  private MotorControllerGroup leftDrive = new MotorControllerGroup(leftFrontMotor, leftBackMotor);
  private MotorControllerGroup rightDrive = new MotorControllerGroup(rightFrontMotor, rightBackMotor);

  private double leftDrivePower;
  private double rightDrivePower;

  private DifferentialDrive drivetrain = new DifferentialDrive(leftDrive, rightDrive);
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
  private CANSparkMax rightStaticMotor = new CANSparkMax(40, MotorType.kBrushless);
  private CANSparkMax leftStaticMotor = new CANSparkMax(41, MotorType.kBrushless);
  
  private VictorSPX rightDynamicMotor = new VictorSPX(30);
  private VictorSPX leftDynamicMotor = new VictorSPX(31);

  private RelativeEncoder rightStaticEncoder = rightStaticMotor.getEncoder(Type.kHallSensor, 42);
  private RelativeEncoder leftStaticEncoder = leftStaticMotor.getEncoder(Type.kHallSensor, 42);
  /********************************************************************************************************************************/

  /* MISCELLANEOUS */
  /********************************************************************************************************************************/
  private Joystick leftController = new Joystick(0);
  private Joystick rightController = new Joystick(1);

  private Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  private PowerDistribution pdp = new PowerDistribution();
  /********************************************************************************************************************************/
  
  @Override
  public void robotInit() 
  {
    // SET MOTOR DIRECTIONS
    leftShooterMotor.follow(rightShooterMotor, true);
    leftStorageMotor.setInverted(false);
    rightStaticMotor.setInverted(false);
    rightDynamicMotor.setInverted(true);

    // RESET PDP
    pdp.clearStickyFaults();

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
    compressor.disable();
  }

  @Override
  public void teleopPeriodic() 
  {
    /* DRIVETRAIN */
  /********************************************************************************************************************************/
    // APPLY JOYSTICK MOVEMENT TO VARIABLES
    leftDrivePower = leftController.getRawAxis(1);
    rightDrivePower = rightController.getRawAxis(1);

    // REMOVE DRIFT
    if(Math.abs(leftDrivePower) < .10)
    {
      leftDrivePower = 0;
    }

    if(Math.abs(rightDrivePower) < .10)
    {
      rightDrivePower = 0;
    }

    // DRIVETRAIN
    if(rightController.getRawAxis(2) < 0.5)
    {
      drivetrain.tankDrive(0, 0);
    }
    else
    {
      drivetrain.tankDrive(leftDrivePower * 1.1, rightDrivePower * 1.1, !rightController.getRawButton(2));
    }
    
  /********************************************************************************************************************************/

    /* SHOOTER AND STORAGE AND INTAKE*/
  /********************************************************************************************************************************/
    // SHOOTER
    rightShooterMotor.set(0.0);

    if(rightController.getRawButton(1))
    {
      rightShooterMotor.setVoltage(11.50);
    }

    if(rightController.getRawButton(1) && rightController.getRawButton(6))
    {
      rightShooterMotor.set(-0.30);
    }

    // STORAGE
    rightStorageMotor.set(VictorSPXControlMode.PercentOutput, 0.0);
    leftStorageMotor.set(VictorSPXControlMode.PercentOutput, 0.0);

    if(rightController.getRawButton(3))
    {
      rightStorageMotor.set(VictorSPXControlMode.PercentOutput, -0.60);
      leftStorageMotor.set(VictorSPXControlMode.PercentOutput, -0.60);
    }

    if(rightController.getRawButton(5))
    {
      rightStorageMotor.set(VictorSPXControlMode.PercentOutput, 0.45);
      leftStorageMotor.set(VictorSPXControlMode.PercentOutput, 0.45);
    }

    // INTAKE
    intakeMotor.set(0);

    if(leftController.getRawButton(1))
    {
      intakeMotor.set(0.3);
    }

    if(leftController.getRawButton(1) && leftController.getRawButton(5))
    {
      intakeMotor.set(-0.2);
    }
  /********************************************************************************************************************************/
  
    /* CLIMB */
  /********************************************************************************************************************************/
    // STATIC CONTROL
    if(leftController.getPOV() == 0)
    {
      rightStaticMotor.set(-0.9);
      leftStaticMotor.set(-0.9);
    }
    if(leftController.getPOV() == 180)
    {
      rightStaticMotor.set( 0.9);
      leftStaticMotor.set(0.9);
    }

    if(leftController.getPOV() == -1)
    {
      rightStaticMotor.set(0.0);
      leftStaticMotor.set(0.0);
    }

    // DYNAMIC CONTROL
    if(rightController.getPOV() == 90)
    {
      rightDynamicMotor.set(VictorSPXControlMode.PercentOutput, 0.33);
      leftDynamicMotor.set(VictorSPXControlMode.PercentOutput, 0.33);
    }
    if(rightController.getPOV() == 270)
    {
      rightDynamicMotor.set(VictorSPXControlMode.PercentOutput, -0.55);
      leftDynamicMotor.set(VictorSPXControlMode.PercentOutput, -0.55);
    }

    if(rightController.getPOV() == -1)
    {
      rightDynamicMotor.set(VictorSPXControlMode.PercentOutput, -0.07);
      leftDynamicMotor.set(VictorSPXControlMode.PercentOutput, -0.07);
    }
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
    
    // CALLIBRATE RIGHT STATIC CLIMB MAX HEIGHT
    SmartDashboard.putString("Current Calibration", "Right Static - MAX HEIGHT");
    while(!rightController.getRawButton(3))
    {
      rightStaticMotor.set(rightController.getRawAxis(1) * 0.5);
      Shuffleboard.update();
    }
    rightStaticMotor.setSoftLimit(SoftLimitDirection.kForward, (float) rightStaticEncoder.getPosition());
    rightStaticMotor.set(0);
    Timer.delay(0.5);

    // CALLIBRATE RIGHT STATIC CLIMB MIN HEIGHT
    SmartDashboard.putString("Current Calibration", "Right Static - MIN HEIGHT");
    while(!rightController.getRawButton(3))
    {
      rightStaticMotor.set(rightController.getRawAxis(1) * 0.5);
      Shuffleboard.update();
    }
    rightStaticMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) rightStaticEncoder.getPosition());
    rightStaticMotor.set(0);
    Timer.delay(0.5);

    // CALLIBRATE LEFT STATIC CLIMB
    SmartDashboard.putString("Current Calibration", "Left Static");
    while(!leftController.getRawButton(3))
    {
      leftStaticMotor.set(leftController.getRawAxis(1) * 0.5);
      Shuffleboard.update();
    }
    leftStaticMotor.set(0);
    Timer.delay(0.5);

    // CALLIBRATE RIGHT DYNAMIC CLIMB
    SmartDashboard.putString("Current Calibration", "Right Dynamic");
    while(!rightController.getRawButton(3))
    {
      rightDynamicMotor.set(VictorSPXControlMode.PercentOutput, rightController.getRawAxis(1) * .2);
    }
    rightDynamicMotor.set(VictorSPXControlMode.PercentOutput, 0);
    rightStaticEncoder.setPosition(0);
    Timer.delay(0.5);

    // CALLIBRATE LEFT DYNAMIC CLIMB
    SmartDashboard.putString("Current Calibration", "Left Dynamic");
    while(!leftController.getRawButton(3))
    {
      leftDynamicMotor.set(VictorSPXControlMode.PercentOutput, leftController.getRawAxis(1) * .2);
    }
    leftDynamicMotor.set(VictorSPXControlMode.PercentOutput, 0);
    leftStaticEncoder.setPosition(0);
    
    // Shuffleboard.update();
  }

  @Override
  public void testPeriodic() {}

  /* HELPER METHODS */
  /********************************************************************************************************************************/
    private void createSmartDashboardValues()
    {
      SmartDashboard.putNumber("Air Pressure", compressor.getPressure());
      SmartDashboard.putNumber("Static Right Position", rightStaticEncoder.getPosition());
      SmartDashboard.putNumber("Static Left Position", leftStaticEncoder.getPosition());
      
    }

    private void disableAllMotors()
    {
      drivetrain.tankDrive(0, 0);
      intakeMotor.set(0);
      rightStorageMotor.set(VictorSPXControlMode.PercentOutput, 0);
      leftStorageMotor.set(VictorSPXControlMode.PercentOutput, 0);
      rightDynamicMotor.set(VictorSPXControlMode.PercentOutput, 0);
      leftDynamicMotor.set(VictorSPXControlMode.PercentOutput, 0);
      rightStaticMotor.set(0.0);
      leftStaticMotor.set(0.0);
    }
  /********************************************************************************************************************************/
}


