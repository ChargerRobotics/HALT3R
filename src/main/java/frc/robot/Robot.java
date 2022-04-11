package frc.robot;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot 
{
  /* DRIVETRAIN */
  /********************************************************************************************************************************/
  private PWMSparkMax leftFrontMotor = new PWMSparkMax(2);
  private CANSparkMax rightFrontMotor = new CANSparkMax(58, MotorType.kBrushed);
  private PWMSparkMax leftBackMotor = new PWMSparkMax(3);
  private CANSparkMax rightBackMotor = new CANSparkMax(59, MotorType.kBrushed);

  private MotorControllerGroup leftDrive = new MotorControllerGroup(leftFrontMotor, leftBackMotor);
  private MotorControllerGroup rightDrive = new MotorControllerGroup(rightFrontMotor, rightBackMotor);

  private Encoder leftDriveEncoder = new Encoder(8, 9, false, EncodingType.k4X);
  private Encoder rightDriveEncoder = new Encoder(6, 7, true, EncodingType.k4X);

  // PIDController rightEncoderLoop = new PIDController(1.0/0.029, 0.0, 0.0);
  // PIDController lefttEncoderLoop = new PIDController(1.0, 0.0, 0.0); //8.998

  private double leftDrivePower;
  private double rightDrivePower;
  /********************************************************************************************************************************/

  /* STORAGE AND INTAKE */
  /********************************************************************************************************************************/
  private CANSparkMax intakeMotor = new CANSparkMax(8, MotorType.kBrushless);
  //private VictorSPX rightStorageMotor = new VictorSPX(10);
  private VictorSPX leftStorageMotor = new VictorSPX(11);

  private DoubleSolenoid leftIntakePiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 7, 6);
  private DoubleSolenoid rightIntakePiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

  private VictorSPXConfiguration storageConfig = new VictorSPXConfiguration();
  /********************************************************************************************************************************/

  /* SHOOTER */
  /********************************************************************************************************************************/
  private CANSparkMax rightShooterMotor = new CANSparkMax(20, MotorType.kBrushless);
  private CANSparkMax leftShooterMotor = new CANSparkMax(21, MotorType.kBrushless);

  private RelativeEncoder shooterEncoder = leftShooterMotor.getEncoder(Type.kHallSensor, 42);

  private boolean canFire = false;
  /********************************************************************************************************************************/

  /* CLIMB */
  /********************************************************************************************************************************/
  private CANSparkMax rightStaticMotor = new CANSparkMax(40, MotorType.kBrushless);
  private CANSparkMax leftStaticMotor = new CANSparkMax(41, MotorType.kBrushless);
  
  private VictorSPX rightDynamicMotor = new VictorSPX(31);
  private VictorSPX leftDynamicMotor = new VictorSPX(30);

  private RelativeEncoder rightStaticEncoder = rightStaticMotor.getEncoder(Type.kHallSensor, 42);
  private RelativeEncoder leftStaticEncoder = leftStaticMotor.getEncoder(Type.kHallSensor, 42);

  private boolean isClimbLocked = true;
  /********************************************************************************************************************************/

  /* MISCELLANEOUS */
  /********************************************************************************************************************************/
  private Joystick leftController = new Joystick(0);
  private Joystick rightController = new Joystick(1);

  private Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  private UsbCamera camera1;
  private UsbCamera camera2;

  private VideoSink server;
  /********************************************************************************************************************************/
  
  @Override
  public void robotInit() 
  {
    // SET MOTOR DIRECTIONS
    leftShooterMotor.follow(rightShooterMotor, true);
    leftStorageMotor.setInverted(false);
    rightStaticMotor.setInverted(true);
    rightDynamicMotor.setInverted(true);

    leftDriveEncoder.setDistancePerPulse(1./256.);
    rightDriveEncoder.setDistancePerPulse(1./256.);

    storageConfig.voltageCompSaturation = 9.00;

    leftStorageMotor.configAllSettings(storageConfig);
    //rightStorageMotor.configAllSettings(storageConfig);

    // START SERVER

    server = CameraServer.addSwitchedCamera("Switchable Camera");
    // START CAMERA
    camera1 = CameraServer.startAutomaticCapture(0);
    camera1.setFPS(30);
    camera1.setResolution(160, 120);

    // START CAMERA
    camera2 = CameraServer.startAutomaticCapture(1);
    camera2.setFPS(30);
    camera2.setResolution(160, 120);

    // APPLY CAMERA TO SERVER
    server.setSource(camera2);

    // TURN OFF ALL MOTORS
    disableAllMotors();
    
    // CREATE SMARTDASHBOARD
    updateSmartDashboardValues();
  }

  @Override
    public void robotPeriodic() 
  {
    updateSmartDashboardValues();
  }

  @Override
  public void autonomousInit() 
  {
    disableAllMotors();
    rightDriveEncoder.reset();
    leftDriveEncoder.reset();

    // while(rightDriveEncoder.getDistance() < 4.5)
    // {
    //   rightDrive.set(rightEncoderLoop.calculate(rightDriveEncoder.getDistance(), -4.5));
    //   System.out.println(rightEncoderLoop.getPositionError());
    //   updateSmartDashboardValues();
    // }
    // rightDrive.set(0);
    // while(leftDriveEncoder.getDistance() < 4.4)
    // {
    //   leftDrive.set(lefttEncoderLoop.calculate(leftDriveEncoder.getDistance(), -4.5));
    //   System.out.println(lefttEncoderLoop.getPositionError());
    //   updateSmartDashboardValues();
    // }
    // leftDrive.set(0);

    leftIntakePiston.set(Value.kForward);
    rightIntakePiston.set(Value.kForward);

    
    // DRIVE TO BALL
    long startTime= System.currentTimeMillis(); 
    while(leftDriveEncoder.getDistance() < 4.5)
    {
      rightDrive.set(0.40);
      leftDrive.set(-0.385);
      intakeMotor.set(-0.7);
      updateSmartDashboardValues();

      if(startTime+ 5000 < System.currentTimeMillis())
      {
        return;
      }
    }
    leftDrive.set(0);
    rightDrive.set(0);
    Timer.delay(0.5);
    intakeMotor.set(-0.2);
    rightDriveEncoder.reset();
    leftDriveEncoder.reset();

    Timer.delay(0.5);
    
    // ROTATE ROBOT
    startTime= System.currentTimeMillis();
    while(leftDriveEncoder.getDistance() > -3.00)
    {
      rightDrive.set(0.30);
      leftDrive.set(0.32);

      if(startTime+ 5000 < System.currentTimeMillis())
      {
        return;
      }
    }
    leftDrive.set(0);
    rightDrive.set(0);
    leftDriveEncoder.reset();

    Timer.delay(0.5);

    // DRIVE TO LINE
    startTime= System.currentTimeMillis();
    while(rightDriveEncoder.getDistance() <  3.5)
    {
      rightDrive.set(0.40);
      leftDrive.set(-0.40);

      if(startTime+ 5000 < System.currentTimeMillis())
      {
        return;
      }
    }
    leftDrive.set(0);
    rightDrive.set(0);
    rightDriveEncoder.reset();
    intakeMotor.set(0);

    // leftStorageMotor.set(VictorSPXControlMode.PercentOutput, 0.70);
    // Timer.delay(0.4);
    // leftStorageMotor.set(VictorSPXControlMode.PercentOutput, 0);

    // // FIRE BALLS
    // rightShooterMotor.setVoltage(11.75);
    // Timer.delay(2.5);
    // leftStorageMotor.set(VictorSPXControlMode.PercentOutput, -0.75);
    // Timer.delay(2.0);
    // intakeMotor.set(-0.7);
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() 
  {
    disableAllMotors();

    // START COMPRESSOR
    compressor.enableDigital();

    // RESET DRIVETRAIN ENCODERS
    leftDriveEncoder.reset();
    rightDriveEncoder.reset();

    updateSmartDashboardValues();
  }

  @Override
  public void teleopPeriodic() 
  {
    /* DRIVETRAIN */
  /********************************************************************************************************************************/
    // APPLY JOYSTICK MOVEMENT TO VARIABLES
    leftDrivePower = leftController.getRawAxis(1);
    rightDrivePower = rightController.getRawAxis(1);

    // SCALE DOWN JOYSTICK AXIS VALUES
    leftDrivePower = leftDrivePower;
    rightDrivePower = rightDrivePower;

    // READS BUTTON VALUES TO ADJUST SPEED OF DRIVETRAIN
    if (rightController.getRawButton(5) == true) 
    {
      leftDrivePower = leftDrivePower * .99;
      rightDrivePower = rightDrivePower * .98;
    } 
    else 
    {
      leftDrivePower = leftDrivePower * .85;
      rightDrivePower = rightDrivePower * .83;
    }

    // REMOVES CONTROLLER DRIFT
    if (leftDrivePower < .1) 
    {
      if (leftDrivePower > -.1) 
      {
        leftDrivePower = 0;
      }
    }
    if (rightDrivePower < .1) 
    {
      if (rightDrivePower > -.1) 
      {
        rightDrivePower = 0;
      }
    }

    // SETS DRIVE SIDES TO AXES
    leftDrive.set(leftDrivePower);
    rightDrive.set(-rightDrivePower);
    
    // LOCK DRIVETRAIN 
    if(rightController.getRawAxis(2) > 0.25)
    {
      leftDrive.set(0);
      rightDrive.set(0);
      compressor.disable();
    }
    else
    {
      compressor.enableDigital();
    }
  /********************************************************************************************************************************/

    /* SHOOTER AND STORAGE AND INTAKE*/
  /********************************************************************************************************************************/
    // STORAGE
    //rightStorageMotor.set(VictorSPXControlMode.PercentOutput, 0.0);
    leftStorageMotor.set(VictorSPXControlMode.PercentOutput, 0.0);

    if(rightController.getRawButton(2))
    {
      //rightStorageMotor.set(VictorSPXControlMode.PercentOutput, -0.60);
      leftStorageMotor.set(VictorSPXControlMode.PercentOutput, -0.60);
    }

    if(rightController.getRawButton(3))
    {
      //rightStorageMotor.set(VictorSPXControlMode.PercentOutput, 0.55);
      leftStorageMotor.set(VictorSPXControlMode.PercentOutput, 0.55);
    }

    // SHOOTER
    rightShooterMotor.set(0.0);

    if(rightController.getRawButton(1))
    {
      rightShooterMotor.setVoltage(10.50);
    }

    if(rightController.getRawButton(6))
    {
      rightShooterMotor.set(-0.50);
      //rightStorageMotor.set(VictorSPXControlMode.PercentOutput, 0.50);
      leftStorageMotor.set(VictorSPXControlMode.PercentOutput, 0.50);
    }

    if(rightController.getRawButton(4))
    {
      rightShooterMotor.setVoltage(2.0);
      //rightStorageMotor.set(VictorSPXControlMode.PercentOutput, -0.55);
      leftStorageMotor.set(VictorSPXControlMode.PercentOutput, -0.55);
    }

    // INTAKE
    intakeMotor.set(0);

    if(leftController.getRawButton(1))
    {
      intakeMotor.set(-0.7);
    }

    if(leftController.getRawButton(5))
    {
      intakeMotor.set(0.7);
    }

    if(leftController.getRawButton(6))
    {
      leftIntakePiston.set(Value.kReverse);
      rightIntakePiston.set(Value.kReverse);
    }
    if(leftController.getRawButton(4))
    {
      leftIntakePiston.set(Value.kForward);
      rightIntakePiston.set(Value.kForward);
    }
  /********************************************************************************************************************************/
  
    /* CLIMB */
  /********************************************************************************************************************************/
    // STATIC CONTROL
    if(leftController.getPOV() == 180)
    {
      rightStaticMotor.set(-0.9);
      leftStaticMotor.set(-0.9);
    }
    if(leftController.getPOV() == 0)
    {
      rightStaticMotor.set( 0.9);
      leftStaticMotor.set(0.9);
    }

    if(leftController.getPOV() == -1)
    {
      rightStaticMotor.set(0.0);
      leftStaticMotor.set(0.0);
    }

    if(leftController.getRawButtonPressed(12))
    {
      isClimbLocked = false;
    }

    if(isClimbLocked)
    {
      if(rightStaticEncoder.getPosition() < 0)
      {
        rightStaticMotor.set(0.055);
      }
  
      if(leftStaticEncoder.getPosition() < 0)
      {
        leftStaticMotor.set(0.055);
      }
    }

    


    // DYNAMIC CONTROL
    if(rightController.getPOV() == 90)
    {
      rightDynamicMotor.set(VictorSPXControlMode.PercentOutput, 0.45);
      leftDynamicMotor.set(VictorSPXControlMode.PercentOutput, 0.45);
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
    
    if (leftController.getRawAxis(2) < 0.5) 
    {
    server.setSource(camera1);   
    } 
    else
    {
    server.setSource(camera2);
    }

    if (shooterEncoder.getVelocity() > 4700)
    {
      canFire = true;
    }
    else
    {
      canFire = false;
    }
  
    updateSmartDashboardValues();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() 
  {
    disableAllMotors();

    // CALLIBRATE RIGHT STATIC CLIMB MIN
    SmartDashboard.putString("Current Calibration", "Right Static");
    while(!rightController.getRawButton(3))
    {
      rightStaticMotor.set(rightController.getRawAxis(1) * 0.5);
      updateSmartDashboardValues();
    }
    updateSmartDashboardValues();
    rightStaticMotor.set(0);
    rightStaticEncoder.setPosition(0);
    Timer.delay(0.5);

    // // CALLIBRATE RIGHT STATIC CLIMB MAX
    // while(!rightController.getRawButton(3))
    // {
    //   rightStaticMotor.set(rightController.getRawAxis(1) * 0.5);
    //   updateSmartDashboardValues();
    // }
    // rightStaticMotor.set(0);
    // Timer.delay(0.5);

    // CALLIBRATE LEFT STATIC CLIMB
    SmartDashboard.putString("Current Calibration", "Left Static");
    while(!leftController.getRawButton(3))
    {
      leftStaticMotor.set(leftController.getRawAxis(1) * 0.5);
      updateSmartDashboardValues();
    }
    updateSmartDashboardValues();
    leftStaticMotor.set(0);
    leftStaticEncoder.setPosition(0);
    Timer.delay(0.5);

    // CALLIBRATE RIGHT DYNAMIC CLIMB
    SmartDashboard.putString("Current Calibration", "Right Dynamic");
    while(!rightController.getRawButton(3))
    {
      rightDynamicMotor.set(VictorSPXControlMode.PercentOutput, rightController.getRawAxis(1) * .6);
    }
    rightDynamicMotor.set(VictorSPXControlMode.PercentOutput, 0);
    rightStaticEncoder.setPosition(0);
    Timer.delay(0.5);

    // CALLIBRATE LEFT DYNAMIC CLIMB
    SmartDashboard.putString("Current Calibration", "Left Dynamic");
    while(!leftController.getRawButton(3))
    {
      leftDynamicMotor.set(VictorSPXControlMode.PercentOutput, leftController.getRawAxis(1) * .6);
    }
    leftDynamicMotor.set(VictorSPXControlMode.PercentOutput, 0);
    leftStaticEncoder.setPosition(0);
    
    SmartDashboard.putString("Current Calibration", "None");
    updateSmartDashboardValues();
    
  }

  @Override
  public void testPeriodic() { return; }

  /* HELPER METHODS */
  /********************************************************************************************************************************/
    private void updateSmartDashboardValues()
    {
      SmartDashboard.putNumber("Static Right Position", rightStaticEncoder.getPosition());
      SmartDashboard.putNumber("Static Left Position", leftStaticEncoder.getPosition());
      SmartDashboard.putNumber("Left Drive Encoder Position", leftDriveEncoder.getDistance());
      SmartDashboard.putNumber("Right Drive Encoder Position", rightDriveEncoder.getDistance());
      SmartDashboard.putNumber("Shooter Velocity", shooterEncoder.getVelocity());
      SmartDashboard.putBoolean("Fire:", canFire);
      SmartDashboard.putBoolean("Static Stoppers On?:", isClimbLocked);
    }

    private void disableAllMotors()
    {
      leftDrive.set(0);
      rightDrive.set(0);
      intakeMotor.set(0);
      //rightStorageMotor.set(VictorSPXControlMode.PercentOutput, 0);
      leftStorageMotor.set(VictorSPXControlMode.PercentOutput, 0);
      rightDynamicMotor.set(VictorSPXControlMode.PercentOutput, 0);
      leftDynamicMotor.set(VictorSPXControlMode.PercentOutput, 0);
      rightStaticMotor.set(0.0);
      leftStaticMotor.set(0.0);
      rightShooterMotor.set(0.0);
    }
    
  /********************************************************************************************************************************/
}


