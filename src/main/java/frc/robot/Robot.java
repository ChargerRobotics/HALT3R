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
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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
  private PWMSparkMax leftFrontMotor = new PWMSparkMax(0);
  private CANSparkMax rightFrontMotor = new CANSparkMax(58, MotorType.kBrushed);
  private PWMSparkMax leftBackMotor = new PWMSparkMax(1);
  private CANSparkMax rightBackMotor = new CANSparkMax(59, MotorType.kBrushed);

  private MotorControllerGroup leftDrive = new MotorControllerGroup(leftFrontMotor, leftBackMotor);
  private MotorControllerGroup rightDrive = new MotorControllerGroup(rightFrontMotor, rightBackMotor);

  private Encoder leftDriveEncoder = new Encoder(8, 9, false, EncodingType.k2X);
  private Encoder rightDriveEncoder = new Encoder(6, 7, true, EncodingType.k2X);

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
    intakeMotor.setInverted(true);
    leftShooterMotor.follow(rightShooterMotor, true);
    leftStorageMotor.setInverted(false);
    rightStaticMotor.setInverted(true);
    rightDynamicMotor.setInverted(true);

    leftDriveEncoder.setDistancePerPulse(1./256.);
    rightDriveEncoder.setDistancePerPulse(1./256.);

    storageConfig.voltageCompSaturation = 9.00;

    leftStorageMotor.configAllSettings(storageConfig);
    //rightStorageMotor.configAllSettings(storageConfig);


    server = CameraServer.addSwitchedCamera("Switchable Camera");
    // START CAMERA
    camera1 = CameraServer.startAutomaticCapture(0);
    camera1.setFPS(30);
    camera1.setResolution(160, 120);

    // START CAMERA
    camera2 = CameraServer.startAutomaticCapture(1);
    camera2.setFPS(30);
    camera2.setResolution(160, 120);

    server.setSource(camera2);

    // TURN OFF ALL MOTORS
    disableAllMotors();
    
    // CREATE SMARTDASHBOARD
    updateSmartDashboardValues();
  }

  @Override
    public void robotPeriodic() 
  {
    SmartDashboard.updateValues();
  }

  @Override
  public void autonomousInit() 
  {
    disableAllMotors();
    rightDriveEncoder.reset();
    leftDriveEncoder.reset();

    leftIntakePiston.set(Value.kForward);
    rightIntakePiston.set(Value.kForward);

    // // TURN TOWARDS BALL
    // while(rightDriveEncoder.getDistance() < 0.55)
    // {
    //   rightDrive.set(0.15);
    // }
    // rightDrive.set(0);
    // rightDriveEncoder.reset(); /// ...< 7.0

    // DRIVE TO BALL
    while(rightDriveEncoder.getDistance() < 4.00)
    {
      rightDrive.set(0.30);
      leftDrive.set(-0.32);
      intakeMotor.set(0.7);
    }
    leftDrive.set(0);
    rightDrive.set(0);
    intakeMotor.set(0);
    rightDriveEncoder.reset();

    Timer.delay(0.5);
    
    // ROTATE ROBOT
    while(rightDriveEncoder.getDistance() < 3.00)
    {
      rightDrive.set(0.20);
      leftDrive.set(0.22);
    }
    leftDrive.set(0);
    rightDrive.set(0);
    rightDriveEncoder.reset();

    Timer.delay(0.5);

    // DRIVE TO LINE
    while(rightDriveEncoder.getDistance() <  3.5)
    {
      rightDrive.set(0.30);
      leftDrive.set(-0.32);
    }
    leftDrive.set(0);
    rightDrive.set(0);
    rightDriveEncoder.reset();

    leftStorageMotor.set(VictorSPXControlMode.PercentOutput, 0.70);
    Timer.delay(0.4);
    leftStorageMotor.set(VictorSPXControlMode.PercentOutput, 0);

    // FIRE BALLS
    rightShooterMotor.setVoltage(10.50);
    Timer.delay(2.25);
    leftStorageMotor.set(VictorSPXControlMode.PercentOutput, -0.75);
    Timer.delay(2.5);
    intakeMotor.set(1.5);
    Timer.delay(3);
    disableAllMotors();

  }

  @Override
  public void autonomousPeriodic() 
  {
    disableAllMotors();

    
  }

  @Override
  public void teleopInit() 
  {
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
    leftDrivePower = leftDrivePower*.8;
    rightDrivePower = rightDrivePower*.8;

    // READS BUTTON VALUES TO ADJUST SPEED OF DRIVETRAIN
    if (rightController.getRawButton(5) == true) 
    {
      leftDrivePower = leftDrivePower * .95;
      rightDrivePower = rightDrivePower * .95;
    } 
    else 
    {
      leftDrivePower = leftDrivePower * .75;
      rightDrivePower = rightDrivePower * .75;
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
      intakeMotor.set(0.7);
    }

    if(leftController.getRawButton(5))
    {
      intakeMotor.set(-0.45);
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

    if(rightStaticEncoder.getPosition() < 0)
    {
      rightStaticMotor.set(0.055);
    }

    if(leftStaticEncoder.getPosition() < 0)
    {
      leftStaticMotor.set(0.055);
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
    
    if (leftController.getRawAxis(2) > 0.5) 
    {
    server.setSource(camera1);
    compressor.disable();    
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
  }

  @Override
  public void testPeriodic() {}

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


