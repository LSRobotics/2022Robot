// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import java.util.*;



//import frc.robot.Constants.Statics;
//import main.java.frc.robot.GyroPIDController;

/**
 * import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
*/
//import com.kauailabs.navx.frc.AHRS;
//import com.ctre.phoenix.motorcontrol.NeutralMode;

;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  MotorControllerGroup left_motors;
  MotorControllerGroup right_motors;
  DifferentialDrive drive;
  
  public CANSparkMax fl_drive;
  public CANSparkMax fr_drive;
  public CANSparkMax bl_drive;
  public CANSparkMax br_drive;

  public WPI_TalonFX climbMotor1;
  public WPI_TalonFX climbMotor2;
  public WPI_TalonFX shooter;

  public Spark intake;
  public Spark index;
  public Spark intakeUpDown;

  double pdpNum;
  double distance;
  double leftMotorN;
  double rightMotorN;
  double shooterN;
  double intakeN;
  double indexN;
  double shooterRPM;
  double navXAngle;

  public XboxController gp1;
  public XboxController gp2;

  double shooterSpeed = Statics.Shooter_Speed;

  double speed;

  boolean goingUp = false;


  public PowerDistribution pdp;

  AnalogInput ultrasonic;  
  DigitalInput bottomLimitSwitch = new DigitalInput(0);
  DigitalInput topLimitSwitch = new DigitalInput(1);
  

  //public VictorSPX motor1;
  ShuffleboardTab testTab = Shuffleboard.getTab("Test Board");
  ShuffleboardTab compTab = Shuffleboard.getTab("Competition Board");
  NetworkTableEntry rightMotorNetworkTable;
  NetworkTableEntry leftMotorNetworkTable;
  NetworkTableEntry ultrasonicDistance;
  NetworkTableEntry pdpVoltage;
  NetworkTableEntry ShooterTable;
  NetworkTableEntry IndexTable;
  NetworkTableEntry IntakeTable;
  NetworkTableEntry shooterRPMEntry;
  SimpleWidget navXEntry;
  ComplexWidget cameraTest;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    
    Camera.startCameras();

    initializeMotorControllers();

    initializeGamePad();

    left_motors = new MotorControllerGroup(fl_drive, bl_drive);
    right_motors = new MotorControllerGroup(fr_drive, br_drive);
    drive = new DifferentialDrive(left_motors, right_motors);

    //pdp = new PowerDistribution();
    
    ultrasonic = new AnalogInput(Statics.ultrasonic);

    shuffleboardStartup();

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {   
    updateInputs();
    updateNetworkEntries();
    
  }
  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
    driveTrain(gp1.getLeftY(), gp1.getRightX());
    setIntakerPosition(gp1.getAButtonPressed(), gp1.getBButtonPressed());    


    if(gp2.getXButtonPressed())
      Camera.changeCam();

    if(gp2.getYButton()) {
      shooter.set(shooterSpeed);
      
      if (Math.abs(shooter.getSelectedSensorVelocity()) > Statics.Shooter_Target_RPM) //todo
        index.set(-Statics.Index_Speed);
    } 
    else {
      shooter.set(0);
      index.set(0);
    }
    

    if (gp2.getRightBumperPressed()){
      shooterSpeed += 0.05;
    }
    
    if (gp2.getLeftBumperPressed()){
      shooterSpeed -= 0.05;
    }

    /* Could be Useful for testing - only for intake
    if(gp1.getBButton()){
      intake.set(Statics.Intake_Speed);
    } else {
      intake.set(0);
    }
    */
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  private void initializeGamePad(){
    gp1 = new XboxController(Statics.XboxController1_ID);
    gp2 = new XboxController(Statics.XboxController2_ID);
  }

  private void initializeMotorControllers() {

    
    climbMotor1 = new WPI_TalonFX(Statics.ClimbMotor1ID);
    climbMotor2 = new WPI_TalonFX(Statics.ClimbMotor2ID);
    shooter = new WPI_TalonFX(Statics.Shooter_Motor_ID);

    intake = new Spark(Statics.Intake_Motor_ID);
    index = new Spark(Statics.Index_Motor_ID);
    intakeUpDown = new Spark(Statics.Intake_Up_Down_Motor_ID);
    fl_drive = new CANSparkMax(Statics.Front_Left_Motor_ID, MotorType.kBrushless);
    fr_drive = new CANSparkMax(Statics.Front_Right_Motor_ID, MotorType.kBrushless);
    bl_drive = new CANSparkMax(Statics.Back_Left_Motor_ID, MotorType.kBrushless);
    br_drive = new CANSparkMax(Statics.Back_Right_Motor_ID, MotorType.kBrushless);
    
  }


  private void driveTrain(double power, double turn) {
    drive.arcadeDrive(cubicScaledDeadband(power, Statics.deadbandCutoff, Statics.Weight),
                      cubicScaledDeadband(turn, Statics.deadbandCutoff, Statics.Weight));

  }

  private double cubic(double x, double w){
    return w * x * x * x  + (1.0 - w) * x;
  }


  public double getRangeInches(double rawVoltage){
    return rawVoltage * Statics.cm_to_in;
  }

  public double cubicScaledDeadband(double x, double deadbandCutoff, double weight){
    if (Math.abs(x) < deadbandCutoff) {
      return 0;
    } else {
      return (cubic(x, weight)- (Math.abs(x)/x)* cubic(deadbandCutoff, weight)) / (1.0 - cubic(deadbandCutoff, weight));
    }
  }

  public void updateNetworkEntries(){
    
    pdpVoltage.setDouble(pdpNum);
    ultrasonicDistance.setDouble(distance);
    rightMotorNetworkTable.setDouble(rightMotorN);
    leftMotorNetworkTable.setDouble(leftMotorN);

    IndexTable.setDouble(indexN);
    ShooterTable.setDouble(shooterN);
    IntakeTable.setDouble(intakeN);

    shooterRPMEntry.setDouble(shooterRPM);
    
    
    //navXEntry.setDouble(navXAngle)
    
  }
  public void updateInputs(){
    
    //pdpNum = pdp.getVoltage();
    distance = getRangeInches(ultrasonic.getValue());
    leftMotorN = fl_drive.get();
    rightMotorN = fr_drive.get();

    shooterN = shooter.get();
    indexN = index.get();
    intakeN = intake.get();

    shooterRPM = shooter.getSelectedSensorVelocity();
    
    //navXAngle = navX.getAngle();
    
  }

  //If button is pressed move until limit switch
  public void setIntakerPosition(boolean left, boolean right){

      //Controls actual intake - only on if at bottom
      if (bottomLimitSwitch.get() && left) {
        intake.set(Statics.Intake_Speed);
      }
      else {
        intake.set(0);
      }

      //defaults movement to go up - if B button - go up until limit switch
      if (!topLimitSwitch.get() && right) 
        goingUp = true;
      else if (topLimitSwitch.get())
        goingUp = false;

        //If we're not going up, figure out if we want to go down until bottom limit switch
      if (goingUp)
        intakeUpDown.set(-Statics.IntakeUppeyDowneySpeed);
        else if (!bottomLimitSwitch.get() && left)
        intakeUpDown.set(Statics.IntakeUppeyDowneySpeed);
        else 
         intakeUpDown.set(0);
  }
  
  public void shuffleboardStartup(){
    rightMotorNetworkTable = testTab.add("Right Motor Value", 1)
    .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -1, "max", 1))
    .withSize(2, 1)
    .withPosition(0, 0)
    .getEntry();   

    leftMotorNetworkTable  = testTab.add("Left Motor Value", 1)
    .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -1, "max", 1))
    .withSize(2, 1)
    .withPosition(2, 0)
    .getEntry();
    
    ultrasonicDistance = testTab.add("Distance to target", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withSize(3, 2)
    .withPosition(0, 3)
    .getEntry();

    pdpVoltage = testTab.add("PDP voltage", 0)
    .withWidget(BuiltInWidgets.kVoltageView)
    .withSize(1, 1)
    .withPosition(7, 0)
    .getEntry();

    testTab.add("camera", Camera.cam0)
    .withWidget(BuiltInWidgets.kCameraStream)
    .withSize( 1, 1)
    .withPosition(4 , 0);
    
    testTab.add("camera dos", Camera.cam1)
    .withWidget(BuiltInWidgets.kCameraStream)
    .withSize(1,1)
    .withPosition(4,2);


    //Start of competition tab stuff
    rightMotorNetworkTable = compTab.add("Right Motor Value", 1)
    .withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min", -1, "max", 1))
    .withSize(2, 1)
    .withPosition(0, 0)
    .getEntry();   

    leftMotorNetworkTable  = compTab.add("Left Motor Value", 1)
    .withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min", -1, "max", 1))
    .withSize(2, 1)
    .withPosition(2, 0)
    .getEntry(); 
    
    ultrasonicDistance = compTab.add("Distance to target", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withSize(3, 2)
    .withPosition(0, 3)
    .getEntry();

    pdpVoltage = compTab.add("PDP voltage", 0)
    .withWidget(BuiltInWidgets.kPowerDistribution)
    .withSize(1, 1)
    .withPosition(7, 0)
    .getEntry();

    cameraTest = compTab.add("camera",Camera.cam0)
    .withWidget(BuiltInWidgets.kCameraStream)
    .withSize( 2, 2)
    .withPosition(4 , 0);

    compTab.add("Camera Numero Two",Camera.cam1)
    .withWidget(BuiltInWidgets.kCameraStream)
    .withSize(1,1)
    .withPosition(4,2);

    compTab.add("Differential Drive", drive)
    .withWidget(BuiltInWidgets.kDifferentialDrive)
    .withSize(1,1)
    .withPosition(2,2);

    shooterRPMEntry = compTab.add("Shooter RPM", 0)
    .withWidget(BuiltInWidgets.kNumberBar)
    .withSize(1,1)
    .withPosition(7, 1)
    .getEntry();

    IntakeTable = compTab.add("Intake", 0)
    .withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min", -1, "max", 1))
    .withSize(1, 1)
    .withPosition(7, 3)
    .getEntry();

    ShooterTable = compTab.add("Shooter", 0)
    .withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min", -1, "max", 1))
    .withSize(1, 1)
    .withPosition(6, 3)
    .getEntry();

    IndexTable = compTab.add("Index", 0)
    .withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min", -1, "max", 1))
    .withSize(1, 1)
    .withPosition(7, 4)
    .getEntry();

    /*
    compTab.add("navX Angle", navX)
    .withWidget(BuiltInWidgets.kGyro)
    .withSize(1,1)
    .withPosition(3, 3);

    OR (Need to check both)
    
    navXEntry = compTab.add("navX Angle", 0)
    .withWidget(BuiltInWidgets.kGyro)
    .withSize(1,1)
    .withPosition(3, 3);
    */
  }
  
  
}
