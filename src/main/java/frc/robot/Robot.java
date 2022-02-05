// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
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
  
  public WPI_TalonFX fl_drive;
  public WPI_TalonFX fr_drive;
  public WPI_TalonFX bl_drive;
  public WPI_TalonFX br_drive;

  public CANSparkMax intake;
  public CANSparkMax shooter;
  public CANSparkMax index;

  double pdpNum;
  double distance;
  double leftMotorN;
  double rightMotorN;
  double shooterN;
  double intakeN;
  double indexN;
  double shooterRPM;
  double navXAngle;

  public XboxController gp;

  double speed;


  public PowerDistribution pdp;

  AnalogInput ultrasonic;  
  

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

    pdp = new PowerDistribution();
    
    ultrasonic = new AnalogInput(Statics.ultrasonic);

    ShuffleboardJunk();
    

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
    drive.arcadeDrive(gp.getRightTriggerAxis()-gp.getLeftTriggerAxis(), gp.getLeftX());
    
    if(gp.getXButtonPressed())
      Camera.changeCam();

    shooter.set(gp.getRightY());


    if(gp.getAButton() && shooter.getEncoder().getVelocity() ==  Statics.Shooter_Target_RPM){
      index.set(Statics.Index_Speed);
    } else {
      index.set(0);
    }

    if(gp.getBButton()){
      intake.set(Statics.Intake_Speed);
    } else {
      intake.set(0);
    }
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
    gp = new XboxController(Statics.XboxController_ID);
  }

  private void initializeMotorControllers() {

    fl_drive = new WPI_TalonFX(Statics.Front_Left_Motor_ID);
    fr_drive = new WPI_TalonFX(Statics.Front_Right_Motor_ID);
    bl_drive = new WPI_TalonFX(Statics.Back_Left_Motor_ID);
    br_drive = new WPI_TalonFX(Statics.Back_Right_Motor_ID);

    shooter = new CANSparkMax(Statics.Shooter_Motor_ID, MotorType.kBrushless);
    intake = new CANSparkMax(Statics.Intake_Motor_ID, MotorType.kBrushed);
    index = new CANSparkMax(Statics.Index_Motor_ID, MotorType.kBrushed);
  }



  public double getRangeInches(double rawVoltage){
    return rawVoltage * Statics.cm_to_in;
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
    
    pdpNum = pdp.getVoltage();
    distance = getRangeInches(ultrasonic.getValue());
    leftMotorN = fl_drive.get();
    rightMotorN = fr_drive.get();

    shooterN = shooter.get();
    indexN = index.get();
    intakeN = intake.get();

    shooterRPM = shooter.getEncoder().getVelocity();
    
    //navXAngle = navX.getAngle();
    
  }
  
  public void ShuffleboardJunk(){
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
