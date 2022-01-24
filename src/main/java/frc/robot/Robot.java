// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
  
  public static WPI_TalonFX fl_drive;
  public static WPI_TalonFX fr_drive;
  public static WPI_TalonFX bl_drive;
  public static WPI_TalonFX br_drive;

  public XboxController gp;

  double speed;


  public PowerDistribution pdp;

  AnalogInput ultrasonic;  
  

  //public VictorSPX motor1;
  ShuffleboardTab tab = Shuffleboard.getTab("Values");
  NetworkTableEntry rightMotorNetworkTable;
  NetworkTableEntry leftMotorNetworkTable;
  NetworkTableEntry ultrasonicDistance;
  NetworkTableEntry pdpVoltage;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    

    initializeMotorControllers();

    initializeGamePad();

    left_motors = new MotorControllerGroup(fl_drive, bl_drive);
    right_motors = new MotorControllerGroup(fr_drive, br_drive);
    drive = new DifferentialDrive(left_motors, right_motors);

    pdp = new PowerDistribution();
    
    ultrasonic = new AnalogInput(Statics.ultrasonic);

    rightMotorNetworkTable = tab.add("Right Motor Value", 1)
    .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -1, "max", 1))
    .getEntry();   

    leftMotorNetworkTable  = tab.add("Left Motor Value", 1)
    .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -1, "max", 1))
    .getEntry(); 
    
    ultrasonicDistance = tab.add("Distance to target", 0)
    .withWidget(BuiltInWidgets.kDial)
    .getEntry();

    pdpVoltage = tab.add("PDP voltage", 0)
    .withWidget(BuiltInWidgets.kVoltageView)
    .getEntry();


    //shuffleboardGo();
    Camera.startCameras();
    

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
    shuffleboardGo();
    
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

  private static void initializeMotorControllers() {
    fl_drive = new WPI_TalonFX(Statics.Front_Left_Motor_ID);
    fr_drive = new WPI_TalonFX(Statics.Front_Right_Motor_ID);
    bl_drive = new WPI_TalonFX(Statics.Back_Left_Motor_ID);
    br_drive = new WPI_TalonFX(Statics.Back_Right_Motor_ID);
  }



  


  public double getRangeInches(double rawVoltage){
    return rawVoltage * Statics.cm_to_in;
  }



  public void shuffleboardGo(){
    double pdpNum = pdp.getVoltage();
    double distance = getRangeInches(ultrasonic.getValue());
    double leftMotorN = fl_drive.get();
    double rightMotorN = fl_drive.get();
    
    pdpVoltage.setDouble(pdpNum);
    ultrasonicDistance.setDouble(distance);
    rightMotorNetworkTable.setDouble(rightMotorN);
    leftMotorNetworkTable.setDouble(leftMotorN);

    SmartDashboard.putNumber("Left Motor",fl_drive.get());  
    SmartDashboard.putNumber("Right Motor", fr_drive.get());

    SmartDashboard.putNumber("Voltage", pdp.getVoltage());
    SmartDashboard.putNumber("Ultrasonic Distance", getRangeInches(ultrasonic.getValue()));

    //SmartDashboard.putNumber("NAVX Z-Axis", navx.getYaw()); navX code needed
  }

  
  
  
}
