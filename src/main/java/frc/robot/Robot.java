// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import java.util.*;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.AnalogInput;

//import com.kauailabs.navx.frc.AHRS;
//import edu.wpi.first.wpilibj.SPI; //TODO: change the port system depending on what we actually use

import java.awt.Desktop;
import java.io.*;
import java.util.Scanner;
import java.util.stream.Collectors;

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

  public Servo climbRatchet;

  public WPI_TalonFX verticalClimb;
  public Spark horizontalClimb;

  public WPI_TalonFX shooter;

  public AnalogInput ballIRSensor;

  public Spark intake;
  public Spark index;
  public Spark intakeUpDown;
  public Spark LED;

  double pdpNum;
  double distance;
  double leftMotorN;
  double rightMotorN;
  double shooterN;
  double intakeN;
  double indexN;
  double shooterRPM;
  double navXAngle;
  double ratchetPos;

  public XboxController gp1;
  public XboxController gp2;

  double shooterSpeed = Statics.Shooter_Speed;

  double speed;

  int upDown = 0;
  double servoAngle = 0;

  public PowerDistribution pdp;

  AnalogInput ultrasonic;  
  DigitalInput bottomLimitSwitchIntake = new DigitalInput(2);
  DigitalInput topLimitSwitchIntake = new DigitalInput(3);
  Servo angleAdjuster = new Servo(4); //channel??
  
  ShuffleboardTab testTab = Shuffleboard.getTab("Test Board");
  ShuffleboardTab compTab = Shuffleboard.getTab("Competition Board");
  NetworkTableEntry rightMotorNetworkTable;
  NetworkTableEntry leftMotorNetworkTable;
  NetworkTableEntry pdpVoltage;
  NetworkTableEntry ShooterTable;
  NetworkTableEntry IndexTable;
  NetworkTableEntry IntakeTable;
  NetworkTableEntry shooterRPMEntry;
  NetworkTableEntry ratchetEngaged;
  SimpleWidget navXEntry;
  ComplexWidget cameraTest;



  //AutonInit
  private PIDController movePid;
  private PIDController gyroPid;

  private int autoIncrement;

  //private AHRS ahrs;

  private enum AutonMode {
    DRIVE, 
    TURN,
    SHOOT,
    DEPLOYINTAKE,
    INTAKEON,
    INTAKEOFF,
    WAIT,
    NONE
  }
  AutonMode[] autonModes;
  String[][] autonArguments;

  private AutonMode currentAuton;

  private boolean autonConditionCompleted;

  private double autonTarget;

  private double autonStartingPos;

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
    right_motors.setInverted(true);
    drive = new DifferentialDrive(left_motors, right_motors);

    fl_drive.setOpenLoopRampRate(0.5);
    fr_drive.setOpenLoopRampRate(0.5);
    bl_drive.setOpenLoopRampRate(0.5);
    br_drive.setOpenLoopRampRate(0.5);
    
    verticalClimb.setNeutralMode(NeutralMode.Brake);

    pdp = new PowerDistribution();
    
    ballIRSensor = new AnalogInput(0);

    shuffleboardStartup();
    LED.set(-0.43);
    
    movePid = new PIDController(Statics.movementPIDp, Statics.movementPIDi, Statics.movementPidd); //TODO: figure out the kP, kI, and kD values required for actual instantiation
    gyroPid = new PIDController(Statics.gyroPIDp, Statics.gyroPIDi, Statics.gyroPIDd); //TODO: figure out the kP, kI, and kD values required for actual instantiation

    //ahrs = new AHRS(SPI.Port.kMXP);


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
    autoIncrement = -1;
    autonConditionCompleted = true;
    currentAuton = AutonMode.NONE;
    ArrayList<AutonMode> tempAutonModes = new ArrayList<AutonMode>();
    ArrayList<String[]> tempAutonArguments = new ArrayList<String[]>();

    //Spaghetti code:
    File autonInstructionFile = new File(Filesystem.getDeployDirectory().getPath() + "/autonInstructions.txt");
    try (Scanner input = new Scanner(autonInstructionFile)) {
      while (input.hasNext()) {
        String line = input.nextLine();
        Scanner lineInput = new Scanner(line); //another scanner so you can use the delimiters
        lineInput.useDelimiter(", ");
        tempAutonModes.add(AutonMode.valueOf(lineInput.next().toUpperCase()));
        ArrayList<String> tempLineArgs = new ArrayList<String>();
        for (String x : lineInput.tokens().collect(Collectors.toList())) {
          tempLineArgs.add(x);
        }
        System.out.println("QQQQ");
        tempAutonArguments.add(tempLineArgs.toArray(new String[0]));
        System.out.println(lineInput.tokens().toArray());
        lineInput.close();
      }
      input.close();
    } catch (FileNotFoundException e) {
      //Auto-Generated catch block :)
      e.printStackTrace();
    }
    
    autonModes = tempAutonModes.toArray(new AutonMode[0]); //TODO: check and see if you need to do `tempAutonModes.size()` instead of `0`
    autonArguments = tempAutonArguments.toArray(new String[0][0]);
  }
  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    //When each step of autonomous is completed
    
    if (autonConditionCompleted) {
      autoIncrement++;

    
      // AUTO INCREMENT [PLACE AUTON INSTRUCTIONS HERE]
      // - each switch case is another instruction
      // - currently it is a switch statement and not an array to allow for some alternative functions to be called other than `set auton`
      // - SetAuton(AutonMode, targetValue) is the main function being used currently.
      // - Current AutonModes are `DRIVE` and `TURN`, with `NONE` being just to do nothing
      // - the targetValue is the value whatever the specific AutonMode is measuring should reach

      setAuton(autonModes[autoIncrement], autonArguments[autoIncrement]);
      System.out.println(autonModes[autoIncrement].toString());
      System.out.println(autonArguments[autoIncrement].toString());
      //auton 1
        //starting position: directly under hub
        //wait a variable amount of seconds (allow it to be changed in dashboard)
        //shoot the ball directly into the low goal
        //turn to face position 1 (see plans)
        //move away (to get out of the way of others)
      //auton 2
        //starting position: directly under hub
        //wait a variable amount of seconds (allow it to be changed in dashboard)
        //shoot the ball directly into the low goal
        //turn to face position 1 (see plans)
        //move away (to get out of the way of others)
        //drop intake
        //pick up ball (predetermined position)
        //turn to aim back at goal
        //shoot the ball into either low or high
      //auton 3
        //drop intake
        //move to pick up cargo
        //turn to aim at goal
        //move into position
        //score goal
      autonConditionCompleted = false;
      System.out.println("Started Next Auton Instruction");
      drive.arcadeDrive(0,0);
    }
    else {
      switch (currentAuton) {
        // DRIVE MODE
        // - Drives forward some distance in **INSERT**UNITS**HERE**
        // - Uses the ahrs in order to ensure the robot drives straight
        case DRIVE: 

          //double error = ahrs.getAngle();
          //double turn = error;
          
          double valueToCalculate = (getAverageEncoderDistance()-autonStartingPos)/Statics.SensorToMeters;
          //System.out.println(movePid.getSetpoint());
          double rawValue = movePid.calculate(valueToCalculate);
          double driveValue = MathUtil.clamp(rawValue, -1, 1);
          drive.arcadeDrive(Statics.Drive_Speed*driveValue, 0); //TODO: divide `getAverageEncoderDistance()-autonStartingPos` by the sensor units to actual units constant
          if (movePid.atSetpoint()) {
            autonConditionCompleted = true;
          }
          break;
        // TURN MODE 
        // - Turns some distance in degrees
        case TURN: 
          //double currentRotationRate = MathUtil.clamp(gyroPid.calculate(ahrs.getAngle()), -.3, .3);
          //drive.arcadeDrive(0,currentRotationRate);
          
          if (gyroPid.atSetpoint()) {
            autonConditionCompleted = true;
          }
          break;
        case NONE:
        default:
          drive.arcadeDrive(0,0);
          break;
      }
    
    }
    
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    climbRatchet.set(0);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
    driveTrain(gp1.getRightTriggerAxis()-gp1.getLeftTriggerAxis(), gp1.getLeftX());
    controlIntakeShooterIndex( gp1.getXButton(), gp1.getYButton(), gp2.getRightTriggerAxis()> .5, gp2.getRightBumperPressed(), gp2.getLeftBumperPressed());    
    

    controlUppeyDowney(gp2.getBButton(), gp2.getXButton());

    climb(gp2.getYButton(), gp2.getAButton(), gp2.getPOV(), gp2.getLeftStickButtonPressed());
   
    if(gp2.getRightStickButtonPressed())
      shooterSetAngle(servoAngle);

    if(gp2.getStartButtonPressed())
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
    gp1 = new XboxController(Statics.XboxController1_ID);
    gp2 = new XboxController(Statics.XboxController2_ID);
  }

  private void initializeMotorControllers() {   
   
    shooter = new WPI_TalonFX(Statics.Shooter_Motor_ID);
    intake = new Spark(Statics.Intake_Motor_ID);
    index = new Spark(Statics.Index_Motor_ID);
    intakeUpDown = new Spark(Statics.Intake_Up_Down_Motor_ID);
    LED = new Spark(Statics.ledControllerID);
    
    fl_drive = new CANSparkMax(Statics.Front_Left_Motor_ID, MotorType.kBrushless);
    fr_drive = new CANSparkMax(Statics.Front_Right_Motor_ID, MotorType.kBrushless);
    bl_drive = new CANSparkMax(Statics.Back_Left_Motor_ID, MotorType.kBrushless);
    br_drive = new CANSparkMax(Statics.Back_Right_Motor_ID, MotorType.kBrushless);

    verticalClimb = new WPI_TalonFX(Statics.Vertical_Climb_Motor_ID);
    horizontalClimb = new Spark(Statics.Horizontal_Climb_Motor_ID);
    climbRatchet = new Servo(Statics.Climb_Ratchet_ID);
    
  }

  private void driveTrain(double power, double turn) {
    drive.arcadeDrive(Statics.Drive_Speed*cubicScaledDeadband(power, Statics.deadbandCutoff, Statics.Weight),
                      Statics.Turn_Speed*cubicScaledDeadband(turn, Statics.deadbandCutoff, Statics.Weight));
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
      return (cubic(x, weight)- (Math.abs(x)/x) * cubic(deadbandCutoff, weight)) / (1.0 - cubic(deadbandCutoff, weight));
    }
  }

  public void climb(boolean up, boolean down, int horizontalDirection, boolean ratchetButton){
    if (up){
      verticalClimb.set(Statics.Vertical_Climb_Speed);
    } else if (down) {
      verticalClimb.set(-Statics.Vertical_Climb_Speed);
    } else {
      verticalClimb.set(0);
    }
    
    if (horizontalDirection == 0){
      horizontalClimb.set(Statics.Horizontal_Climb_Speed);
    } else if (horizontalDirection == 180){
      horizontalClimb.set(-Statics.Horizontal_Climb_Speed);
    } else {
      horizontalClimb.set(0);
    }

    if (ratchetButton && climbRatchet.get() == 0){
      climbRatchet.set(1);
    } else if (ratchetButton && climbRatchet.get() == 1){
      climbRatchet.set(0);
    }

  }
  public void updateNetworkEntries(){
    pdpVoltage.setDouble(pdpNum);
    rightMotorNetworkTable.setDouble(rightMotorN);
    leftMotorNetworkTable.setDouble(leftMotorN);

    IndexTable.setDouble(indexN);
    ShooterTable.setDouble(shooterN);
    IntakeTable.setDouble(intakeN);

    shooterRPMEntry.setDouble(shooterRPM);
    if (ratchetPos > 45)
      ratchetEngaged.setBoolean(true);
    else
      ratchetEngaged.setBoolean(false);
    
    //navXEntry.setDouble(navXAngle)
  }
  public void updateInputs(){
    //pdpNum = pdp.getVoltage();
    leftMotorN = fl_drive.get();
    rightMotorN = fr_drive.get();

    shooterN = shooter.get();
    indexN = index.get();
    intakeN = intake.get();
    ratchetPos = climbRatchet.getAngle();

    //shooterRPM = 0;
    shooter.getSelectedSensorVelocity();
    
    //navXAngle = navX.getAngle();
  }

  
  public void controlUppeyDowney(boolean upIntake, boolean downIntake){
    if(downIntake){
      if(!bottomLimitSwitchIntake.get()){
        intakeUpDown.set(0);
      }
      else{
        intakeUpDown.set(-Statics.IntakeUppeyDowneySpeed);
       
      }
    }
    else if(upIntake){
      if(!topLimitSwitchIntake.get()){
        intakeUpDown.set(0);
        
      }
      else{ 
        intakeUpDown.set(Statics.IntakeUppeyDowneySpeed);
        System.out.println(topLimitSwitchIntake.get());
        System.out.println(intakeUpDown.get());
      }
    }
    else{
      intakeUpDown.set(0);
    }
  }


  public void controlIntakeShooterIndex( boolean reverseIntake, boolean forwardIntake, boolean shoot, boolean raiseSpeed, boolean lowerSpeed){
        
      if (forwardIntake)
        intake.set(Statics.Intake_Speed);
      else if (reverseIntake)
        intake.set(-Statics.Intake_Speed);
      else 
        intake.set(0);
   
      
      if (Math.abs(shooter.getSelectedSensorVelocity()) > Statics.Shooter_Target_RPM){
        index.set(Statics.Index_Speed);
      }
      else if (forwardIntake && !scanForBalls()){
        index.set(Statics.Index_Speed);
      }
      else{
        index.set(0);
      }



      if(shoot) {
        shooter.set(shooterSpeed);
       //System.out.println(shooter.getSelectedSensorVelocity());
        
        }
       else {
        shooter.set(0);
        
      }
      
      if (raiseSpeed){
        shooterSpeed += 0.05;
      }
      if (lowerSpeed){
        shooterSpeed -= 0.05;
      } 
  }
  
 
  public boolean scanForBalls(){
    return Math.pow(ballIRSensor.getAverageVoltage(), -1.2045) * 27.726 < 50;
  }

 
    
  public void shooterSetAngle(double previousAngle){
    
    if (previousAngle == 0) {
      angleAdjuster.setAngle(45);
      servoAngle = 45;
    } else if (previousAngle == 45){
      angleAdjuster.setAngle(90);
      servoAngle = 90;
    }
    else if (previousAngle == 90){
      angleAdjuster.setAngle(0);
      servoAngle = 0;
      }
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

    ratchetEngaged = compTab.add("Ratchet", true)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .withSize(1,1)
    .withPosition(3,3)
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

  private void setAuton(AutonMode mode, String[] targetValue) {
    currentAuton = mode;
    autonTarget = Double.parseDouble(targetValue[0]);

    switch (mode) {
      case DRIVE:
        autonStartingPos = getAverageEncoderDistance();
        movePid.setSetpoint(Double.parseDouble(targetValue[0]) + (autonStartingPos/Statics.SensorToMeters));
        System.out.println("target value" + Double.parseDouble(targetValue[0]));
        System.out.println("Starting position"+ autonStartingPos);
        break;
      case TURN:
        //ahrs.reset();
        gyroPid.setSetpoint(Double.parseDouble(targetValue[0]));
        break;
      case SHOOT:
        //shoot code
        break;
      case DEPLOYINTAKE:
        //deploy the intake by applying speed to the motor
        break;
      case INTAKEON:

        autonConditionCompleted = true;
        break;
      case INTAKEOFF:

        autonConditionCompleted = true;
        break;
      case WAIT:
        //start a timer
        //auton condition completed when timer reaches desired time
        break;
      case NONE:
      default:
        //just if there's nothing else to do
        break;
    }
  }

  //setAuton([);

  /* gAED:
    - Gets the average encoder distance in each of the main drive motors
    - Returns a value in sensor units and must be converted (could change later)
  */
  private double getAverageEncoderDistance() {
    return (fl_drive.getEncoder().getPosition() + fr_drive.getEncoder().getPosition() + bl_drive.getEncoder().getPosition() + br_drive.getEncoder().getPosition())/4;
  }
  
}
