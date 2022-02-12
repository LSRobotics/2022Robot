// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI; //TODO: change the port system depending on what we actually use

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

  private PIDController movePid;
  private PIDController gyroPid;
  private static final int mP = 1; //TODO: move these to statics file
  private static final int mI = 0;
  private static final int mD = 0;

  private static final int gP = 1;
  private static final int gI = 0;
  private static final int gD = 0;

  private Timer debugTimer;

  private int autoIncrement;

  private AHRS ahrs;

  private enum AutonMode {
    DRIVE, 
    TURN,
    NONE
  }

  private AutonMode currentAuton;

  private boolean autonConditionCompleted;

  private double autonTarget;

  private double autonStartingPos;

  /*private Object[][] AutonInstructions = {
    {AutonMode.DRIVE, 10}
  };      // this could be cool but is not practical right now*/


  @Override
  public void robotInit() {
    initializeMotorControllers();

    initializeGamePad();

    left_motors = new MotorControllerGroup(fl_drive, bl_drive);
    right_motors = new MotorControllerGroup(fr_drive, br_drive);
    drive = new DifferentialDrive(left_motors, right_motors);

    movePid = new PIDController(mP,mI,mD); //TODO: figure out the kP, kI, and kD values required for actual instantiation
    gyroPid = new PIDController(gP,gI,gD); //TODO: figure out the kP, kI, and kD values required for actual instantiation

    ahrs = new AHRS(SPI.Port.kMXP);

    debugTimer = new Timer();
  }


  @Override
  public void robotPeriodic() {
  }

  /**
   * You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    autoIncrement = -1;
    autonConditionCompleted = true;
    currentAuton = AutonMode.NONE;
    debugTimer.reset();
    debugTimer.start();
  }

  @Override
  public void autonomousPeriodic() {
    //When each step of autonomous is completed
    drive.arcadeDrive(0, 0);
    
    if (autonConditionCompleted) {
      autoIncrement++;

    
      // AUTO INCREMENT [PLACE AUTON INSTRUCTIONS HERE]
      // - each switch case is another instruction
      // - currently it is a switch statement and not an array to allow for some alternative functions to be called other than `set auton`
      // - SetAuton(AutonMode, targetValue) is the main function being used currently.
      // - Current AutonModes are `DRIVE` and `TURN`, with `NONE` being just to do nothing
      // - the targetValue is the value whatever the specific AutonMode is measuring should reach
      
      switch (autoIncrement) {
        case 0:
          setAuton(AutonMode.DRIVE, .5);
          break;
        case 1:
          setAuton(AutonMode.TURN, .5);
          break;
      }

      autonConditionCompleted = false;
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
          double rawValue = movePid.calculate(valueToCalculate);
          double driveValue = .4 * MathUtil.clamp(rawValue, -1, 1);
          drive.arcadeDrive(driveValue, 0); //TODO: divide `getAverageEncoderDistance()-autonStartingPos` by the sensor units to actual units constant
          if (movePid.atSetpoint()) {
            autonConditionCompleted = true;
          }
          break;
        // TURN MODE 
        // - Turns some distance in degrees
        case TURN: 
          double currentRotationRate = MathUtil.clamp(gyroPid.calculate(ahrs.getAngle()), -.3, .3);
          drive.arcadeDrive(0,currentRotationRate);

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

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    drive.arcadeDrive(gp.getRightTriggerAxis()-gp.getLeftTriggerAxis(), gp.getLeftX());

    if (gp.getAButton()) {
      System.out.println(getAverageEncoderDistance());
      System.out.println("Front left: " + fl_drive.getSelectedSensorPosition());
      System.out.println("Front right: " + fr_drive.getSelectedSensorPosition());
      System.out.println("Back left: " + bl_drive.getSelectedSensorPosition());
      System.out.println("Back right: " + br_drive.getSelectedSensorPosition());
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  private void initializeGamePad() {
    gp = new XboxController(Statics.XboxController_ID);
  }

  private static void initializeMotorControllers() {
    fl_drive = new WPI_TalonFX(Statics.Front_Left_Motor_ID);
    fr_drive = new WPI_TalonFX(Statics.Front_Right_Motor_ID);
    bl_drive = new WPI_TalonFX(Statics.Back_Left_Motor_ID);
    br_drive = new WPI_TalonFX(Statics.Back_Right_Motor_ID);

    fr_drive.setInverted(true);
    br_drive.setInverted(true);
  }

  private void setAuton(AutonMode mode, double targetValue) {


    /*
    currentAuton = mode;
    autonTarget = targetValue;

    switch (mode) {
      case DRIVE:
        movePid.reset();
        autonStartingPos = getAverageEncoderDistance();
        movePid.setSetpoint(targetValue + (autonStartingPos/Statics.SensorToMeters));
        break;
      case TURN:
        ahrs.reset();
        gyroPid.reset();
        gyroPid.setSetpoint(targetValue);
        break;
      case NONE:
      default:
        //just if there's nothing else to do
        break;
    }
    */

  }

  /* gAED:
    - Gets the average encoder distance in each of the main drive motors
    - Returns a value in sensor units and must be converted (could change later)
  */
  private double getAverageEncoderDistance() {
    return (fl_drive.getSelectedSensorPosition() + fr_drive.getSelectedSensorPosition() + bl_drive.getSelectedSensorPosition() + br_drive.getSelectedSensorPosition())/4;
  }

}
