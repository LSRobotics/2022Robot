// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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


  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";

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
  private static final int P = 1;
  private static final int I = 0;
  private static final int D = 0;

  private int autoIncrement;

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private AHRS ahrs;

  private enum AutonMode {
    DRIVE, 
    TURN
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


    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    movePid = new PIDController(P,I,D); //TODO: figure out the kP, kI, and kD values required for actual instantiation

    ahrs = new AHRS(SPI.Port.kMXP);

  }


  @Override
  public void robotPeriodic() {
    drive.arcadeDrive(gp.getRightTriggerAxis()-gp.getLeftTriggerAxis(), gp.getLeftX());
  }

  /**
   * You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    autoIncrement = 0;
    autonConditionCompleted = false;
    currentAuton = AutonMode.DRIVE;
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto: //Development Auton

        //When each step of autonomous is completed
        if (autonConditionCompleted) {
          autoIncrement++;
          

          /* AUTO INCREMENT [PLACE AUTON INSTRUCTIONS HERE]
            - each switch case is another instruction
            - currently it is a switch statement and not an array to allow for some alternative functions to be called other than `set auton`
            - SetAuton(AutonMode, targetValue) is the main function being used currently.
              - Current AutonModes are `DRIVE` and `TURN`
              - the targetValue is the value whatever the specific AutonMode is measuring should reach
          */
          switch (autoIncrement) {
            case 0:
              setAuton(AutonMode.DRIVE, 3);
              break;
            case 1:
              setAuton(AutonMode.TURN, 5);
              break;
          }

          autonConditionCompleted = false;
        }

        switch (currentAuton) {
          /* DRIVE MODE
           - Drives forward some distance in **INSERT**UNITS**HERE**
           - Uses the ahrs in order to ensure the robot drives straight
          */
          case DRIVE: 
            double error = ahrs.getAngle();
            double turn = error;
            drive.arcadeDrive(movePid.calculate(getAverageEncoderDistance()-autonStartingPos), turn); //TODO: divide `getAverageEncoderDistance()-autonStartingPos` by the sensor units to actual units constant

            if (movePid.atSetpoint()) {
              autonConditionCompleted = true;
            }
            break;
          /* TURN MODE 
           - Turns some distance in degrees
          */
          case TURN: 
            double currentRotationRate = MathUtil.clamp(gyroPid.calculate(ahrs.getAngle()), -1.0, 1.0);
            drive.arcadeDrive(0,currentRotationRate);

            if (gyroPid.atSetpoint()) {
              autonConditionCompleted = true;
            }
            break;
        }

        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

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
  }

  private void setAuton(AutonMode mode, double targetValue) {

    currentAuton = mode;
    autonTarget = targetValue;
    ahrs.reset();

    switch (mode) {
      case DRIVE:
        movePid.setSetpoint(targetValue);
        autonStartingPos = getAverageEncoderDistance();
        break;
      case TURN:
        gyroPid.setSetpoint(targetValue);
        break;
    }

  }

  /* gAED:
    - Gets the average encoder distance in each of the main drive motors
    - Returns a value in sensor units and must be converted (could change later)
  */
  private double getAverageEncoderDistance() {
    double average;
    average = (fl_drive.getSelectedSensorPosition() + fr_drive.getSelectedSensorPosition() + bl_drive.getSelectedSensorPosition() + br_drive.getSelectedSensorPosition())/4;
    
    return average;
  }

}
