// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


/** Add your docs here. */
public class Statics {
    public final static int Front_Left_Motor_ID = 1;
    public final static int Front_Right_Motor_ID = 2;
    public final static int Back_Left_Motor_ID = 3;
    public final static int Back_Right_Motor_ID = 4;
    public final static int ClimbMotor1ID = 11;
    public final static int ClimbMotor2ID = 12; 

    public final static double Index_Speed = .5;
    public final static double Intake_Speed = .5;
    public final static double IntakeUppeyDowneySpeed = 0.2;

    //Just for now
    public final static double Shooter_Target_RPM = 1;
    public final static double Shooter_Speed = 0.5;
    

    //weight for cubic function
    public final static double Weight = 0.5;
    public final static double deadbandCutoff = 0.1;

    public final static int Shooter_Motor_ID = 5;
    public final static int Index_Motor_ID = 0;
    public final static int Intake_Motor_ID = 1;
    public final static int Intake_Up_Down_Motor_ID = 2;


    public final static int XboxController1_ID = 0;
    public final static int XboxController2_ID = 1;
    
    public static int ultrasonic = 7; //ID has to be figured out
    public static double cm_to_in = 0.049212598;
    
    //TODO determine if `public static double inchToSensorUnits = 2148.15278;` did anything
    
    public final static double SensorToMeters = 45347.75; //TODO this is not right

    public final static double movementPIDp = 1;
    public final static double movementPIDi = 0;
    public final static double movementPidd = 0;

    public final static double gyroPIDp = 1;
    public final static double gyroPIDi = 0;
    public final static double gyroPIDd = 0;

    /*
    public static int navx = 10;
    public final static double PID_GYRO_TOLERANCE = 2;
    public final static double GYRO_P = .2;
    public final static double GYRO_I = 0;
    public final static double GYRO_D = .2;
    public final static double GYRO_F = 0;
    */
    

    
    
    //public static double stickDeadzone = 0.1;  Needed?
    //public static double moveSpeed = 0.5;         ^

    

    
}



