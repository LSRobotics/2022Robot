// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public class Statics {
    public final static int Front_Left_Motor_ID = 1;
    public final static int Front_Right_Motor_ID = 2;
    public final static int Back_Left_Motor_ID = 3;
    public final static int Back_Right_Motor_ID = 4;

    public final static int Vertical_Climb_Motor_ID = 11;
    public final static int Horizontal_Climb_Motor_ID = 8; 
    public final static int Climb_Ratchet_ID = 3;

    public final static double Limelight_Mount_Angle_Degrees = 25.0;
    public final static double LimeLight_Height_Inches = 25.0;
    public final static double Goal_Height_Inches = 102.0;
    
    
    public final static double Vertical_Climb_Speed = 0.5;
    public final static double Horizontal_Climb_Speed = 0.5;

    public final static double Index_Speed = -1;
    public final static double Intake_Speed = 1;

    public final static double shooting_threshold_value = .5;

    public final static double IntakeUppeyDowneySpeed = 0.9;

    //Just for now
    public final static double Shooter_Target_RPM = 18000;
    public final static double Shooter_Speed = 0.5;
    
    public final static double Fast_Drive_Speed = 0.8;
    public final static double Slow_Drive_Speed = 0.6
    ;
    public final static double Fast_Turn_Speed = 0.7;
    public final static double Slow_Turn_Speed = 0.4;
  
    //weight for cubic function
    public final static double Weight = 0.5;
    public final static double deadbandCutoff = 0.1;

    public final static int Shooter_Motor_ID = 5;
    public final static int Index_Motor_ID = 0;
    public final static int Intake_Motor_ID = 2; 
    public final static int Intake_Up_Down_Motor_ID = 7;
    public final static int ledControllerID = 6;


    public final static int XboxController1_ID = 0;
    public final static int XboxController2_ID = 1;
    
    public static double cm_to_in = 0.049212598;
    
    public final static double SensorToMeters = 22;

    public final static double movementPIDp = 1.2;
    public final static double movementPIDi = .3;
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



