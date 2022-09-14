package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Utilities.LinearInterpolationTable;

import java.awt.geom.Point2D;

  /**
   * Static method containing all constant values for the robot in one location
   */
public final class Constants {

  public static final class CurrentLimit{
    public static final int kIntake = 25;
    public static final int kTurret = 30;
    public static final int kShooter = 40;
    public static final int kHood = 20;
    public static final int kElevator = 30;
    public static final int kTranslation = 30;
    public static final int kRotation = 25;
    public static final int kClimber = 80;
  }

  public static final class GoalConstants{
    public static final Translation2d kGoalLocation = new Translation2d(8.23,4.115);
    public static final Translation2d kWrongBallGoal = new Translation2d(5.50,4.115);
    public static final Translation2d kHangerLocation = new Translation2d(2.00,6.00);

  }

  /**
   * Static method containing all Drivetrain constants 
   */
  public static final class DriveConstants {
    public static final int kFrontLeftDriveMotorPort = 5;   //CANID of the Translation SparkMAX
    public static final int kFrontRightDriveMotorPort = 7;  //CANID of the Translation SparkMAX
    public static final int kBackLeftDriveMotorPort = 3;    //CANID of the Translation SparkMAX
    public static final int kBackRightDriveMotorPort = 1;   //CANID of the Translation SparkMAX

    public static final int kFrontLeftTurningMotorPort = 6;   //CANID of the Rotation SparkMAX
    public static final int kFrontRightTurningMotorPort = 8;  //CANID of the Rotation SparkMAX
    public static final int kBackLeftTurningMotorPort = 4;    //CANID of the Rotation SparkMAX
    public static final int kBackRightTurningMotorPort = 2;   //CANID of the Rotation SparkMAX

    public static final int kFrontLeftTurningEncoderPort = 2;   //Analog Port of the Module Absolute Encoder
    public static final int kFrontRightTurningEncoderPort = 1;  //Analog Port of the Module Absolute Encoder
    public static final int kBackLeftTurningEncoderPort = 3;    //Analog Port of the Module Absolute Encoder
    public static final int kBackRightTurningEncoderPort = 0;   //Analog Port of the Module Absolute Encoder

    public static final double kFrontLeftOffset = -3.813+2*Math.PI;  //Encoder Offset in Radians
    public static final double kFrontRightOffset = -1.351+Math.PI;  //Encoder Offset in Radians
    public static final double kBackLeftOffset = -2.711+Math.PI;   //Encoder Offset in Radians
    public static final double kBackRightOffset = -3.715+Math.PI;  //Encoder Offset in Radians

    //Drive motor PID is best done on the roboRIO currently as the SparkMAX does not allow for static gain values on the PID controller, 
    //    these are necessary to have high accuracy when moving at extremely low RPMs
    //public static final double[] kFrontLeftTuningVals   =   {0.0120,0.2892,0.25,0};   //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}
    //public static final double[] kFrontRightTuningVals  =   {0.0092,0.2835,0.25,1};   //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}
    //public static final double[] kBackLeftTuningVals    =   {0.0142,0.2901,0.25,2};   //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}
    //public static final double[] kBackRightTuningVals   =   {0.0108,0.2828,0.25,3};   //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}

    public static final double[] kFrontLeftTuningVals   =   {0.01,0.2850,0.2,0};   //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}
    public static final double[] kFrontRightTuningVals  =   {0.01,0.2850,0.2,1};   //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}
    public static final double[] kBackLeftTuningVals    =   {0.01,0.2850,0.2,2};   //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}
    public static final double[] kBackRightTuningVals   =   {0.01,0.2850,0.2,3};   //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}

    public static final Translation2d kFrontLeftLocation = new Translation2d(0.327,0.240);
    public static final Translation2d kFrontRightLocation = new Translation2d(0.314,-0.252);
    public static final Translation2d kBackLeftLocation = new Translation2d(-0.314,0.252);
    public static final Translation2d kBackRightLocation = new Translation2d(-0.327,-0.240);
     
    //Because the swerve modules poisition does not change, define a constant SwerveDriveKinematics for use throughout the code
    public static final SwerveDriveKinematics kDriveKinematics 
      = new SwerveDriveKinematics(kFrontLeftLocation,kFrontRightLocation,kBackLeftLocation,kBackRightLocation);

    public static final double kMaxAcceleration = 4.00;
    public static final double kMaxSpeedMetersPerSecond = 3.5; //Maximum Sustainable Drivetrain Speed under Normal Conditions & Battery, Robot will not exceed this speed in closed loop control
    public static final double kMaxAngularSpeed = 4.00;      //Maximum Angular Speed desired. NOTE: Robot can exceed this but spinning fast is not particularly useful or driver friendly
    public static final double kMaxAngularAccel = 8.00;      //Maximum Angular Speed desired. NOTE: Robot can exceed this but spinning fast is not particularly useful or driver friendly

    public static final double kInnerDeadband = 0.10; //This value should exceed the maximum value the analog stick may read when not in use (Eliminates "Stick Drift")
    public static final double kOuterDeadband = 0.98; //This value should be lower than the analog stick X or Y reading when aimed at a 45deg angle (Such that X and Y are are maximized simultaneously)
  
    //Minimum allowable rotation command (in radians/s) assuming user input is squared using quadraticTransform, this value is always positive and should be compared agaisnt the absolute value of the drive command
    public static final double kMinRotationCommand = DriveConstants.kMaxAngularSpeed * Math.pow(DriveConstants.kInnerDeadband,2);
    //Minimum allowable tranlsation command (in m/s) assuming user input is squared using quadraticTransform, this value is always positive and should be compared agaisnt the absolute value of the drive command
    public static final double kMinTranslationCommand = DriveConstants.kMaxSpeedMetersPerSecond * Math.pow(DriveConstants.kInnerDeadband,2);

    public static final double[] kKeepAnglePID = { 0.700, 0, 0 }; //Defines the PID values for the keep angle PID

  }
  /**
   * Static method containing all Swerve Module constants 
   */
  public static final class ModuleConstants {
    public static final double kTranslationRampRate = 3.0;          //Units of %power/s, ie 4.0 means it takes 0.25s to reach 100% power from 0%
    private static final double kTranslationGearRatio = 5.6111111; //Overall gear ratio of the swerve module
    private static final double kWheelDiameter = 0.0777;           //Wheel Diameter in meters, may need to be experimentally determined due to compliance of floor/tread material

    public static final double kVelocityFactor = (1.0 / kTranslationGearRatio / 60.0) * kWheelDiameter * Math.PI; //Calculates the conversion factor of RPM of the translation motor to m/s at the floor

    //NOTE: You shoulds ALWAYS define a reasonable current limit when using brushless motors 
    //      due to the extremely high stall current avaialble

    public static final double[] kTurnPID = { 0.600, 0, 0 }; //Defines the PID values for rotation of the serve modules, should show some minor oscillation when no weight is loaded on the modules
  }
  /**
   * Static method containing all User I/O constants 
   */
  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;    //When making use of multiple controllers for drivers each controller will be on a different port
    public static final int kOperatorControllerPort = 1;  //When making use of multiple controllers for drivers each controller will be on a different port
  }

  /**
   * Static method containing all Global constants 
   */
  public static final class GlobalConstants {
    public static final double kVoltCompensation = 12.6;        //Sets a voltage compensation value ideally 12.6V
    public static final int PCHID = 20;
    public static final int PDHID = 24;
    public static final double kLoopTime = 0.020;
  }
  /**
   * Static method containing all Vision/Limelight constants 
   */
  public static final class VisionConstants {
    public static final double kElevationOffset =38.5;              // Degree offset of lens from horizontal due to camera mount
    public static final double kAzimuthalAngle = -0.50;                // Degree azimuthal offset of limelight
    public static final double kTargetCenterHeightFromLens = 81.0;  // Center Height of the Target in inches above the lens
    public static final double kTrackTolerance = 0.0200;             // Allowable Limelight angle error in radians
  }
  /**
   * Static method containing all Intake constants 
   */
  public static final class IntakeConstants {
    public static final int kLeftMotorID = 8;
    public static final int kRightMotorID = 5;
    public static final int[] kLeftAirPorts = {3,2};
    public static final int[] kRightAirPorts = {1,0};
    public static final double []kPIDF = {0.00005,0,0,0.000091};
  }

  public static final class ElevatorConstants {
    public static final int kLowMotorID = 17;
    public static final int kHighMotorID = 16;
    public static final double []kPIDF = {0.000075,0,0,0.000091};
    public static final int kLowSensor = 10;
    public static final int kHighSensor = 11;
  }

  public static final class TurretConstants {
    public static final int kTurretPort = 14;                    //CANID of the turret motor controller
    public static final int kTurretPotentiometerPort = 4;       //Analog port of the turret analog potentiometer
    public static final double kTurretTolerance = 2*0.0349;    //allowable angle error in radians for the PIDSubsystem to report atSetpoint() to true
    public static final double[] kTurretPID = { 4.0, 0.0, 0 };  //Defines the PID values for rotation of the turret
    public static final double kStaticGain = 0.025;             //Limits Integral term so as to not wind up values when making larger moves
    public static final double kTurretLow = 0.50;               //Minimum angle in radians allowed (defines the turret deadzone)
    public static final double kTurretHigh = 5.78;              //Maximum angle in radians allowed (defines the turret deadzone)
  }

  public static final class ClimberConstants {
    public static final int[] kMotorID = {15,9};
    public static final int[] kValvePorts = {4,5};
  }

  public static final class HoodConstants{
    public static final int kMotorID = 11;
    public static final double kHoodTolerance = 5.0;
  }

  /**
   * Static method containing all Shooter constants 
   */
  public static final class ShooterConstants {
    public static final int[] kMotorIDs = {12,13};        //CANID of the Motor Controller for the Sooter Motor
    public static final double kShotRPMTolerance = 200.0;          //RPMs of error allowed before a ball can be fed into t he shooter
    public static final double[] kPID = { 0.00005, 0.0003, 0 };         //Defines PID values for the shooter 0.00045
    public static final double kShooterFF = 0.018;            //Defines shooter FeedForward Value, should be roughly equal to 1/MaxMotorRPM * MaxRPMVoltage / Compensation Voltage
    public static final double kStaticGain = 0.0001635;
    public static final double kAccelCompFactor = 0.100; //in units of seconds


    private static final Point2D[] khoodPoints = 
        new Point2D.Double[]{
            //(ty-angle,distance)
            new Point2D.Double(25,0.0),
            new Point2D.Double(35,0.0),
            new Point2D.Double(75,10.0),//
            new Point2D.Double(90,15.5),//
            new Point2D.Double(105,18.4),//
            new Point2D.Double(120,23.0),//
            new Point2D.Double(135,26.0),//
            new Point2D.Double(150,28.0),//
            new Point2D.Double(165,30.5),//
            new Point2D.Double(180,32.0),//
            new Point2D.Double(195,36.0),//
            new Point2D.Double(210,37.0),//
            new Point2D.Double(225,38.0)//
        };
    public static final LinearInterpolationTable khoodTable = new LinearInterpolationTable(khoodPoints);

    private static final Point2D[] krpmPoints = 
        new Point2D.Double[]{
            //(ty-angle,distance)
            new Point2D.Double(25,1500),
            new Point2D.Double(35,1500),
            new Point2D.Double(75,2350),//
            new Point2D.Double(90,2460),//
            new Point2D.Double(105,2515),//
            new Point2D.Double(120,2620),//
            new Point2D.Double(135,2715),//
            new Point2D.Double(150,2815),//
            new Point2D.Double(165,2900),//
            new Point2D.Double(180,3050),//
            new Point2D.Double(195,3195),//
            new Point2D.Double(210,3315),//
            new Point2D.Double(225,3450),//
            new Point2D.Double(350,3730)
        };

    public static final LinearInterpolationTable krpmTable = new LinearInterpolationTable(krpmPoints);
  }
  
    /**
   * Static method containing all Autonomous constants 
   */
  public static final class AutoConstants {
    public static final double kMaxAcceleration = 2.50;
    public static final double kMaxSpeed = 3.25; //Maximum Sustainable Drivetrain Speed under Normal Conditions & Battery, Robot will not exceed this speed in closed loop control
    public static final double kMaxAngularSpeed = Math.PI;      //Maximum Angular Speed desired. NOTE: Robot can exceed this but spinning fast is not particularly useful or driver friendly
    public static final double kMaxAngularAccel = Math.PI;      //Maximum Angular Speed desired. NOTE: Robot can exceed this but spinning fast is not particularly useful or driver friendly

    public static final double kPXController = 3.0;
    public static final double kPYController = 3.0;
    public static final double kPThetaController = 3.0;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeed, kMaxAngularAccel); //Creates a trapezoidal motion for the auto rotational commands
  }
}