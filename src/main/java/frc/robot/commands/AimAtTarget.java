package frc.robot.commands;

import org.ejml.interfaces.decomposition.TridiagonalSimilarDecomposition;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Utilities.FieldRelativeAccel;
import frc.robot.Utilities.FieldRelativeSpeed;
import frc.robot.Utilities.LinearInterpolationTable;
import frc.robot.Utilities.MathUtils;
import frc.robot.subsystems.Swerve.Drivetrain;
import java.awt.geom.Point2D;

public class AimAtTarget extends CommandBase {
    private final Drivetrain m_drive;
    private final XboxController m_controller;
    private final PIDController m_rotPID 
        = new PIDController(2.75, 0, 0.100);

        private final SlewRateLimiter m_slewX = new SlewRateLimiter(2.5);
        private final SlewRateLimiter m_slewY = new SlewRateLimiter(2.5);

    private final Translation2d ktargetLocation;

    private final boolean m_moveComp;
    private boolean fieldOrient = true;


    private static Point2D[] m_shotTimes = 
        new Point2D.Double[]{
            //(ty-angle,time)
            new Point2D.Double(105,1.00), 
            new Point2D.Double(135,1.00), 
            new Point2D.Double(165,1.05),//
            new Point2D.Double(195,1.05),
            new Point2D.Double(250,1.20),
            ////
        };
    private static LinearInterpolationTable m_timeTable = new LinearInterpolationTable(m_shotTimes);


    public AimAtTarget(Drivetrain drive, XboxController driveController, Translation2d targetLocation, Boolean MoveAndShootComp){
        m_drive = drive;
        m_controller = driveController;
        ktargetLocation = targetLocation;
        m_rotPID.enableContinuousInput(0, 2*Math.PI);
        m_moveComp = MoveAndShootComp;
        addRequirements(m_drive);
    }
    
    @Override
    public void initialize(){
        m_rotPID.reset();
    }
    @Override
    public void execute(){
        Translation2d robotLocation = m_drive.getPose().getTranslation();
        Translation2d robotToGoal = ktargetLocation.minus(robotLocation);

if(m_moveComp){

        FieldRelativeSpeed robotVel = m_drive.getFieldRelativeSpeed();
        FieldRelativeAccel robotAccel = m_drive.getFieldRelativeAccel();

        double dist = robotToGoal.getDistance(new Translation2d())*39.37;

        SmartDashboard.putNumber("Calculated (in)", dist);

        double fixedShotTime = m_timeTable.getOutput(dist);

        double virtualGoalX = ktargetLocation.getX()-fixedShotTime*(robotVel.vx+robotAccel.ax*ShooterConstants.kAccelCompFactor);
        double virtualGoalY = ktargetLocation.getY()-fixedShotTime*(robotVel.vy+robotAccel.ay*ShooterConstants.kAccelCompFactor);

        Translation2d movingGoalLocation = new Translation2d(virtualGoalX, virtualGoalY);
        robotToGoal = movingGoalLocation.minus(robotLocation);

        SmartDashboard.putNumber("Goal X", virtualGoalX);
        SmartDashboard.putNumber("Goal Y", virtualGoalY);
}
        double targetAngle = Math.atan2(robotToGoal.getY(),robotToGoal.getX());
        double currentAngle = MathUtils.toUnitCircAngle(m_drive.getGyro().getRadians());
        double pidOutput = m_rotPID.calculate(currentAngle,targetAngle);

        if(pidOutput > DriveConstants.kMaxAngularSpeed){
            pidOutput = DriveConstants.kMaxAngularSpeed;
        }
        else if(pidOutput < -DriveConstants.kMaxAngularSpeed){
            pidOutput = -DriveConstants.kMaxAngularSpeed;
        }

        SmartDashboard.putNumber("Target Angle", targetAngle);
        SmartDashboard.putNumber("Current Angle", currentAngle);
        SmartDashboard.putNumber("PID Output", pidOutput);

        m_drive.drive(
        -m_slewX.calculate(inputTransform(m_controller.getLeftY()))
            * DriveConstants.kMaxSpeedMetersPerSecond/2.0,
          -m_slewY.calculate(inputTransform(m_controller.getLeftX()))
            * DriveConstants.kMaxSpeedMetersPerSecond/2.0,
            pidOutput,
        fieldOrient,
        false);
    }    
    @Override
    public void end(boolean interrupted){
        m_drive.updateKeepAngle();
    }

    private double inputTransform(double input){
        return MathUtils.singedSquare(MathUtils.applyDeadband(input));
      }
}
