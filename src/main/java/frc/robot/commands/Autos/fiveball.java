package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Utilities.AutoFromPathPlanner;
import frc.robot.commands.AimAtTarget;
import frc.robot.subsystems.Swerve.Drivetrain;
import frc.robot.subsystems.Intake;

public class fiveball extends SequentialCommandGroup {
    
    private final Drivetrain m_drive;
    private final Intake m_intake;

    public fiveball(Drivetrain drive, Intake intake) {
        m_drive = drive;
        m_intake = intake;

        final AutoFromPathPlanner fiveballone = new AutoFromPathPlanner(m_drive, "5-ball-1", DriveConstants.kMaxSpeedMetersPerSecond);
        final AutoFromPathPlanner fiveballtwo = new AutoFromPathPlanner(m_drive, "5-ball-2", DriveConstants.kMaxSpeedMetersPerSecond);
        final AutoFromPathPlanner fiveballthree = new AutoFromPathPlanner(m_drive, "5-ball-3", DriveConstants.kMaxSpeedMetersPerSecond);

        final AimAtTarget aim1 = new AimAtTarget(m_drive, driveController, targetLocation, MoveAndShootComp)

    }

}
