package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Utilities.AutoFromPathPlanner;
import frc.robot.commands.SmartShooter;
import frc.robot.commands.RunIntake;
import frc.robot.commands.SmartFeed;
import frc.robot.subsystems.Swerve.Drivetrain;
import frc.robot.subsystems.*;

public class fiveball extends SequentialCommandGroup {
    
    private final Drivetrain m_drive;
    private final Intake m_intake;
    private final Shooter m_shooter;
    private final ShooterHood m_hood;
    private final Elevator m_top;
    private final XboxController m_operator;

    public fiveball(Drivetrain drive, Intake intake, Shooter shooter, ShooterHood hood, Elevator top, XboxController operator) {
        m_drive = drive;
        m_intake = intake;
        m_shooter = shooter;
        m_hood = hood;
        m_operator = operator;
        m_top = top;

        final AutoFromPathPlanner fiveballone = new AutoFromPathPlanner(m_drive, "5-ball-1", DriveConstants.kMaxSpeedMetersPerSecond, true);
        final AutoFromPathPlanner fiveballtwo = new AutoFromPathPlanner(m_drive, "5-ball-2", DriveConstants.kMaxSpeedMetersPerSecond, true);
        final AutoFromPathPlanner fiveballthree = new AutoFromPathPlanner(m_drive, "5-ball-3", DriveConstants.kMaxSpeedMetersPerSecond, true);

        final SmartShooter shoot1 = new SmartShooter(m_shooter, m_drive, m_hood, true);
        final SmartShooter shoot2 = new SmartShooter(m_shooter, m_drive, m_hood, true);

        final RunIntake runintake = new RunIntake(m_intake);

        final SmartFeed feed1 = new SmartFeed(m_top, m_drive, m_shooter, m_hood, m_operator);
        final SmartFeed feed2 = new SmartFeed(m_top, m_drive, m_shooter, m_hood, m_operator);

        addCommands(
            new InstantCommand(() -> m_drive.resetOdometry(fiveballone.getInitialPose())),
            
            new ParallelCommandGroup(
                runintake,
                new SequentialCommandGroup(
                    fiveballone,
                    shoot1.raceWith(new WaitCommand(0.5).andThen(feed1.withTimeout(1.5))),
                    fiveballtwo.andThen(new WaitCommand(3.0)),
                    fiveballthree,
                    shoot2.raceWith(new WaitCommand(0.5).andThen(feed2.withTimeout(1.5)))
                )
            )
        );

    }

}
