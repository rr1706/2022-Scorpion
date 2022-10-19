package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;

public class SimpleShooter extends CommandBase {
    private final Shooter m_shooter;
    private final ShooterHood m_hood;
    private final double m_RPM;
    private final double m_angle;

    public SimpleShooter(Shooter shooter, ShooterHood hood, double rpm, double angle) {
        m_shooter = shooter;
        m_hood = hood;
        m_RPM = rpm;
        m_angle = angle;
        addRequirements(m_shooter,m_hood);
    }

    @Override
    public void initialize() {
        m_shooter.run(m_RPM);
        m_hood.run(m_angle);
    }

    @Override
    public void execute() {
        m_shooter.run(m_RPM);
        m_hood.run(m_angle);
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
        m_hood.stop();
    }

}
