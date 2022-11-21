package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ClimbFromFloor extends CommandBase {
    private final Climber m_climber;

    public ClimbFromFloor(Climber climber) {
        m_climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        m_climber.setDesiredPose(20.0);
    }

    @Override
    public void execute() {
        m_climber.run();
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.setDesiredPose(5.0);
    }

}
