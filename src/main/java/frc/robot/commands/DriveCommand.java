package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.XRPDrivetrain;

public class DriveCommand extends Command {
    
    DoubleSupplier m_xspeed;
    DoubleSupplier m_zrotation;
    XRPDrivetrain m_drivetrain;

    public DriveCommand(XRPDrivetrain drivetrain, DoubleSupplier xspeed, DoubleSupplier zrotation) {
        m_xspeed = xspeed;
        m_zrotation = zrotation;
        m_drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_drivetrain.arcadeDrive(m_xspeed.getAsDouble(), m_zrotation.getAsDouble());
    }

    @Override
    public void end(boolean isInterrupted) {
        m_drivetrain.arcadeDrive(0,0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
