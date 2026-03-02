package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AutoAlign extends Command{
    private final SwerveSubsystem swerve;
    private final PIDController rController;

    public AutoAlign(SwerveSubsystem swerve){
        this.swerve = swerve;
        rController = new PIDController(0.1, 0, 0.1);
    }

    public void initialize(){

    }
}
