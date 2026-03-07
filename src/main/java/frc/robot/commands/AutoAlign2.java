package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoAlign2 extends Command{
    private PIDController rotController;
    

    private boolean isAtRot = false;

    public AutoAlign2(){
        
    }
}
