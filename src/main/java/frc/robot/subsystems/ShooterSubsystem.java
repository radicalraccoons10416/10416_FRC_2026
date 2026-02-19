package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    // TODO: change ID
    public final TalonFX shooterMotor = new TalonFX(100, "rio");
    double wheelSpeed = 0;
    /**
     * Set the shooter state. Use states.FORWARD / BACKWARDS / NONE.
     */
    public void runShooter() {
        wheelSpeed = -8 ;
    }

    public void stopShooter() {
        wheelSpeed = 0;
    }

    @Override
    public void periodic() {
        shooterMotor.setControl(new VoltageOut(wheelSpeed));
    }
}