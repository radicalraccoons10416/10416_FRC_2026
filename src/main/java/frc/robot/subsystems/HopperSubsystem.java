package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer.States;

public class HopperSubsystem extends SubsystemBase {
    // TODO: Change ID
    public final TalonFX cimberMotor = new TalonFX(100, "rio");



    private States state = States.NONE;
    private double wheelSpeed;

    public void runClimber(States direction) {
        wheelSpeed = 1;
        state = direction;
    }

    @Override
    public void periodic() {
        double outputVoltage = wheelSpeed * state.getMultiplier();
        cimberMotor.setControl(new VoltageOut(outputVoltage));
        wheelSpeed = 0;
    }
}