package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer.States;

public class HopperSubsystem extends SubsystemBase {
    public final TalonFX cimberMotor = new TalonFX(50, "rio");



    private States state = States.NONE;
    private double hopperSpeed;

    public void runHopper(States direction) {
        hopperSpeed = 4;
        state = direction;
    }

    @Override
    public void periodic() {
        double outputVoltage = -1 * hopperSpeed * state.getMultiplier();
        cimberMotor.setControl(new VoltageOut(outputVoltage));
        hopperSpeed = 0;
    }
}