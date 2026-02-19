package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer.States;

public class IntakeSubsystem extends SubsystemBase {
    // TODO: Change ID
    public final TalonFX intakeMotor = new TalonFX(52, "rio");

    private States state = States.NONE;
    private double wheelSpeed = 0.0;

    public void runIntake(States direction) {
        wheelSpeed = 2.0; // Set the speed when the trigger is pressed
        state = direction;
    }

    @Override
    public void periodic() {
        double outputVoltage = wheelSpeed * state.getMultiplier();
        intakeMotor.setControl(new VoltageOut(outputVoltage));
        state = States.NONE;
    }
}