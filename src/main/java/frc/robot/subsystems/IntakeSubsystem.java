package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    public final TalonFX intakeMotor = new TalonFX(52, "rio");

    public enum Direction {
        FORWARD,
        BACKWARDS,
        NONE
    }

    private Direction direction = Direction.NONE;
    private double wheelSpeed = 3;

    private double getMultiplier(Direction direction) {
        switch (direction) {
            case FORWARD:
                return 1.0;
            case BACKWARDS:
                return -1.0;
            case NONE:
            default:
                return 0.0;
        }
    }

    public void runIntake(Direction state) {
        direction = state;
    }

    @Override
    public void periodic() {
        double outputVoltage = wheelSpeed * getMultiplier(direction);
        intakeMotor.setControl(new VoltageOut(outputVoltage));
        wheelSpeed = 0;
    }
}