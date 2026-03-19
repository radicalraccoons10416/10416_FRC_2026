package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer.States;
import java.util.function.BooleanSupplier;

public class HopperSubsystem extends SubsystemBase {
CANBus rio = new CANBus("rio");
    public final TalonFX cimberMotor = new TalonFX(50, rio);


    private States state = States.NONE;
    private double hopperSpeed;

    public Command runHopper(States direction, BooleanSupplier shooterReady) {
        return run(() -> {
            if (shooterReady.getAsBoolean()) {
                hopperSpeed = 4;
                state = direction;
            }
        });
    }

    /** Direct per-tick setter — call from another command's execute() instead of using runHopper(). */
    public void setHopper(States direction, BooleanSupplier shooterReady) {
        if (shooterReady.getAsBoolean()) {
            hopperSpeed = 4;
            state = direction;
        }
    }

    @Override
    public void periodic() {
        double outputVoltage = -1 * hopperSpeed * state.getMultiplier();
        cimberMotor.setControl(new VoltageOut(outputVoltage));

        hopperSpeed = 0;
    }
}