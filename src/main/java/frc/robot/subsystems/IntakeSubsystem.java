package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer.States;

public class IntakeSubsystem extends SubsystemBase {
    // TODO: Change ID
    public final TalonFX intakeMotor = new TalonFX(52, "rio");
    public final TalonFX storeMotor = new TalonFX(53, "rio");

    private States state = States.NONE;
    private double wheelSpeed = 0.0;

    public IntakeSubsystem(){
        var slot0Configs = new Slot0Configs();

        slot0Configs.kP = 0;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;

        storeMotor.getConfigurator().apply(slot0Configs);
        storeMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setPosition(double pos){
        final PositionTorqueCurrentFOC m_Request = new PositionTorqueCurrentFOC(pos).withSlot(0);

        storeMotor.setControl(m_Request);
    }

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