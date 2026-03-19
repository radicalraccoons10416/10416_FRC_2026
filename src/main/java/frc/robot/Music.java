package frc.robot;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

public class Music {
    CANBus rio = new CANBus("rio");
	private final TalonFX swerveBackLeftAngleMotor = new TalonFX(1, rio);
	private final TalonFX swerveBackLeftDriveMotor = new TalonFX(2, rio);
	private final TalonFX swerveFrontLeftAngleMotor = new TalonFX(3, rio);
	private final TalonFX swerveFrontLeftDriveMotor = new TalonFX(4, rio);
	private final TalonFX swerveFrontRightAngleMotor = new TalonFX(5, rio);
	private final TalonFX swerveFrontRightDriveMotor = new TalonFX(6, rio);
	private final TalonFX swerveBackRightAngleMotor = new TalonFX(7, rio);
	private final TalonFX swerveBackRightDriveMotor = new TalonFX(8, rio);
	private final TalonFX hopperMotor = new TalonFX(50, rio);
	private final TalonFX shooterLeftMotor = new TalonFX(51, rio);
	private final TalonFX shooterRightMotor = new TalonFX(52, rio);
	private final TalonFX shooterIntakeMotor = new TalonFX(53, rio);
	private final TalonFX storeMotor = new TalonFX(54, rio);
	private final TalonFX intakeMotor = new TalonFX(55, rio);
	private final TalonFX climberMotor = new TalonFX(56, rio);

	private final Orchestra orchestra = new Orchestra();
	private final String chrpFileName;
	private boolean trackLoaded = false;

	public Music(String chrpFileName) {
		this.chrpFileName = chrpFileName;
		List<TalonFX> instruments = List.of(
			swerveBackLeftAngleMotor,
			swerveBackLeftDriveMotor,
			swerveFrontLeftAngleMotor,
			swerveFrontLeftDriveMotor,
			swerveFrontRightAngleMotor,
			swerveFrontRightDriveMotor,
			swerveBackRightAngleMotor,
			swerveBackRightDriveMotor,
			hopperMotor,
			shooterLeftMotor,
			shooterRightMotor,
			shooterIntakeMotor,
			storeMotor,
			intakeMotor,
			climberMotor
		);
		configurePlaybackWhileDisabled(instruments);
		addInstruments(instruments);
		loadTrack();
	}

	public void addInstruments(List<TalonFX> motors) {
		for (TalonFX motor : motors) {
			orchestra.addInstrument(motor);
		}
	}

	public boolean loadTrack() {
		Path trackPath = Filesystem.getDeployDirectory().toPath().resolve("music").resolve(chrpFileName);
		if (!Files.exists(trackPath)) {
			DriverStation.reportError("CHRP file not found at " + trackPath + ". Put it in src/main/deploy/music/.", false);
			trackLoaded = false;
			return false;
		}

		StatusCode status = orchestra.loadMusic(trackPath.toString());
		trackLoaded = status.isOK();

		if (!trackLoaded) {
			DriverStation.reportError("Orchestra failed to load track from " + trackPath + " (" + status + ")", false);
		}

		return trackLoaded;
	}

	public void play() {
		if (!trackLoaded && !loadTrack()) {
			return;
		}

		StatusCode status = orchestra.play();
		if (!status.isOK()) {
			DriverStation.reportError("Orchestra play failed: " + status, false);
		}
	}

	public void pause() {
		StatusCode status = orchestra.pause();
		if (!status.isOK()) {
			DriverStation.reportError("Orchestra pause failed: " + status, false);
		}
	}

	public void stop() {
		StatusCode status = orchestra.stop();
		if (!status.isOK()) {
			DriverStation.reportError("Orchestra stop failed: " + status, false);
		}
	}

	public void toggle() {
		if (isPlaying()) {
			pause();
		} else {
			play();
		}
	}

	public boolean isPlaying() {
		return orchestra.isPlaying();
	}

	private static void configurePlaybackWhileDisabled(List<TalonFX> motors) {
		AudioConfigs audioConfigs = new AudioConfigs().withAllowMusicDurDisable(true);
		for (TalonFX motor : motors) {
			StatusCode status = motor.getConfigurator().apply(audioConfigs);
			if (!status.isOK()) {
				DriverStation.reportWarning("Audio config failed for CAN ID " + motor.getDeviceID() + ": " + status, false);
			}
		}
	}
}
