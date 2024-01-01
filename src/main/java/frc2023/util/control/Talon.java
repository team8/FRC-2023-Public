package frc2023.util.control;

import static frc2023.config.constants.RobotConstants.pidIndex;
import static frc2023.config.constants.RobotConstants.timeoutMS;
import static java.util.Map.entry;

import java.util.Map;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.esotericsoftware.minlog.Log;

public class Talon extends TalonSRX implements Controller {

	static class BaseTalonController<T extends BaseTalon & Controller> extends ProfiledControllerBase<T> {

		protected double mPositionConversion, mVelocityConversion;

		protected BaseTalonController(T talon) {
			super(talon);
		}

		@Override
		protected void updateGains(boolean isInit, int slot, Gains newGains, Gains lastGains) {
			super.updateGains(isInit, slot, newGains, lastGains);
			if (isInit) {
				var err = mController.configMotionSCurveStrength(4, timeoutMS);
				if (err != ErrorCode.OK) {
					System.out.println(err);
				}
			}
		}

		@Override
		void setProfiledAcceleration(int slot, double acceleration) {
			var err = mController.configMotionAcceleration(round(acceleration), timeoutMS);
			if (err != ErrorCode.OK) {
				System.out.println(err);
			}
		}

		@Override
		void setProfiledCruiseVelocity(int slot, double cruiseVelocity) {
			var err = mController.configMotionCruiseVelocity(round(cruiseVelocity), timeoutMS);
			if (err != ErrorCode.OK) {
				System.out.println(err);
			}
		}

		@Override
		protected void setProfiledAllowableError(int slot, double allowableError) {
			var err = mController.configAllowableClosedloopError(slot, round(allowableError), timeoutMS);
			if (err != ErrorCode.OK) {
				System.out.println(err);
			}
		}

		@Override
		protected void setProfiledMinimumVelocityOutput(int slot, double minimumOutputVelocity) {
			// Not supported by Talons
		}

		@Override
		protected boolean setReference(ControllerOutput.Mode mode, int slot, double reference, double arbitraryPercentOutput) {
			ControlMode controllerMode = kModeToController.get(mode);
			double convertedReference;
			switch (mode) {
				case VELOCITY:
				case PROFILED_VELOCITY:
					convertedReference = reference / mVelocityConversion;
					break;
				case POSITION:
				case PROFILED_POSITION:
					convertedReference = reference / mPositionConversion;
					break;
				default:
					convertedReference = reference;
					break;
			}
			mController.selectProfileSlot(slot, pidIndex);
			mController.set(controllerMode, convertedReference, DemandType.ArbitraryFeedForward, arbitraryPercentOutput);
			return true;
		}

		@Override
		protected void setP(int slot, double p) {
			var err = mController.config_kP(slot, p, timeoutMS);
			if (err != ErrorCode.OK) {
				System.out.println(err);
			}
		}

		@Override
		protected void setI(int slot, double i) {
			var err = mController.config_kI(slot, i, timeoutMS);
			if (err != ErrorCode.OK) {
				System.out.println(err);
			}
		}

		@Override
		protected void setD(int slot, double d) {
			var err = mController.config_kD(slot, d, timeoutMS);
			if (err != ErrorCode.OK) {
				System.out.println(err);
			}
		}

		@Override
		protected void setF(int slot, double f) {
			var err = mController.config_kF(slot, f, timeoutMS);
			if (err != ErrorCode.OK) {
				System.out.println(err);
			}
		}

		@Override
		protected void setIZone(int slot, double iZone) {
			var err = mController.config_IntegralZone(slot, round(iZone), timeoutMS);
			if (err != ErrorCode.OK) {
				System.out.println(err);
			}
		}

		@Override
		protected void setIMax(int slot, double iMax) {
			var err = mController.configMaxIntegralAccumulator(slot, iMax, timeoutMS);
			if (err != ErrorCode.OK) {
				System.out.println(err);
			}
		}

		@Override
		protected void setFrameTimings() {
			/* Update period of commands sent to controller */
			mController.setControlFramePeriod(ControlFrame.Control_3_General, mControlFrameMs);
			/* Update period of feedback received from controller */
			// Applied motor output, fault information, limit switch information
			mController.setStatusFramePeriod(StatusFrame.Status_1_General, mStatusFrameMs, timeoutMS);
			// Selected sensor position and velocity, supply current measurement, sticky fault information
			mController.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, mStatusFrameMs, timeoutMS);
		}
	}

	protected static final Map<ControllerOutput.Mode, ControlMode> kModeToController = Map.ofEntries(
			entry(ControllerOutput.Mode.PERCENT_OUTPUT, ControlMode.PercentOutput),
			entry(ControllerOutput.Mode.POSITION, ControlMode.Position),
			entry(ControllerOutput.Mode.VELOCITY, ControlMode.Velocity),
			entry(ControllerOutput.Mode.PROFILED_POSITION, ControlMode.MotionMagic),
			entry(ControllerOutput.Mode.PROFILED_VELOCITY, ControlMode.MotionProfile));
	private final BaseTalonController<Talon> mController = new BaseTalonController<>(this);
	private final String mName;

	public Talon(int deviceId, String name) {
		super(deviceId);
		mName = name;
		clearStickyFaults(timeoutMS);
	}

	public boolean setOutput(ControllerOutput output) {
		return mController.setOutput(output);
	}

	/**
	 * When controllers reset over CAN, frame periods are cleared. This handles resetting them to their
	 * configured values before.
	 */
	public void handleReset() {
		if (hasResetOccurred()) {
			Log.error("reset", String.format("%s reset", mController.getName()));
			mController.updateFrameTimings();
		}
	}

	public void configFrameTimings(int controlFrameMs, int statusFrameMs) {
		mController.configFrameTimings(controlFrameMs, statusFrameMs);
	}

	public String getName() {
		return String.format("(Talon #%d), %s", getDeviceID(), mName);
	}

	public static int round(double d) {
		return (int) Math.round(d);
	}
}
