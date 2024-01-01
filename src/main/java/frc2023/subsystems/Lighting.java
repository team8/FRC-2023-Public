package frc2023.subsystems;

import java.util.Comparator;
import java.util.PriorityQueue;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

import frc2023.config.constants.PortConstants;
import frc2023.config.subsystem.LightingConfig;
import frc2023.robot.Commands;
import frc2023.robot.ReadOnly;
import frc2023.robot.RobotState;
import frc2023.subsystems.controllers.lighting.FadeInFadeOutController;
import frc2023.subsystems.controllers.lighting.OneColorController;
import frc2023.subsystems.controllers.lighting.RainbowController;
import frc2023.util.Color;
import frc2023.util.config.Configs;
import frc2023.util.control.LightingOutputs;

public class Lighting extends SubsystemBase {

	public enum State {
		OFF, CUBE, CONE, ALIGN, DISABLE, FRANCE, ADIT, MIHIR, DO_NOTHING, HIGH, LOW, MID, LINE_UP
	}

	public abstract static class LEDController {

		protected static final double kZeroSpeed = 1e-4;

		protected Timer mTimer = new Timer();
		protected LightingOutputs mOutputs = new LightingOutputs();
		protected int mStartIndex;
		protected int mLastIndex;
		protected double mSpeed;
		protected int kPriority;

		protected LEDController(int startIndex, int lastIndex) {
			for (var i = 0; i <= Math.abs(lastIndex - startIndex); i++) {
				mOutputs.lightingOutput.add(new Color.HSV());
			}
			mTimer.reset();
		}

		public abstract void updateSignal(@ReadOnly Commands commands, @ReadOnly RobotState state);

		public final LightingOutputs update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
			updateSignal(commands, state);
			return mOutputs;
		}

		public boolean checkFinished() {
			return false;
		}
	}

	private static final Lighting INSTANCE = new Lighting();

	private State state = State.DO_NOTHING;

	private final LightingConfig config = Configs.get(LightingConfig.class);
	private final AddressableLEDBuffer outputBuffer = new AddressableLEDBuffer(config.ledCount);
	private final PriorityQueue<LEDController> LEDControllers = new PriorityQueue<>(1, Comparator.comparingInt(o -> o.kPriority));

	public final AddressableLED ledStrip = new AddressableLED(PortConstants.lightingPwmPort);

	private Lighting() {
	}

	public static Lighting getInstance() {
		return INSTANCE;
	}

	@Override
	public void update(Commands commands, RobotState state) {
		State wantedState = commands.lightingWantedState;
		if (RobotController.getBatteryVoltage() < config.minVoltageToFunction) wantedState = State.OFF;
		boolean isNewState = this.state != wantedState;
		this.state = wantedState;
		if (isNewState) {
			switch (this.state) {
				case CUBE:
					LEDControllers.clear();
					addToControllers(new OneColorController(config.segment1FirstIndex, config.segment1LastIndex, Color.HSV.kPurple));
					addToControllers(new OneColorController(config.segment2FirstIndex, config.segment2LastIndex, Color.HSV.kPurple));
					addToControllers(new OneColorController(config.segment3FirstIndex, config.segment3LastIndex, Color.HSV.kPurple));
					addToControllers(new OneColorController(config.segment4FirstIndex, config.segment4LastIndex, Color.HSV.kPurple));
					break;
				case CONE:
					LEDControllers.clear();
					addToControllers(new OneColorController(config.segment1FirstIndex, config.segment1LastIndex, Color.HSV.kYellow));
					addToControllers(new OneColorController(config.segment2FirstIndex, config.segment2LastIndex, Color.HSV.kYellow));
					addToControllers(new OneColorController(config.segment3FirstIndex, config.segment3LastIndex, Color.HSV.kYellow));
					addToControllers(new OneColorController(config.segment4FirstIndex, config.segment4LastIndex, Color.HSV.kYellow));
					break;
				case LINE_UP:
					addToControllers(new FadeInFadeOutController(config.segment1FirstIndex, config.segment1LastIndex, Color.HSV.kLime, 1));
					addToControllers(new FadeInFadeOutController(config.segment2FirstIndex, config.segment2LastIndex, Color.HSV.kLime, 1));
					addToControllers(new FadeInFadeOutController(config.segment3FirstIndex, config.segment3LastIndex, Color.HSV.kLime, 1));
					addToControllers(new FadeInFadeOutController(config.segment4FirstIndex, config.segment4LastIndex, Color.HSV.kLime, 1));
					break;
				case ALIGN:
					LEDControllers.clear();
					addToControllers(new OneColorController(config.segment1FirstIndex, config.segment1LastIndex, Color.HSV.kLime));
					addToControllers(new OneColorController(config.segment2FirstIndex, config.segment2LastIndex, Color.HSV.kLime));
					addToControllers(new OneColorController(config.segment3FirstIndex, config.segment3LastIndex, Color.HSV.kLime));
					addToControllers(new OneColorController(config.segment4FirstIndex, config.segment4LastIndex, Color.HSV.kLime));
					break;
				case DISABLE:
					LEDControllers.clear();
					addToControllers(new RainbowController(config.segment1FirstIndex, config.segment1LastIndex, 5, 175, true));
					addToControllers(new RainbowController(config.segment2FirstIndex, config.segment2LastIndex, 5, 175, false));
					addToControllers(new RainbowController(config.segment3FirstIndex, config.segment3LastIndex, 5, 175, false));
					addToControllers(new RainbowController(config.segment4FirstIndex, config.segment4LastIndex, 5, 175, true));
					break;
				case FRANCE:
					LEDControllers.clear();
					addToControllers(new OneColorController(config.segment1FirstIndex, config.segment1LastIndex, Color.HSV.kRed));
					addToControllers(new OneColorController(config.segment2FirstIndex, config.segment2LastIndex, Color.HSV.kBlue));
					addToControllers(new OneColorController(config.segment3FirstIndex, config.segment3LastIndex, Color.HSV.kBlue));
					addToControllers(new OneColorController(config.segment4FirstIndex, config.segment4LastIndex, Color.HSV.kWhite));
					break;
				case ADIT:
					LEDControllers.clear();
					addToControllers(new OneColorController(config.segment1FirstIndex, config.segment1LastIndex, Color.HSV.kLime));
					addToControllers(new OneColorController(config.segment2FirstIndex, config.segment2LastIndex, Color.HSV.kBlue));
					addToControllers(new OneColorController(config.segment3FirstIndex, config.segment3LastIndex, Color.HSV.kBlue));
					addToControllers(new OneColorController(config.segment4FirstIndex, config.segment4LastIndex, Color.HSV.kLime));
					break;
				case HIGH:
					LEDControllers.clear();
					addToControllers(new OneColorController(config.segment1FirstIndex, config.segment1LastIndex, Color.HSV.kRed));
					addToControllers(new OneColorController(config.segment2FirstIndex, config.segment2LastIndex, Color.HSV.kRed));
					addToControllers(new OneColorController(config.segment3FirstIndex, config.segment3LastIndex, Color.HSV.kRed));
					addToControllers(new OneColorController(config.segment4FirstIndex, config.segment4LastIndex, Color.HSV.kRed));
					break;
				case MID:
					LEDControllers.clear();
					addToControllers(new OneColorController(config.segment1FirstIndex, config.segment1LastIndex, Color.HSV.kLime));
					addToControllers(new OneColorController(config.segment2FirstIndex, config.segment2LastIndex, Color.HSV.kLime));
					addToControllers(new OneColorController(config.segment3FirstIndex, config.segment3LastIndex, Color.HSV.kLime));
					addToControllers(new OneColorController(config.segment4FirstIndex, config.segment4LastIndex, Color.HSV.kLime));
					break;
				case LOW:
					LEDControllers.clear();
					addToControllers(new OneColorController(config.segment1FirstIndex, config.segment1LastIndex, Color.HSV.kBlue));
					addToControllers(new OneColorController(config.segment2FirstIndex, config.segment2LastIndex, Color.HSV.kBlue));
					addToControllers(new OneColorController(config.segment3FirstIndex, config.segment3LastIndex, Color.HSV.kBlue));
					addToControllers(new OneColorController(config.segment4FirstIndex, config.segment4LastIndex, Color.HSV.kBlue));
					break;
				case DO_NOTHING:
					break;
				case OFF:
					break;
				default:
					LEDControllers.clear();
					break;

			}
		}
		resetLedStrip();

		if (LEDControllers.removeIf(LEDController::checkFinished)) {
			this.state = State.DO_NOTHING;
		}
		for (LEDController ledController : LEDControllers) {
			LightingOutputs controllerOutput = ledController.update(commands, state);
			for (int i = ledController.mStartIndex; i < ledController.mLastIndex; i++) {
				Color.HSV hsvValue = controllerOutput.lightingOutput.get(i - ledController.mStartIndex);
				outputBuffer.setHSV(i, hsvValue.getH(), hsvValue.getS(), Math.min(hsvValue.getV(), config.maximumBrightness));
			}
		}
	}

	@Override
	public void writeHardware(RobotState state) {
		ledStrip.setData(outputBuffer);
	}

	@Override
	public void configureHardware() {
		ledStrip.setLength(Configs.get(LightingConfig.class).ledCount);
		ledStrip.start();
	}

	@Override
	public void readHardware(RobotState state) {
	}

	private void addToControllers(LEDController controller) {
		LEDControllers.add(controller);
	}

	public void resetLedStrip() {
		for (int i = 0; i < outputBuffer.getLength(); i++) {
			outputBuffer.setRGB(i, 0, 0, 0);
		}
	}

}
