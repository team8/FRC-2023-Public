package frc2023.subsystems.controllers.lighting;

import frc2023.robot.Commands;
import frc2023.robot.RobotState;
import frc2023.subsystems.Lighting;

public class RainbowController extends Lighting.LEDController {

	private double mDuration = -1;

	int s = 255;
	int v = 20;
	int h;

	private int mFinalHue;
	private int mInitHue;
	private boolean reverse;

	/**
	 * Band color converges to center of strip
	 *
	 * @param startIndex Initial index upon which led patterns should start
	 * @param lastIndex  End index upon which led patterns should stop
	 */

	public RainbowController(int startIndex, int lastIndex, int initHue, int finalHue, boolean initReverse) {
		super(startIndex, lastIndex);
		mStartIndex = startIndex;
		mLastIndex = lastIndex;
		kPriority = 1;
		mTimer.start();
		mInitHue = initHue;
		mFinalHue = finalHue;
		h = mInitHue;
		reverse = initReverse;
	}

	@Override
	public void updateSignal(Commands commands, RobotState state) {
		int hprime = h;
		if (!reverse) {
			for (int i = 0; i < mOutputs.lightingOutput.size(); i++) {
				mOutputs.lightingOutput.get(i).setHSV(hprime, s, v);
				hprime++;
				if (hprime > mFinalHue) {
					hprime = mInitHue;
				}
			}
			h++;
			if (h > mFinalHue) h = mInitHue;
		} else {
			for (int i = mOutputs.lightingOutput.size() - 1; i >= 0; i--) {
				mOutputs.lightingOutput.get(i).setHSV(hprime, s, v);
				hprime++;
				if (hprime > mFinalHue) {
					hprime = mInitHue;
				}
			}
			h++;
			if (h > mFinalHue) h = mInitHue;
		}
	}

	@Override
	public boolean checkFinished() {
		return mDuration != -1 && mTimer.hasElapsed(mDuration);
	}
}
