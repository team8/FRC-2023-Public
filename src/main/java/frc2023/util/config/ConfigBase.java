package frc2023.util.config;

public abstract class ConfigBase {

	@Override
	public String toString() {
		return Configs.toJson(this);
	}

	void onPostUpdate() {
	}
}
