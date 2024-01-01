package frc2023.util.config;

import java.io.IOException;

public class ConfigException extends RuntimeException {

	public ConfigException(String message) {
		super(message);
	}

	public ConfigException(String message, Throwable cause) {
		super(message, cause);
	}

	public ConfigException(IOException readException, Class<? extends ConfigBase> configClass) {
		this(String.format("An error occurred trying to read config for class %s%n%nSee here for default JSON: %s%n",
				configClass.getSimpleName(), Configs.getDefaultJson(configClass)), readException);
	}
}
