package frc2023.util.commands;

import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;

public class AtomicString {

	private final ReentrantLock m_Lock = new ReentrantLock();
	private String m_String;

	public String waitAndGet() throws InterruptedException {
		synchronized (m_Lock) {
			m_Lock.wait();
			return m_String;
		}
	}

	public void tryGetAndReset(Consumer<String> result) {
		if (m_Lock.tryLock()) {
			result.accept(m_String);
			m_String = null;
			m_Lock.unlock();
		}
	}

	public void setAndNotify(final String newString) {
		synchronized (m_Lock) {
			m_String = newString;
			m_Lock.notify();
		}
	}

	public void set(final String newString) {
		synchronized (m_Lock) {
			m_String = newString;
		}
	}
}
