package uni.oldenburg.client.presenter;

import uni.oldenburg.client.service.SimulationServiceAsync;

import com.google.gwt.event.shared.HandlerManager;
import com.google.gwt.user.client.ui.HasWidgets;
import com.google.gwt.user.client.ui.Widget;

public abstract class Presenter {
	protected final HandlerManager eventBus;

	public Presenter(HandlerManager eventBus) {
		this.eventBus = eventBus;
	}

	public abstract void bind();
	public abstract Widget getDisplay();

	public void go(final HasWidgets container) {
		bind();
		container.clear();
		container.add(getDisplay());
	}
}