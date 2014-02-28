package uni.oldenburg.client.presenter;

import uni.oldenburg.client.service.ServiceAsync;

import com.google.gwt.event.shared.HandlerManager;
import com.google.gwt.user.client.ui.HasWidgets;
import com.google.gwt.user.client.ui.Widget;

public abstract class Presenter {
    protected ServiceAsync rpcService;
	protected final HandlerManager eventBus;

	public Presenter(ServiceAsync rpcService, HandlerManager eventBus) {
		this.rpcService = rpcService;
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