package uni.oldenburg.client.presenter;



import uni.oldenburg.client.SimulationServiceAsync;

import com.google.gwt.event.shared.HandlerManager;
import com.google.gwt.user.client.ui.Widget;

public class RegistrationPresenter extends Presenter {
	private final IDisplay display;
	
	public interface IDisplay {}
	
	public RegistrationPresenter(SimulationServiceAsync rpcService, HandlerManager eventBus, IDisplay view) {
		super(rpcService, eventBus);
		this.display = view;
	}
	
	public Widget getDisplay() {
		return (Widget)display;
	}	
  
	public void bind() {
	}
}
