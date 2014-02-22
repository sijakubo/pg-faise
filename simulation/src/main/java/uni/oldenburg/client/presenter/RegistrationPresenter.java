package uni.oldenburg.client.presenter;



import uni.oldenburg.client.SimulationServiceAsync;
import uni.oldenburg.client.event.RegisterCompleteEvent;

import com.google.gwt.event.dom.client.ClickEvent;
import com.google.gwt.event.dom.client.ClickHandler;
import com.google.gwt.event.dom.client.HasClickHandlers;
import com.google.gwt.event.shared.HandlerManager;
import com.google.gwt.user.client.ui.HasValue;
import com.google.gwt.user.client.ui.Widget;

public class RegistrationPresenter extends Presenter {
	private final IDisplay display;
	
	public interface IDisplay {
		HasValue<String> getUsername();
		HasValue<String> getPassword();
		HasClickHandlers getRegisterButton();
	}
	
	public RegistrationPresenter(SimulationServiceAsync rpcService, HandlerManager eventBus, IDisplay view) {
		super(rpcService, eventBus);
		this.display = view;
	}
	
	public Widget getDisplay() {
		return (Widget)display;
	}
	
	private void addRegisterButtonListener() {
		display.getRegisterButton().addClickHandler(new ClickHandler() {
			public void onClick(ClickEvent event) {
				sendRegister(	display.getUsername().getValue(),
							 	display.getPassword().getValue()
							 	);
			}
		});
	}
	
	public void sendRegister(String user, String pass) {
		eventBus.fireEvent(new RegisterCompleteEvent());
	}
  
	public void bind() {
		this.addRegisterButtonListener();
	}
}
