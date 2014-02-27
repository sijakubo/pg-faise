package uni.oldenburg.client.presenter;



import uni.oldenburg.client.service.SimulationServiceAsync;
import uni.oldenburg.client.event.CallRegisterEvent;
import uni.oldenburg.client.event.LoginCompletedEvent;

import com.google.gwt.event.dom.client.ClickEvent;
import com.google.gwt.event.dom.client.ClickHandler;
import com.google.gwt.event.dom.client.HasClickHandlers;
import com.google.gwt.event.shared.HandlerManager;
import com.google.gwt.user.client.ui.HasValue;
import com.google.gwt.user.client.ui.Widget;

public class LoginPresenter extends Presenter {
    private SimulationServiceAsync rpcService;
    private final IDisplay display;
	
	public interface IDisplay {
		HasValue<String> getUsername();
		HasValue<String> getPassword();
		HasClickHandlers getLoginButton();
		HasClickHandlers getRegisterButton();
	}
  
	public LoginPresenter(SimulationServiceAsync rpcService, HandlerManager eventBus, IDisplay view) {
		super(eventBus);
        this.rpcService = rpcService;
        this.display = view;
	}
	
	public Widget getDisplay() {
		return (Widget)display;
	}
  
	private void addLoginButtonListener() {
		display.getLoginButton().addClickHandler(new ClickHandler() {
			public void onClick(ClickEvent event) {
				sendeLogin(	display.getUsername().getValue(),
							display.getPassword().getValue()
							);					
			}	
		});
	}
	
	public void sendeLogin(String user, String pass) {
		eventBus.fireEvent(new LoginCompletedEvent());
    }

	private void addRegisterButtonListener() {
		display.getRegisterButton().addClickHandler(new ClickHandler() {
			public void onClick(ClickEvent event) {
				eventBus.fireEvent(new CallRegisterEvent());
			}	
		});
	}
  
	public void bind(){
		this.addLoginButtonListener();
		this.addRegisterButtonListener();
	}
}
