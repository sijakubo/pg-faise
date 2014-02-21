package uni.oldenburg.client.presenter;



import uni.oldenburg.client.SimulationServiceAsync;
import uni.oldenburg.client.event.CallRegisterEvent;
import uni.oldenburg.client.event.LoginCompletedEvent;

import com.google.gwt.event.dom.client.ClickEvent;
import com.google.gwt.event.dom.client.ClickHandler;
import com.google.gwt.event.dom.client.HasClickHandlers;
import com.google.gwt.event.shared.HandlerManager;
import com.google.gwt.user.client.ui.HasValue;
import com.google.gwt.user.client.ui.Widget;

public class LoginPresenter extends Presenter {
	private final IDisplay display;
	
	public interface IDisplay {
		HasValue<String> getUsername();
		HasValue<String> getPassword();
		HasClickHandlers getLoginButton();
		HasClickHandlers getRegisterButton();
	}
  
	public LoginPresenter(SimulationServiceAsync rpcService, HandlerManager eventBus, IDisplay view) {
		super(rpcService, eventBus);
		this.display = view;
	}
	
	public Widget getDisplay() {
		return (Widget)display;
	}
  
	private void addSendenButtonListener(){
		display.getLoginButton().addClickHandler(new ClickHandler() {
			public void onClick(ClickEvent event) {
				sendeLogin(	display.getUsername().getValue(),
							display.getPassword().getValue()
							);					
			}	
		});
	}
	
	public void sendeLogin(String value, String value2) {
		eventBus.fireEvent(new LoginCompletedEvent());
    }

	private void addRegistrierenButtonListener(){
		display.getRegisterButton().addClickHandler(new ClickHandler() {
			public void onClick(ClickEvent event) {
				eventBus.fireEvent(new CallRegisterEvent());
			}	
		});
	}
  
	public void bind(){
		this.addSendenButtonListener();
		this.addRegistrierenButtonListener();
	}
}
