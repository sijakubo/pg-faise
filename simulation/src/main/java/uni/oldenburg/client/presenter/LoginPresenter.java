package uni.oldenburg.client.presenter;



import uni.oldenburg.client.SimulationServiceAsync;
import uni.oldenburg.client.event.CallRegisterEvent;
import uni.oldenburg.client.event.LoginCompletedEvent;

import com.google.gwt.core.client.GWT;
import com.google.gwt.event.dom.client.ClickEvent;
import com.google.gwt.event.dom.client.ClickHandler;
import com.google.gwt.event.dom.client.HasClickHandlers;
import com.google.gwt.event.shared.HandlerManager;
import com.google.gwt.user.client.ui.HasValue;
import com.google.gwt.user.client.ui.HasWidgets;
import com.google.gwt.user.client.ui.Widget;

public class LoginPresenter implements Presenter {  



  public interface Display {
	    HasValue<String> getTextboxUsername();
		HasValue<String> getTextboxPassword();
		HasClickHandlers getLoginButton();
		HasClickHandlers getRegistrierenButton();
		  
		Widget asWidget();
  }
  
  private final SimulationServiceAsync rpcService;
  private final HandlerManager eventBus;
  private final Display display;
  
  public LoginPresenter(SimulationServiceAsync rpcService, HandlerManager eventBus, Display view) {
    this.rpcService = rpcService;
    this.eventBus = eventBus;
    this.display = view;
  }
  
  private void addSendenButtonListener(){
		display.getLoginButton().addClickHandler(new ClickHandler(){
			public void onClick(ClickEvent event) {
				sendeLogin(display.getTextboxUsername().getValue(), display.getTextboxPassword().getValue());
				
			}	
		});
	}
	
	public void sendeLogin(String value, String value2) {
		eventBus.fireEvent(new LoginCompletedEvent());
	
    }

	private void addRegistrierenButtonListener(){
		display.getRegistrierenButton().addClickHandler(new ClickHandler(){
			public void onClick(ClickEvent event) {
				eventBus.fireEvent(new CallRegisterEvent());
				
			}	
		});
	}
  
  
  public void bind(){
	  this.addSendenButtonListener();
	  this.addRegistrierenButtonListener();
  }

  
  public void go(final HasWidgets container) {
	  bind();
	  container.clear();
	  container.add(display.asWidget());
  }

 
  
}
