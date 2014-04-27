package uni.oldenburg.client.presenter;


import com.google.gwt.event.dom.client.ClickEvent;
import com.google.gwt.event.dom.client.ClickHandler;
import com.google.gwt.event.dom.client.HasClickHandlers;
import com.google.gwt.event.shared.HandlerManager;
import com.google.gwt.user.client.Window;
import com.google.gwt.user.client.rpc.AsyncCallback;
import com.google.gwt.user.client.ui.HasValue;
import com.google.gwt.user.client.ui.Widget;

import de.novanic.eventservice.client.event.Event;
import de.novanic.eventservice.client.event.RemoteEventService;
import de.novanic.eventservice.client.event.RemoteEventServiceFactory;
import de.novanic.eventservice.client.event.domain.DomainFactory;
import de.novanic.eventservice.client.event.listener.RemoteEventListener;
import uni.oldenburg.client.event.CallRegisterEvent;
import uni.oldenburg.client.event.HelloEvent;
import uni.oldenburg.client.event.LoginCompletedEvent;
import uni.oldenburg.client.service.RegistrationAndLoginServiceAsync;

public class LoginPresenter extends Presenter {
    private final IDisplay display;
    RemoteEventService theRemoteEventService;

    public interface IDisplay {
        HasValue<String> getEmail();

        HasValue<String> getPassword();

        HasClickHandlers getLoginButton();

        HasClickHandlers getRegisterButton();
    }

    public LoginPresenter(RegistrationAndLoginServiceAsync rpcService, HandlerManager eventBus, IDisplay view) {
        super(rpcService, eventBus);
        this.display = view;
    }

    public Widget getDisplay() {
        return (Widget) display;
    }

    private void addLoginButtonListener() {
        display.getLoginButton().addClickHandler(new ClickHandler() {
            public void onClick(ClickEvent event) {
                sendLogin(	display.getEmail().getValue(),
                        	display.getPassword().getValue()
                );
            }
        });
    }

    public void sendLogin(final String email, final String password) {
        ((RegistrationAndLoginServiceAsync) rpcService).loginUser(email, password, new AsyncCallback<Boolean>() {
            public void onFailure(Throwable throwable) {
                Window.alert("Fehler beim Abfragen der Benutzerdaten");
            }

            public void onSuccess(Boolean loginSuccessful) {
                if (loginSuccessful) {
                    eventBus.fireEvent(new LoginCompletedEvent());
                } else {
                    Window.alert("Benutzername oder Passwort nicht korrekt");
                }
            }
        });

    }

    private void addRegisterButtonListener() {
        display.getRegisterButton().addClickHandler(new ClickHandler() {
            public void onClick(ClickEvent event) {
                eventBus.fireEvent(new CallRegisterEvent());
            }
        });
    }

    public void bind() {
        this.addLoginButtonListener();
        this.addRegisterButtonListener();
        this.horcheAufDomain();
    }
    
    
    //Beispiel, wie man einen Listener für eine Domain registriert
    private void horcheAufDomain() {

		// registriere den EventListener auf dem RemoteeventService
		this.theRemoteEventService = RemoteEventServiceFactory.getInstance()
				.getRemoteEventService();
		theRemoteEventService.removeListeners();
		
		theRemoteEventService.addListener(
				DomainFactory.getDomain("Test"),
				new RemoteEventListener() {
					public void apply(Event anEvent) {
						
						if (anEvent instanceof HelloEvent) {
							
						     Window.alert("Now you will be redirected to the Mainframe");
						}
					}});
	}
}
