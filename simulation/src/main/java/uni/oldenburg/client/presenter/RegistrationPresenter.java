package uni.oldenburg.client.presenter;


import com.google.gwt.event.dom.client.ClickEvent;
import com.google.gwt.event.dom.client.ClickHandler;
import com.google.gwt.event.dom.client.HasClickHandlers;
import com.google.gwt.event.shared.HandlerManager;
import com.google.gwt.user.client.Window;
import com.google.gwt.user.client.rpc.AsyncCallback;
import com.google.gwt.user.client.ui.HasValue;
import com.google.gwt.user.client.ui.Widget;
import uni.oldenburg.client.event.RegisterCompleteEvent;
import uni.oldenburg.client.service.RegistrationAndLoginServiceAsync;
import uni.oldenburg.shared.model.SimulationUser;

public class RegistrationPresenter extends Presenter {
    private RegistrationAndLoginServiceAsync rpcService;
    private final IDisplay display;

    public interface IDisplay {
        HasValue<String> getPassword();
        HasValue<String> getEmail();
        HasValue<String> getName();
        HasClickHandlers getRegisterButton();
    }

    public RegistrationPresenter(RegistrationAndLoginServiceAsync rpcService, HandlerManager eventBus, IDisplay view) {
        super(eventBus);
        this.rpcService = rpcService;
        this.display = view;
    }

    public Widget getDisplay() {
        return (Widget) display;
    }

    private void addRegisterButtonListener() {
        display.getRegisterButton().addClickHandler(new ClickHandler() {
            public void onClick(ClickEvent event) {
                registerUser(display.getEmail().getValue(),
                        display.getName().getValue(),
                        display.getPassword().getValue());
            }
        });
    }

    public void registerUser(String email, String name, String password) {
        rpcService.registerUser(new SimulationUser(email, name, password), new AsyncCallback<Boolean>() {
            @Override
            public void onFailure(Throwable throwable) {
                Window.alert("Email bereits vergeben");
            }

            @Override
            public void onSuccess(Boolean aBoolean) {
                eventBus.fireEvent(new RegisterCompleteEvent());
            }
        });
    }

    public void bind() {
        this.addRegisterButtonListener();
    }
}
