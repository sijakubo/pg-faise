package uni.oldenburg.client;

import com.google.gwt.core.client.GWT;
import com.google.gwt.event.logical.shared.ValueChangeEvent;
import com.google.gwt.event.logical.shared.ValueChangeHandler;
import com.google.gwt.event.shared.HandlerManager;
import com.google.gwt.user.client.History;
import com.google.gwt.user.client.ui.HasWidgets;
import com.google.gwt.user.client.ui.Widget;

import uni.oldenburg.client.event.*;
import uni.oldenburg.client.presenter.LoginPresenter;
import uni.oldenburg.client.presenter.MainFramePresenter;
import uni.oldenburg.client.presenter.Presenter;
import uni.oldenburg.client.presenter.RegistrationPresenter;
import uni.oldenburg.client.service.RegistrationAndLoginService;
import uni.oldenburg.client.service.ServiceAsync;
import uni.oldenburg.client.service.SimulationService;
import uni.oldenburg.client.view.LoginView;
import uni.oldenburg.client.view.MainFrameView;
import uni.oldenburg.client.view.RegistrationView;


public class AppController extends Presenter implements ValueChangeHandler<String> {
    private HasWidgets container;

    public AppController(ServiceAsync rpcService, HandlerManager eventBus) {
        super(rpcService, eventBus);
        bind();
    }

    public Widget getDisplay() {
        return null;
    }

    public void bind() {
        History.addValueChangeHandler(this);

        eventBus.addHandler(LoginCompletedEvent.TYPE,
                new LoginCompletedEventHandler() {
                    public void onLogin(LoginCompletedEvent event) {
                        History.newItem("Main");
                    }
                });

        eventBus.addHandler(CallRegisterEvent.TYPE,
                new CallRegisterEventHandler() {
                    public void onRegisterCall(CallRegisterEvent event) {
                        History.newItem("Register");
                    }
                });

        eventBus.addHandler(RegisterCompleteEvent.TYPE,
                new RegisterCompleteEventHandler() {
                    public void onRegister(RegisterCompleteEvent event) {
                        History.newItem("Login");
                    }
                });
    }

    @Override
    public void go(final HasWidgets container) {
        this.container = container;

        if ("".equals(History.getToken())) {
            History.newItem("Login");
        } else {
            History.fireCurrentHistoryState();
        }
    }

    public void onValueChange(ValueChangeEvent<String> event) {
        String token = event.getValue();

        if (token == null)
            return;

        Presenter presenter = null;

        if (token.equals("Login")) {
            ServiceAsync identityService = GWT.create(RegistrationAndLoginService.class);
            presenter = new LoginPresenter(identityService, eventBus, new LoginView());
        }
        else if (token.equals("Register")) {
        	ServiceAsync identityService = GWT.create(RegistrationAndLoginService.class);
            presenter = new RegistrationPresenter(identityService, eventBus, new RegistrationView());
        }
        else if (token.equals("Main")) {
        	ServiceAsync identityService = GWT.create(SimulationService.class);
            presenter = new MainFramePresenter(identityService, eventBus, new MainFrameView());
        }

        if (presenter != null) {
            presenter.go(container);
        }
    }
}
