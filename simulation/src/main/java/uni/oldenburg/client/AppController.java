package uni.oldenburg.client;

import com.google.gwt.core.client.GWT;
import com.google.gwt.event.logical.shared.ValueChangeEvent;
import com.google.gwt.event.logical.shared.ValueChangeHandler;
import com.google.gwt.event.shared.HandlerManager;
import com.google.gwt.user.client.History;
import com.google.gwt.user.client.Window;
import com.google.gwt.user.client.rpc.AsyncCallback;
import com.google.gwt.user.client.ui.HasWidgets;
import com.google.gwt.user.client.ui.Widget;

import uni.oldenburg.client.event.*;
import uni.oldenburg.client.presenter.LoginPresenter;
import uni.oldenburg.client.presenter.MainFramePresenter;
import uni.oldenburg.client.presenter.Presenter;
import uni.oldenburg.client.presenter.RegistrationPresenter;
import uni.oldenburg.client.service.*;
import uni.oldenburg.client.view.LoginView;
import uni.oldenburg.client.view.MainFrameView;
import uni.oldenburg.client.view.RegistrationView;


public class AppController extends Presenter implements ValueChangeHandler<String> {
   private HasWidgets container;
   private boolean isUserLoggedIn;

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

        eventBus.addHandler(UserNotLoggedInEvent.TYPE,
                new UserNotLoggedInEventHandler() {
                    public void onSiteCall(UserNotLoggedInEvent event) {
                        History.back();
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
         RegistrationAndLoginServiceAsync identityService = GWT.create(RegistrationAndLoginService.class);
         presenter = new LoginPresenter(identityService, eventBus, new LoginView());
      } else if (token.equals("Register")) {
         RegistrationAndLoginServiceAsync identityService = GWT.create(RegistrationAndLoginService.class);
         presenter = new RegistrationPresenter(identityService, eventBus, new RegistrationView());
      } else if (token.equals("Main")) {
         if (isUserLoggedIn()) {
            SimulationServiceAsync identityService = GWT.create(SimulationService.class);
            presenter = new MainFramePresenter(identityService, eventBus, new MainFrameView());
         } else {
            Window.alert("User is not logged in");
            History.back();
         }
      }

      if (presenter != null) {
         presenter.go(container);
      }
   }

   /**
    * @author sijakubo
    */
   private boolean isUserLoggedIn() {
      //Check if User is LoggedIn. Else return User to LoginPage
      SessionInformationServiceAsync sessionInformationService = GWT.create(SessionInformationService.class);
      sessionInformationService.isUserLoggedInInSession(new AsyncCallback<Boolean>() {
         public void onFailure(Throwable caught) {
            Window.alert("Error while getting the Session Information");
         }

         public void onSuccess(Boolean isUserLoggedIn) {
            AppController.this.isUserLoggedIn = isUserLoggedIn;

         }
      });

      return isUserLoggedIn;
   }
}
