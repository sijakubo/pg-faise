package uni.oldenburg.client;

import uni.oldenburg.client.event.CallRegisterEvent;
import uni.oldenburg.client.event.CallRegisterEventHandler;
import uni.oldenburg.client.event.LoginCompletedEvent;
import uni.oldenburg.client.event.LoginCompletedEventHandler;
import uni.oldenburg.client.presenter.LoginPresenter;
import uni.oldenburg.client.presenter.MainFramePresenter;
import uni.oldenburg.client.presenter.Presenter;
import uni.oldenburg.client.presenter.RegistrationPresenter;
import uni.oldenburg.client.view.LoginView;
import uni.oldenburg.client.view.MainFrameView;
import uni.oldenburg.client.view.RegistrationView;

import com.google.gwt.event.logical.shared.ValueChangeEvent;
import com.google.gwt.event.logical.shared.ValueChangeHandler;
import com.google.gwt.event.shared.HandlerManager;
import com.google.gwt.user.client.History;
import com.google.gwt.user.client.ui.HasWidgets;



public class AppController implements Presenter, ValueChangeHandler<String> {
    private final HandlerManager eventBus;
    private final SimulationServiceAsync rpcService;
    private HasWidgets container;

    public AppController(SimulationServiceAsync rpcService, HandlerManager eventBus) {
        this.eventBus = eventBus;
        this.rpcService = rpcService;
        bind();
    }

    private void bind() {
    	 History.addValueChangeHandler(this);
    	 
    	 eventBus.addHandler(LoginCompletedEvent.TYPE,
    	            new LoginCompletedEventHandler() {
    	              public void onLogin(LoginCompletedEvent event) {
    	            	  History.newItem("Mainpage");
    	              }
    	            }); 
    	 
    	 eventBus.addHandler(CallRegisterEvent.TYPE,
    	            new CallRegisterEventHandler() {
    				public void onRegisterCall(CallRegisterEvent event) {
    					History.newItem("Register");
    					
    				}
    	            }); 
    }


    public void go(final HasWidgets container) {
        this.container = container;

        if ("".equals(History.getToken())) {
            History.newItem("Startpage");
        } else {
            History.fireCurrentHistoryState();
        }
    }
    
    

      public void onValueChange(ValueChangeEvent<String> event) {
        String token = event.getValue();
        
        if (token != null) {
          Presenter presenter = null;

          if (token.equals("Startpage")) {
              presenter = new LoginPresenter(rpcService,eventBus,new LoginView());
          }else if(token.equals("Register")){
        	  presenter =new RegistrationPresenter(rpcService,eventBus,new RegistrationView());
          }else if(token.equals("Mainpage")){
        	  presenter =new MainFramePresenter(rpcService,eventBus,new MainFrameView());
          }
         
          
          if (presenter != null) {
            presenter.go(container);
          }
        }
      } 

    
}
