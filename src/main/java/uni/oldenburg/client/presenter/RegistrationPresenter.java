package uni.oldenburg.client.presenter;



import com.google.gwt.event.dom.client.ClickEvent;
import com.google.gwt.event.dom.client.ClickHandler;
import com.google.gwt.event.dom.client.HasClickHandlers;
import com.google.gwt.event.shared.HandlerManager;
import com.google.gwt.user.client.Window;
import com.google.gwt.user.client.rpc.AsyncCallback;
import com.google.gwt.user.client.ui.HasWidgets;
import com.google.gwt.user.client.ui.Widget;

import java.util.ArrayList;
import java.util.List;

import uni.oldenburg.client.SimulationsServiceAsync;

public class RegistrationPresenter implements Presenter {  

  

  public interface Display {
    
  }
  
  private final SimulationsServiceAsync rpcService;
  private final HandlerManager eventBus;
  private final Display display;
  
  public RegistrationPresenter(SimulationsServiceAsync rpcService, HandlerManager eventBus, Display view) {
    this.rpcService = rpcService;
    this.eventBus = eventBus;
    this.display = view;
  }
  
  public void bind() {
   
  }
  
  public void go(final HasWidgets container) {
  
  }

  
}
