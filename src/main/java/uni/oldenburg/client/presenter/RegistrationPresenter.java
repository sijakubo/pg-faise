package uni.oldenburg.client.presenter;



import com.google.gwt.event.shared.HandlerManager;
import com.google.gwt.user.client.ui.HasWidgets;

import uni.oldenburg.client.SimulationServiceAsync;

public class RegistrationPresenter implements Presenter {  

  

  public interface Display {
    
  }
  
  private final SimulationServiceAsync rpcService;
  private final HandlerManager eventBus;
  private final Display display;
  
  public RegistrationPresenter(SimulationServiceAsync rpcService, HandlerManager eventBus, Display view) {
    this.rpcService = rpcService;
    this.eventBus = eventBus;
    this.display = view;
  }
  
  public void bind() {
   
  }
  
  public void go(final HasWidgets container) {
  
  }

  
}
