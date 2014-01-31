package uni.oldenburg.client;



import uni.oldenburg.client.presenter.Presenter;

import com.google.gwt.event.logical.shared.ValueChangeEvent;
import com.google.gwt.event.logical.shared.ValueChangeHandler;
import com.google.gwt.event.shared.HandlerManager;
import com.google.gwt.user.client.History;
import com.google.gwt.user.client.ui.HasWidgets;

public class AppController implements Presenter, ValueChangeHandler<String> {
  private final HandlerManager eventBus;
  private final SimulationsServiceAsync rpcService; 
  private HasWidgets container;
  
  public AppController(SimulationsServiceAsync rpcService, HandlerManager eventBus) {
	  this.eventBus = eventBus;
	  this.rpcService = rpcService;
	  bind();
  }
  
  private void bind() {
    
  }
  
  
  
  public void go(final HasWidgets container) {
    this.container = container;
    
    if ("".equals(History.getToken())) {
      History.newItem("list");
    }
    else {
      History.fireCurrentHistoryState();
    }
  }

  public void onValueChange(ValueChangeEvent<String> event) {
    
  } 
}
