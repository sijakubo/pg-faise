package uni.oldenburg.client.view;

import com.google.gwt.event.dom.client.ClickEvent;
import com.google.gwt.event.dom.client.ClickHandler;
import com.google.gwt.user.client.ui.Button;

public class ClickHandlerWithButtonReference implements ClickHandler {
	
	private Button referencedButton;
	private DialogBoxScenarioSelection dialogBox;
	
	
	public ClickHandlerWithButtonReference(Button button,DialogBoxScenarioSelection dialogBox){
		this.referencedButton=button;
		this.dialogBox=dialogBox;
	}
	

	public void onClick(ClickEvent event) {
		dialogBox.hide();
		dialogBox.getPresenter().loadSzenario(referencedButton.getText());
		
	}

}
