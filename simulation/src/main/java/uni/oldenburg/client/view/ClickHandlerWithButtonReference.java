package uni.oldenburg.client.view;

import com.google.gwt.event.dom.client.ClickEvent;
import com.google.gwt.event.dom.client.ClickHandler;
import com.google.gwt.user.client.ui.ListBox;


/**
 * The ClickHandler gets the name of the selected Listboxitem and delivers it to loadSzenario Method
 * 
 * @author Raschid
 */
public class ClickHandlerWithButtonReference implements ClickHandler {
	private ListBox referencedListBox;
	private DialogBoxScenarioSelection dialogBox;
	
	public ClickHandlerWithButtonReference(ListBox lBox,DialogBoxScenarioSelection dialogBox){
		this.referencedListBox=lBox;
		this.dialogBox=dialogBox;
	}

	public void onClick(ClickEvent event) {
		dialogBox.hide();
		dialogBox.getPresenter().loadSzenario(referencedListBox.getItemText(referencedListBox.getSelectedIndex()));	
	}
}
