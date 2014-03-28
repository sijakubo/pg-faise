package uni.oldenburg.client.view;

import uni.oldenburg.client.presenter.MainFramePresenter;

import com.google.gwt.event.dom.client.ClickEvent;
import com.google.gwt.event.dom.client.ClickHandler;
import com.google.gwt.user.client.ui.Button;
import com.google.gwt.user.client.ui.DialogBox;
import com.google.gwt.user.client.ui.HasHorizontalAlignment;
import com.google.gwt.user.client.ui.HorizontalPanel;
/**
 * Dialogbox is used to ask the user if he wants to overwrite or not
 * 
 * @author Raschid
 */
public class DialogBoxOverwrite extends DialogBox {

	private MainFramePresenter presenter;

	// The Dialog needs the reference to the Presenter in order to load it
	public DialogBoxOverwrite(MainFramePresenter presenter) {

		this.presenter = presenter;

		// Set the dialog box's caption.
		setText("Do you want to overwrite the existing Szenario?");

		// Enable animation.
		setAnimationEnabled(true);

		// Enabl glass background.
		setGlassEnabled(true);

		HorizontalPanel vpSzenario = new HorizontalPanel();
		vpSzenario.setHorizontalAlignment(HasHorizontalAlignment.ALIGN_RIGHT);

		// DialogBox is a SimplePanel, so you have to set its widget property to
		// whatever you want its contents to be.

		// Sends the Szenario to the Server in order to overwrite an existing
		// one
		Button yes = new Button("yes");
		yes.addClickHandler(new ClickHandler() {
			public void onClick(ClickEvent event) {
				DialogBoxOverwrite.this.presenter.sendSzenarioToServer(
						DialogBoxOverwrite.this.presenter.getActualSzenario());
				DialogBoxOverwrite.this.hide();
			}
		});

		vpSzenario.add(yes);

		Button no = new Button("No");
		no.addClickHandler(new ClickHandler() {
			public void onClick(ClickEvent event) {
				DialogBoxOverwrite.this.hide();
			}
		});

		vpSzenario.add(no);

		this.setWidget(vpSzenario);
		this.center();
	}

	public MainFramePresenter getPresenter() {
		return presenter;
	}
}
