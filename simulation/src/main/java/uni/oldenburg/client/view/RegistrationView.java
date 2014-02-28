package uni.oldenburg.client.view;

import com.google.gwt.event.dom.client.HasClickHandlers;
import com.google.gwt.user.client.ui.*;
import uni.oldenburg.client.presenter.RegistrationPresenter;

public class RegistrationView extends Composite implements RegistrationPresenter.IDisplay {
    private TextBox txtEmail;
    private TextBox txtName;
    private PasswordTextBox txtPassword;
    private Button btnRegister;

    public RegistrationView() {
        Grid grid = new Grid(4, 2);
        grid.setWidget(0, 0, new Label("E-Mail: "));
        grid.setWidget(0, 1, createEmailInput());

        grid.setWidget(1, 0, new Label("Name: "));
        grid.setWidget(1, 1, createUserInput());

        grid.setWidget(2, 0, new Label("Passwort: "));
        grid.setWidget(2, 1, createPasswordInput());

        grid.setWidget(3, 1, createRegisterButton());

        VerticalPanel vpLogin = new VerticalPanel();
        vpLogin.add(grid);
        initWidget(vpLogin);
    }

    private Button createRegisterButton() {
        btnRegister = new Button("Registrieren");
        btnRegister.setText("Registrieren");
        btnRegister.setSize("100px", "30px");

        return btnRegister;
    }

    private IsWidget createUserInput() {
        txtName = new TextBox();
        txtName.setPixelSize(200, 30);
        return txtName;
    }

    private IsWidget createEmailInput() {
        txtEmail = new TextBox();
        txtEmail.setPixelSize(200, 30);
        return txtEmail;
    }

    private IsWidget createPasswordInput() {
        txtPassword = new PasswordTextBox();
        txtPassword.setPixelSize(200, 30);
        return txtPassword;
    }

    public HasValue<String> getPassword() {
        return txtPassword;
    }

    public HasValue<String> getEmail() {
        return txtEmail;
    }

    public HasValue<String> getName() {
        return txtName;
    }

    public HasClickHandlers getRegisterButton() {
        return btnRegister;
    }
}
