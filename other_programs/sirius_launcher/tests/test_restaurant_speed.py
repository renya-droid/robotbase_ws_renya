from PySide6.QtWidgets import QApplication, QInputDialog
from sirius_launcher import RobotLauncher


def test_restaurant_speed_selection_and_cancel(monkeypatch):
    app = QApplication.instance() or QApplication([])
    window = RobotLauncher()
    button = window.button_map['koko_restaurant_medium']
    answers = iter([('2倍速', True), ('等倍（1倍）', True), ('', False)])

    def choose(parent, title, prompt, labels, default, editable):
        assert labels == ['等倍（1倍）', '2倍速']
        assert default == 0
        return next(answers)

    monkeypatch.setattr(QInputDialog, 'getItem', choose)
    assert button.prepare_launch(button)
    assert button.process_manager.command == button.command + ' speed:=2'
    assert button.prepare_launch(button)
    assert button.process_manager.command == button.command + ' speed:=1'
    assert not button.prepare_launch(button)
    window.close()
