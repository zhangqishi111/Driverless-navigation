import os

os.environ.setdefault(
    'QT_QPA_PLATFORM',
    'offscreen',
)

from PyQt5.QtWidgets import QApplication

from landerpi_sandbox_display.main_window import MainWindow
from landerpi_sandbox_display.sandbox_display_node import create_main_window


app = QApplication.instance()

if app is None:
    app = QApplication([])


def test_application_creates_main_window():
    window = create_main_window()

    assert isinstance(
        window,
        MainWindow,
    )

    assert window.windowTitle() == (
        'LanderPi Navigation Console'
    )

    window.close()