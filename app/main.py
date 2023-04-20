import sys
from PyQt5 import QtWidgets

from app.widget import AnimationWidget


def run_app():
    q_app = QtWidgets.QApplication(sys.argv)
    aw = AnimationWidget()
    aw.show()
    sys.exit(q_app.exec_())
