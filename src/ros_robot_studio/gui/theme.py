DARK = """
QWidget { background: #10151f; color: #dce4ef; font-family: "DejaVu Sans"; font-size: 12px; }
QMainWindow, QDialog, QWizard { background: #10151f; }
QMenuBar, QToolBar, QStatusBar { background: #151c28; border: none; padding: 5px; }
QMenuBar::item:selected, QMenu::item:selected { background: #28364b; }
QMenu { background: #1b2433; padding: 6px; border: 1px solid #334155; }
QLabel { background: transparent; }
QLabel#eyebrow { color: #50d6ba; font-size: 10px; font-weight: bold; }
QLabel#title { font-size: 27px; font-weight: bold; color: #f4f7fb; }
QLabel#subtitle { color: #8c9bb0; font-size: 12px; }
QLabel#section { font-weight: bold; color: #ecf1f8; font-size: 14px; }
QLabel#badge { color: #53ddbd; background: #16382f; padding: 7px 12px; border-radius: 5px; }
QLabel#warning { color: #efc77e; background: #342c1d; padding: 10px; border-radius: 5px; }
QFrame#card { background: #171f2c; border: 1px solid #283447; border-radius: 8px; }
QTabWidget::pane { border: none; }
QTabBar { background: #10151f; }
QTabBar::tab { background: #10151f; color: #92a0b4; padding: 15px 23px; border-bottom: 2px solid #283447; }
QTabBar::tab:selected { color: #61e0c5; border-bottom: 2px solid #50d6ba; }
QTabBar::tab:hover { color: #f4f7fb; background: #1b2433; }
QPushButton, QToolButton { background: #253247; border: 1px solid #35455c; border-radius: 5px; padding: 8px 13px; }
QPushButton:hover, QToolButton:hover { background: #30435a; border-color: #5a728f; }
QPushButton:disabled, QToolButton:disabled { color: #5a697d; background: #1c2431; border-color: #283447; }
QPushButton#primary { background: #43c7ac; color: #0b211c; font-weight: bold; border-color: #43c7ac; }
QPushButton#primary:hover { background: #63e1c5; }
QLineEdit, QTextEdit, QPlainTextEdit, QSpinBox, QDoubleSpinBox, QComboBox { background: #111925; border: 1px solid #344257; border-radius: 4px; padding: 7px; selection-background-color: #23665e; }
QLineEdit:focus, QTextEdit:focus, QComboBox:focus { border-color: #50d6ba; }
QComboBox::drop-down { border: none; width: 22px; }
QComboBox QAbstractItemView { background: #1b2433; selection-background-color: #2c4356; }
QTreeWidget, QListWidget, QTableWidget { background: #131c29; alternate-background-color: #192433; border: 1px solid #2b394d; border-radius: 5px; outline: none; }
QTreeWidget::item, QListWidget::item, QTableWidget::item { padding: 7px 5px; border: none; }
QTreeWidget::item:selected, QListWidget::item:selected, QTableWidget::item:selected { background: #24483f; color: #86f2d8; }
QHeaderView::section { background: #202b3d; color: #9babbe; border: none; padding: 9px; font-size: 10px; }
QSplitter::handle { background: #10151f; width: 8px; height: 8px; }
QSplitter::handle:hover { background: #345765; }
QScrollBar:vertical { background: #151e2b; width: 10px; }
QScrollBar::handle:vertical { background: #35445a; min-height: 25px; border-radius: 4px; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
QProgressBar { border: 1px solid #344257; border-radius: 4px; background: #111925; text-align: center; }
QProgressBar::chunk { background: #329f8b; }
QCheckBox { spacing: 8px; background: transparent; }
QCheckBox::indicator { width: 16px; height: 16px; }
QSlider::groove:horizontal { height: 4px; background: #324257; border-radius: 2px; }
QSlider::handle:horizontal { background: #50d6ba; width: 12px; margin: -5px 0; border-radius: 6px; }
QToolTip { color: #edf6ff; background: #28384c; padding: 8px; border: 1px solid #4e6580; }
"""

LIGHT = (
    DARK.replace("#10151f", "#edf2f7")
    .replace("#151c28", "#e2e9f2")
    .replace("#171f2c", "#ffffff")
    .replace("#131c29", "#ffffff")
    .replace("#111925", "#f8fafc")
    .replace("#1b2433", "#e0e8f2")
    .replace("#202b3d", "#dce5ef")
    .replace("#dce4ef", "#203047")
    .replace("#f4f7fb", "#15273c")
    .replace("#ecf1f8", "#22354b")
    .replace("#8c9bb0", "#53677e")
    .replace("#92a0b4", "#53677e")
    .replace("#253247", "#dce7f2")
    .replace("#9babbe", "#415773")
    .replace("#283447", "#c8d4e3")
    .replace("#344257", "#b1c1d3")
)
