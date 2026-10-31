"""Generate the RViz dock state using Qt's own saveState format."""
import os
os.environ['QT_QPA_PLATFORM']='offscreen'
from PyQt5.QtWidgets import QApplication,QMainWindow,QWidget,QDockWidget,QToolBar
from PyQt5.QtCore import Qt
from pathlib import Path
import yaml
app=QApplication([])
w=QMainWindow();w.resize(1280,950)
w.setCentralWidget(QWidget())
tools=QToolBar('Tools');tools.setObjectName('Tools');w.addToolBar(tools)
dock=QDockWidget('Frontier and Traversability')
dock.setObjectName('Frontier and Traversability');dock.setWidget(QWidget())
w.addDockWidget(Qt.RightDockWidgetArea,dock)
w.show();app.processEvents()
w.resizeDocks([dock],[960],Qt.Horizontal);app.processEvents()
path=Path('/home/ly/beamng_navigation_frontier.rviz')
config=yaml.safe_load(path.read_text())
config['Window Geometry']['QMainWindow State']=bytes(w.saveState().toHex()).decode()
config['Window Geometry']['Frontier and Traversability']={'collapsed':False}
path.write_text(yaml.safe_dump(config,sort_keys=False))
print(config['Window Geometry'],flush=True)

