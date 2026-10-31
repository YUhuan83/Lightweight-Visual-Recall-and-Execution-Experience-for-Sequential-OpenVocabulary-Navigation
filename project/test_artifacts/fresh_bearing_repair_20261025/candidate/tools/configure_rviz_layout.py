"""Generate/verify RViz's native Qt dock state without ROS or a display server.

Qt is needed only for this offline tool; the saved .rviz file is consumed by
ordinary RViz. The optional preview is a layout fixture, not a simulation.
"""
import argparse
import json
import os
from pathlib import Path
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
try:
    from PySide6.QtCore import Qt,QByteArray
    from PySide6.QtGui import QColor,QPainter,QPen,QPixmap,QFont,QFontDatabase
    from PySide6.QtWidgets import QApplication,QMainWindow,QWidget,QDockWidget,QLabel,QToolBar,QSizePolicy
except ImportError:
    from PyQt5.QtCore import Qt,QByteArray
    from PyQt5.QtGui import QColor,QPainter,QPen,QPixmap,QFont,QFontDatabase
    from PyQt5.QtWidgets import QApplication,QMainWindow,QWidget,QDockWidget,QLabel,QToolBar,QSizePolicy

ROOT=Path(__file__).resolve().parents[1]
APP=None


class LayoutScene(QWidget):
    def paintEvent(self,event):
        painter=QPainter(self);painter.fillRect(self.rect(),QColor('#303030'))
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setPen(QPen(QColor('#45494d'),1))
        for x in range(0,self.width(),40):painter.drawLine(x,0,x,self.height())
        for y in range(0,self.height(),40):painter.drawLine(0,y,self.width(),y)
        points=[(.18,.72),(.3,.58),(.46,.48),(.61,.33),(.77,.29)]
        pixel=[(int(x*self.width()),int(y*self.height())) for x,y in points]
        painter.setPen(QPen(QColor('#8fa1b6'),3))
        for a,b in zip(pixel,pixel[1:]):painter.drawLine(*a,*b)
        painter.setPen(QPen(QColor('#ffd23c'),5))
        for a,b in zip(pixel[:3],pixel[1:4]):painter.drawLine(*a,*b)
        for index,(x,y) in enumerate(pixel):
            painter.setBrush(QColor('#ffac29') if index>=3 else QColor('#36df7a'))
            painter.setPen(Qt.NoPen);painter.drawEllipse(x-6,y-6,12,12)
        painter.setPen(QColor('#efefef'))
        painter.drawText(24,34,'OFFLINE LAYOUT FIXTURE - not an RViz / arrival acceptance')
        painter.drawText(24,58,'nodes  |  orange: frontier  |  yellow: route')
        painter.drawText(pixel[-1][0]-90,pixel[-1][1]-20,'object label (fixture)')
        painter.drawText(24,self.height()-25,'XY top-down navigation scene')


def make_window(preview=False):
    global APP
    APP=QApplication.instance() or QApplication([])
    APP.setStyle('Fusion')
    font=Path('C:/Windows/Fonts/segoeui.ttf')
    if font.exists():
        registered=QFontDatabase.addApplicationFont(str(font))
        families=QFontDatabase.applicationFontFamilies(registered)
        if families:APP.setFont(QFont(families[0],10))
    window=QMainWindow();window.setObjectName('VisualizationFrame');window.resize(1920,1080)
    central=LayoutScene() if preview else QWidget();central.setMinimumSize(0,0)
    window.setCentralWidget(central)
    toolbar=QToolBar('Tools');toolbar.setObjectName('Tools');window.addToolBar(toolbar)
    dock=QDockWidget('RGB + Target',window);dock.setObjectName('RGB + Target')
    dock.setMinimumSize(0,0);label=QLabel('Live front camera + fresh target mask')
    label.setAlignment(Qt.AlignCenter);label.setStyleSheet('background:#191919;color:#eeeeee')
    label.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Ignored)
    if preview:
        frame=ROOT/'test_artifacts/tracking_contract_repair_20261014/green_house_dds_trial1/failure_last_rgb.png'
        if frame.exists():label.setPixmap(QPixmap(str(frame)).scaled(950,900,Qt.KeepAspectRatio,Qt.SmoothTransformation))
    dock.setWidget(label);window.addDockWidget(Qt.RightDockWidgetArea,dock)
    window.show();APP.processEvents();window.resizeDocks([dock],[957],Qt.Horizontal);APP.processEvents()
    return window,dock


def check_state(state):
    window,dock=make_window()
    try:
        if not window.restoreState(QByteArray.fromHex(state.encode())):raise ValueError('Qt rejected RViz dock state')
        APP.processEvents()
        total=window.centralWidget().width()+dock.width()
        ratio=dock.width()/total
        if window.dockWidgetArea(dock)!=Qt.RightDockWidgetArea or not .45<=ratio<=.55:
            raise ValueError('Expected right camera pane with half-width layout')
        return dict(restored=True,camera_on_right=True,camera_width=dock.width(),
                    map_width=window.centralWidget().width(),camera_fraction=ratio,
                    scope='Qt dock layout only; actual RViz/Ogre live rendering pending')
    finally:window.close()


def main():
    import yaml
    parser=argparse.ArgumentParser();parser.add_argument('--check',action='store_true')
    parser.add_argument('--preview',type=Path);parser.add_argument('--report',type=Path)
    args=parser.parse_args();path=ROOT/'nav2_adapter/config/beamng_navigation.rviz'
    data=yaml.safe_load(path.read_text(encoding='utf8'))
    if not args.check:
        window,dock=make_window()
        state=bytes(window.saveState().toHex()).decode();window.close()
        data['Window Geometry']={'Width':1920,'Height':1080,'X':0,'Y':0,
            'Hide Left Dock':False,'Hide Right Dock':False,'RGB + Target':{'collapsed':False},
            'QMainWindow State':state}
        path.write_bytes(yaml.safe_dump(data,sort_keys=False,allow_unicode=True).encode())
    report=check_state(data['Window Geometry']['QMainWindow State'])
    if args.preview:
        window,dock=make_window(preview=True)
        try:
            window.restoreState(QByteArray.fromHex(data['Window Geometry']['QMainWindow State'].encode()))
            APP.processEvents();args.preview.parent.mkdir(parents=True,exist_ok=True)
            if not window.grab().save(str(args.preview)):raise RuntimeError('Preview save failed')
        finally:window.close()
    if args.report:
        args.report.parent.mkdir(parents=True,exist_ok=True);args.report.write_text(json.dumps(report,indent=2))
    print(json.dumps(report))

if __name__=='__main__':main()
