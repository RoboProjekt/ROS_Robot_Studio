"""Ownership-aware disposal for the pinned PyQtGraph 0.13.7 PlotWidget."""

import shiboken6
from PySide6.QtCore import QObject


def close_plot(plot):
    if not shiboken6.isValid(plot) or plot.plotItem is None:
        return
    item = plot.plotItem
    view = item.getViewBox()
    menus = [item.ctrlMenu, view.menu]
    graphics = [obj for obj in plot.scene().items() if isinstance(obj, QObject)]
    # PlotItem.close removes axes and ViewBox from its scene and detaches the
    # options menu; PlotWidget.close also detaches itself from its QWidget parent.
    # Keep those objects until Qt can delete them in the GUI event loop.
    view.close()  # Clear data and unregister the ViewBox's global link registry.
    plot.close()
    for obj in (*menus, *graphics, plot):
        if obj is not None and shiboken6.isValid(obj):
            obj.deleteLater()
