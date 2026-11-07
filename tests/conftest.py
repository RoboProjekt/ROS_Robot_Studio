import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("PYTEST_QT_API", "pyside6")

import pytest


def pytest_addoption(parser):
    parser.addoption("--run-docker", action="store_true", default=False)


def pytest_collection_modifyitems(config, items):
    if not config.getoption("--run-docker"):
        for item in items:
            if "integration" in item.keywords:
                item.add_marker(
                    pytest.mark.skip(
                        reason="Explicit --run-docker required; creates only Studio test containers"
                    )
                )
