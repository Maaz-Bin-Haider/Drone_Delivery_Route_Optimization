"""Flask application factory (TDD section 3.1).

The web tier owns no logic. It loads a scenario, hands it to a Simulator and
exposes that over HTTP; every routing and scheduling decision is made below it.
"""

from __future__ import annotations

from pathlib import Path

from flask import Flask, render_template

from ..domain.loader import load_scenario
from ..simulation.orchestrator import Simulator
from .api import api

ROOT = Path(__file__).resolve().parent.parent.parent
DEFAULT_DATA = ROOT / "data"


def create_app(data_dir: str | Path = DEFAULT_DATA) -> Flask:
    app = Flask(__name__, static_folder="static", template_folder="templates")
    app.config["SIMULATOR"] = Simulator(load_scenario(data_dir))
    app.config["LAST_PLAN"] = None
    app.config["JSON_SORT_KEYS"] = False
    app.register_blueprint(api)

    @app.get("/")
    def index():
        return render_template("index.html",
                               scenario=app.config["SIMULATOR"].scenario.name)

    return app


def main(host: str = "127.0.0.1", port: int = 5000, debug: bool = False) -> None:
    """Bind to the loopback interface by default, per NFR-13."""
    create_app().run(host=host, port=port, debug=debug)
