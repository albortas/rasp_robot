"""Calibrador web + tuning de marcha del Robodelta.

Uso (desde la raiz del repositorio):

    python -m webapp.app                 # 1 PCA9685, puerto 5000
    python -m webapp.app --pcas 2        # 2 placas
    python -m webapp.app --port 8080

Paginas:

    http://<ip>:5000/        calibracion de servos
    http://<ip>:5000/gait    tuning interactivo de la marcha

Aviso: el proceso web es el unico que debe usar el bus I2C. No ejecutes
``src/main.py`` a la vez.
"""

from __future__ import annotations

import argparse
import atexit
from functools import wraps

from flask import Flask, current_app, jsonify, render_template, request

from src.utils.logger import log
from webapp.gait_service import GaitService
from webapp.service import ServoService


def _service() -> ServoService:
    return current_app.config["SERVO_SERVICE"]


def _gait_service() -> GaitService:
    return current_app.config["GAIT_SERVICE"]


def api(fn):
    """Convierte errores en respuestas JSON legibles en vez de HTML."""

    @wraps(fn)
    def wrapper(*args, **kwargs):
        try:
            return jsonify(fn(*args, **kwargs))
        except KeyError as exc:
            return jsonify({"error": f"Servo no encontrado: {exc}"}), 404
        except (TypeError, ValueError) as exc:
            return jsonify({"error": str(exc)}), 400
        except Exception as exc:  # noqa: BLE001
            log.error(f"Error en {request.path}: {exc}")
            return jsonify({"error": str(exc)}), 500

    return wrapper


def _body() -> dict:
    return request.get_json(silent=True) or {}


def _require(body: dict, key: str):
    if key not in body:
        raise ValueError(f"Falta el campo '{key}' en el cuerpo JSON")
    return body[key]


def create_app(num_pcas: int = 1) -> Flask:
    app = Flask(__name__)
    servo_service = ServoService(num_pcas=num_pcas)
    gait_service = GaitService(servo=servo_service)
    app.config["SERVO_SERVICE"] = servo_service
    app.config["GAIT_SERVICE"] = gait_service

    # Si el proceso termina con una marcha activa, intentar parar y volver a
    # neutro. No sustituye al boton Detener, pero evita dejar los servos
    # empujando en la ultima postura.
    atexit.register(gait_service.stop)

    # ---------------------------------------------------------------- paginas
    @app.get("/")
    def index():
        return render_template("index.html")

    @app.get("/gait")
    def gait():
        return render_template("gait.html")

    # ----------------------------------------------------------- calibracion
    @app.get("/api/state")
    @api
    def get_state():
        return _service().state()

    @app.post("/api/servo/<name>/angle")
    @api
    def post_angle(name: str):
        return _service().set_angle(name, float(_require(_body(), "angle")))

    @app.post("/api/servo/<name>/offset")
    @api
    def post_offset(name: str):
        body = _body()
        if "value" in body:
            return _service().set_offset(name, int(body["value"]))
        return _service().change_offset(name, int(_require(body, "delta")))

    @app.post("/api/servo/<name>/invert")
    @api
    def post_invert(name: str):
        return _service().toggle_invert(name)

    @app.post("/api/servo/<name>/test")
    @api
    def post_test(name: str):
        body = _body()
        return _service().test_preset(name, int(body.get("delta", 0)))

    @app.post("/api/rest-pose")
    @api
    def post_rest_pose():
        return _service().rest_pose()

    @app.post("/api/disable")
    @api
    def post_disable():
        return _service().disable()

    @app.post("/api/save")
    @api
    def post_save():
        return _service().save()

    # ------------------------------------------------------------------ marcha
    @app.get("/api/gait/state")
    @api
    def get_gait_state():
        return _gait_service().state()

    @app.post("/api/gait/preview")
    @api
    def post_gait_preview():
        return _gait_service().preview(_body())

    @app.post("/api/gait/start")
    @api
    def post_gait_start():
        return _gait_service().start(_body())

    @app.post("/api/gait/stop")
    @api
    def post_gait_stop():
        return _gait_service().stop()

    @app.post("/api/gait/neutral")
    @api
    def post_gait_neutral():
        return _gait_service().neutral()

    @app.get("/api/gait/status")
    @api
    def get_gait_status():
        return _gait_service().status()

    return app


def main() -> None:
    parser = argparse.ArgumentParser(description="Calibrador web + marcha")
    parser.add_argument("--pcas", type=int, choices=(1, 2), default=1)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()

    app = create_app(num_pcas=args.pcas)
    log.info(f"Web Robodelta en http://{args.host}:{args.port} (PCA: {args.pcas})")
    log.info(f"Calibracion: http://{args.host}:{args.port}/  Marcha: /gait")
    # use_reloader=False: con el reloader habria DOS procesos y dos HardwareManager.
    app.run(
        host=args.host,
        port=args.port,
        threaded=True,
        use_reloader=False,
        debug=args.debug,
    )


if __name__ == "__main__":
    main()
