"""Calibrador web de servos del Robodelta.

Uso (desde la raiz del repositorio):

    python -m webapp.app                 # 1 PCA9685, puerto 5000
    python -m webapp.app --pcas 2        # 2 placas
    python -m webapp.app --port 8080

Luego abre http://<ip-de-la-robot>:5000 desde el navegador.
"""

from __future__ import annotations

import argparse
from functools import wraps

from flask import Flask, current_app, jsonify, render_template, request

from src.utils.logger import log
from webapp.service import ServoService


def _service() -> ServoService:
    return current_app.config["SERVO_SERVICE"]


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
    app.config["SERVO_SERVICE"] = ServoService(num_pcas=num_pcas)

    @app.get("/")
    def index():
        return render_template("index.html")

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

    @app.post("/api/servo/<name>/rest")
    @api
    def post_rest(name: str):
        return _service().set_rest(name, int(_require(_body(), "angle")))

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

    return app


def main() -> None:
    parser = argparse.ArgumentParser(description="Calibrador web de servos")
    parser.add_argument("--pcas", type=int, choices=(1, 2), default=1)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5000)
    args = parser.parse_args()

    app = create_app(num_pcas=args.pcas)
    log.info(f"Calibrador web en http://{args.host}:{args.port} (PCA: {args.pcas})")
    # use_reloader=False: con el reloader habria DOS procesos y dos HardwareManager.
    app.run(host=args.host, port=args.port, threaded=True, use_reloader=False)


if __name__ == "__main__":
    main()
