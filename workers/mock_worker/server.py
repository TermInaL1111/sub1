"""HTTP boundary for the deterministic mock environment worker."""

from __future__ import annotations

from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import threading
from typing import Any

from luxinav_sim.backend_protocol import DecisionPayload, ProtocolValidationError

from .state_machine import MockStateMachine, SessionConflictError


class MockWorkerServer(ThreadingHTTPServer):
    def __init__(self, address: tuple[str, int], state_machine: MockStateMachine | None = None) -> None:
        super().__init__(address, MockWorkerHandler)
        self.state_machine = state_machine or MockStateMachine()


class MockWorkerHandler(BaseHTTPRequestHandler):
    server: MockWorkerServer

    def do_GET(self) -> None:  # noqa: N802
        if self.path == "/health":
            self._respond(HTTPStatus.OK, {"status": "ok"})
        elif self.path == "/metrics":
            self._dispatch(self.server.state_machine.metrics)
        else:
            self._error(HTTPStatus.NOT_FOUND, "not_found", "unknown endpoint")

    def do_POST(self) -> None:  # noqa: N802
        if self.path == "/reset":
            self._dispatch(self._reset)
        elif self.path == "/step":
            self._dispatch(self._step)
        elif self.path == "/shutdown":
            self._respond(HTTPStatus.OK, {"status": "shutting_down"})
            threading.Thread(target=self.server.shutdown, daemon=True).start()
        else:
            self._error(HTTPStatus.NOT_FOUND, "not_found", "unknown endpoint")

    def _reset(self) -> dict[str, Any]:
        body = self._json_body()
        return self.server.state_machine.reset(
            body.get("run_id"), body.get("episode_id"), body.get("seed"), body.get("max_steps")
        ).to_wire()

    def _step(self) -> dict[str, Any]:
        body = self._json_body()
        for field in ("run_id", "episode_id", "frame_id"):
            if field not in body:
                raise SessionConflictError(
                    "session_mismatch",
                    f"decision {field} is required",
                    self._expected_frame_id(),
                )
        return self.server.state_machine.step(DecisionPayload.from_wire(body)).to_wire()

    def _json_body(self) -> dict[str, Any]:
        try:
            size = int(self.headers.get("Content-Length", ""))
        except ValueError as error:
            raise ProtocolValidationError("Content-Length must be an integer") from error
        try:
            value = json.loads(self.rfile.read(size))
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            raise ProtocolValidationError("request body must be JSON") from error
        if not isinstance(value, dict):
            raise ProtocolValidationError("request body must be a JSON object")
        return value

    def _dispatch(self, operation) -> None:
        try:
            self._respond(HTTPStatus.OK, operation())
        except SessionConflictError as error:
            self._error(HTTPStatus.CONFLICT, error.error, error.detail, error.expected_frame_id)
        except ProtocolValidationError as error:
            self._error(HTTPStatus.BAD_REQUEST, "invalid_request", str(error), self._expected_frame_id())

    def _expected_frame_id(self) -> int | None:
        return getattr(self.server.state_machine, "_frame_id", None)

    def _respond(self, status: HTTPStatus, payload: dict[str, Any]) -> None:
        encoded = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    def _error(
        self, status: HTTPStatus, error: str, detail: str, expected_frame_id: int | None = None
    ) -> None:
        self._respond(
            status,
            {"error": error, "detail": detail, "expected_frame_id": expected_frame_id},
        )

    def log_message(self, format: str, *args: Any) -> None:
        return


def create_server(
    host: str = "127.0.0.1", port: int = 0, state_machine: MockStateMachine | None = None
) -> MockWorkerServer:
    return MockWorkerServer((host, port), state_machine)


def main() -> None:
    server = create_server()
    try:
        host, port = server.server_address[:2]
        print(json.dumps({"host": host, "port": port}), flush=True)
        server.serve_forever()
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
