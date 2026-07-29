"""Typed HTTP client for any LuxiNav environment backend."""

from __future__ import annotations

from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen
import json
import math
from typing import Any, Mapping

from .backend_protocol import (
    DecisionPayload,
    EpisodeMetrics,
    Observation,
    ProtocolValidationError,
    StepResult,
)


class BackendHttpError(RuntimeError):
    def __init__(self, status: int, payload: Mapping[str, Any]) -> None:
        self.status = status
        self.payload = dict(payload)
        super().__init__(f"{self.payload.get('error', 'http_error')}: {self.payload.get('detail', '')}")


class BackendTransportError(RuntimeError):
    pass


class HttpEnvironmentBackend:
    def __init__(self, base_url: str, timeout_seconds: float = 5.0) -> None:
        if not isinstance(base_url, str) or not base_url.startswith(("http://", "https://")):
            raise ProtocolValidationError("base_url must be an HTTP URL")
        self._base_url = base_url.rstrip("/")
        self._timeout_seconds = self._positive_timeout(timeout_seconds)

    def health(self, *, timeout_seconds: float | None = None) -> dict[str, Any]:
        return self._request("GET", "/health", timeout_seconds=timeout_seconds)

    def reset(
        self,
        run_id: str,
        episode_id: str,
        seed: int,
        max_steps: int,
        *,
        timeout_seconds: float | None = None,
    ) -> Observation:
        return Observation.from_wire(
            self._request(
                "POST",
                "/reset",
                {"run_id": run_id, "episode_id": episode_id, "seed": seed, "max_steps": max_steps},
                timeout_seconds=timeout_seconds,
            )
        )

    def step(
        self,
        decision: DecisionPayload | Mapping[str, Any],
        *,
        timeout_seconds: float | None = None,
    ) -> StepResult:
        payload = decision if isinstance(decision, DecisionPayload) else DecisionPayload.from_wire(decision)
        return StepResult.from_wire(
            self._request(
                "POST", "/step", payload.to_wire(), timeout_seconds=timeout_seconds
            )
        )

    def metrics(self, *, timeout_seconds: float | None = None) -> EpisodeMetrics:
        return EpisodeMetrics.from_wire(
            self._request("GET", "/metrics", timeout_seconds=timeout_seconds)
        )

    def shutdown(self, *, timeout_seconds: float | None = None) -> dict[str, Any]:
        return self._request(
            "POST", "/shutdown", {}, timeout_seconds=timeout_seconds
        )

    def _request(
        self,
        method: str,
        path: str,
        body: Mapping[str, Any] | None = None,
        *,
        timeout_seconds: float | None = None,
    ) -> dict[str, Any]:
        encoded = None if body is None else json.dumps(dict(body), separators=(",", ":")).encode("utf-8")
        request = Request(
            f"{self._base_url}{path}",
            data=encoded,
            method=method,
            headers={"Content-Type": "application/json"} if encoded is not None else {},
        )
        effective_timeout = self._timeout_seconds
        if timeout_seconds is not None:
            effective_timeout = min(
                effective_timeout, self._positive_timeout(timeout_seconds)
            )
        try:
            with urlopen(request, timeout=effective_timeout) as response:
                return self._decode_json(response.read())
        except HTTPError as error:
            payload = self._decode_json(error.read())
            raise BackendHttpError(error.code, payload) from error
        except URLError as error:
            raise BackendTransportError(str(error.reason)) from error
        except TimeoutError as error:
            raise BackendTransportError(str(error)) from error

    @staticmethod
    def _decode_json(data: bytes) -> dict[str, Any]:
        try:
            value = json.loads(data)
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            raise BackendTransportError("backend returned invalid JSON") from error
        if not isinstance(value, dict):
            raise BackendTransportError("backend returned a non-object JSON payload")
        return value

    @staticmethod
    def _positive_timeout(value: Any) -> float:
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or value <= 0.0
        ):
            raise ProtocolValidationError("timeout_seconds must be a positive finite number")
        return float(value)
