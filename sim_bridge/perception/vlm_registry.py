"""VLM Registry — pluggable perception models.

Supports: full (GroundingDINO+YOLOv7+SAM+BLIP2), passthrough, individual models.
"""

from dataclasses import dataclass, field
from typing import Any, Dict, Optional, TYPE_CHECKING

if TYPE_CHECKING:
    import numpy as np


@dataclass
class VLMOutput:
    """Unified output from ANY perception model."""

    cosine_score: float = 0.0
    boxes: list = field(default_factory=list)
    scores: list = field(default_factory=list)
    masks: list = field(default_factory=list)
    labels: list = field(default_factory=list)


class VLMPipeline:
    """Wraps the VLM call chain (DINO → YOLO → SAM → BLIP2).

    Args:
        name: 'full' or 'passthrough'.
        detector_cfg: Hydra/OmegaConf detector config section.
        target_label: object category to search for (e.g. 'chair').
        llm_answer: LLM-generated misdetection list.
        room: most-likely room for ITM context.
    """

    def __init__(
        self,
        name: str,
        detector_cfg: Any,
        target_label: str,
        llm_answer: str = "",
        room: str = "",
    ):
        self.name = name
        self._cfg = detector_cfg
        self._label = target_label
        self._llm_answer = llm_answer
        self._room = room

    def process_frame(self, rgb: "np.ndarray", depth: "np.ndarray") -> VLMOutput:
        """Run VLM perception on one frame.  Same call chain as habitat_evaluation.py."""
        if self.name == "passthrough":
            return VLMOutput()
        return self._run_full_vlm(rgb, depth)

    def _run_full_vlm(self, rgb: "np.ndarray", depth: "np.ndarray") -> VLMOutput:
        """Full stack: BLIP2 ITM + GroundingDINO/YOLOv7 + MobileSAM."""
        result = VLMOutput()

        # BLIP2 ITM cosine score
        try:
            from vlm.utils.get_itm_message import get_itm_message_cosine

            result.cosine_score = get_itm_message_cosine(
                rgb, self._label, self._room
            )
        except Exception:
            pass

        # GroundingDINO + YOLOv7 detection + SAM segmentation
        try:
            from vlm.utils.get_object_utils import get_object

            rgb_out, scores, masks, labels = get_object(
                self._label, rgb, self._cfg, self._llm_answer
            )
            result.boxes = []
            result.scores = list(scores)
            result.masks = list(masks)
            result.labels = list(labels)
        except Exception:
            pass

        return result


# ── Factory ──────────────────────────────────────────────────────────────

_VLM_REGISTRY: Dict[str, type] = {}


def register_vlm(name: str):
    """Decorator: register a VLM pipeline class."""
    def decorator(cls):
        _VLM_REGISTRY[name] = cls
        return cls
    return decorator


def get_vlm_pipeline(name: str = "full", **kwargs) -> VLMPipeline:
    """Factory: create a VLM pipeline by name."""
    if name in _VLM_REGISTRY:
        return _VLM_REGISTRY[name](name=name, **kwargs)
    return VLMPipeline(name=name, **kwargs)


def list_vlm_pipelines() -> list:
    """Return all registered VLM pipeline names."""
    return sorted(_VLM_REGISTRY.keys())
