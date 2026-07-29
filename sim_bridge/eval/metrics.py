"""Shared metrics computation (extracted from habitat_evaluation.py)."""

from dataclasses import dataclass, field
from typing import Dict, List


@dataclass
class EpisodeResult:
    """Per-episode evaluation metrics."""

    episode_id: str = ""
    scene_id: str = ""
    target_label: str = ""
    success: int = 0
    spl: float = 0.0
    soft_spl: float = 0.0
    distance_to_goal: float = 0.0
    steps: int = 0
    result_text: str = ""


def compute_averages(results: List[EpisodeResult]) -> Dict[str, float]:
    """Aggregate a list of per-episode results into average metrics.

    Args:
        results: list of EpisodeResult from one or more episodes.

    Returns:
        Dict with keys: success_rate, avg_spl, avg_soft_spl,
        avg_distance_to_goal, avg_steps.
    """
    n = max(len(results), 1)
    return {
        "success_rate": sum(r.success for r in results) / n,
        "avg_spl": sum(r.spl for r in results) / n,
        "avg_soft_spl": sum(r.soft_spl for r in results) / n,
        "avg_distance_to_goal": sum(r.distance_to_goal for r in results) / n,
        "avg_steps": sum(r.steps for r in results) / n,
        "total_episodes": n,
        "total_success": sum(r.success for r in results),
    }
