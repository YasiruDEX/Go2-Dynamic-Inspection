from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Any, Iterable
import random
import threading


@dataclass(frozen=True)
class GeneratorMeta:
    name: str
    func: Callable[..., Any]
    tags: frozenset[str] = field(default_factory=frozenset)
    weight: float = 1.0
    version: str = "v1"
    description: str = ""
    extra: dict = field(default_factory=dict)


class GeneratorRegistry:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._by_name: Dict[str, GeneratorMeta] = {}

    def register(
        self,
        *,
        name: Optional[str] = None,
        tags: Iterable[str] = (),
        weight: float = 1.0,
        version: str = "v1",
        description: str = "",
        **extra,
    ):
        """example: @registry.register(name='clock_simple', tags={'clock'}, weight=1.0)"""

        def deco(func: Callable):
            gen_name = name or func.__name__
            meta = GeneratorMeta(
                name=gen_name,
                func=func,
                tags=frozenset(tags),
                weight=float(weight),
                version=version,
                description=description,
                extra=extra or {},
            )
            with self._lock:
                if gen_name in self._by_name:
                    raise ValueError(f"Duplicate generator name: {gen_name}")
                self._by_name[gen_name] = meta
            return func

        return deco

    def get(self, name: str) -> GeneratorMeta:
        with self._lock:
            return self._by_name[name]

    def list(
        self,
        *,
        include_tags: Iterable[str] = (),
        exclude_tags: Iterable[str] = (),
        version: Optional[str] = None,
        name_prefix: Optional[str] = None,
    ) -> List[GeneratorMeta]:
        inc, exc = set(include_tags), set(exclude_tags)
        with self._lock:
            out = []
            for m in self._by_name.values():
                if inc and not inc.issubset(m.tags):
                    continue
                if exc and (exc & m.tags):
                    continue
                if version and m.version != version:
                    continue
                if name_prefix and not m.name.startswith(name_prefix):
                    continue
                out.append(m)
            return out

    def weighted_choice(
        self, metas: List[GeneratorMeta], rng: random.Random
    ) -> GeneratorMeta:
        weights = [max(m.weight, 0.0) for m in metas]
        if not metas:
            raise ValueError("No generators after filtering.")
        # Python 3.11+ has random.choices cum-weights; this is fine too:
        return rng.choices(metas, weights=weights, k=1)[0]


registry = GeneratorRegistry()
