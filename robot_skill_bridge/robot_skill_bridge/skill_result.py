"""Resultado de dominio; los transportes deciden su representación externa."""

from dataclasses import dataclass, field
from typing import Any


@dataclass
class SkillResult:
    ok: bool
    status: str
    skill: str = ''
    detail: str = ''
    data: dict[str, Any] = field(default_factory=dict)

    def to_dict(self):
        return {**self.data, 'ok': self.ok, 'status': self.status,
                'skill': self.skill, 'detail': self.detail}

    @classmethod
    def from_dict(cls, value):
        value = dict(value)
        return cls(value.pop('ok'), value.pop('status'), value.pop('skill', ''),
                   value.pop('detail', ''), value)
