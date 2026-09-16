"""YAML 파일 기반 설정 로더 구현체."""

from __future__ import annotations

import logging
from typing import Any

from vda5050_fleet_adapter.usecase.ports.config_port import ConfigPort
import yaml

logger = logging.getLogger(__name__)


class YamlConfigLoader(ConfigPort):
    """ConfigPort의 YAML 파일 구현체."""

    def load(
        self, config_path: str, nav_graph_path: str
    ) -> dict[str, Any]:
        """YAML 파일에서 설정을 로드한다."""
        try:
            with open(config_path, encoding='utf-8') as f:
                data = yaml.safe_load(f)
        except FileNotFoundError:
            logger.warning(
                'Config file not found: %s, returning empty dict',
                config_path,
            )
            return {}

        if not isinstance(data, dict):
            logger.warning('Invalid YAML format, returning empty dict')
            return {}

        logger.info('Config loaded from %s', config_path)
        return data
