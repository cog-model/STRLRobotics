import logging
import os
import sys
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Dict

import wandb


class BaseLogger(ABC):
    _name: str

    @property
    def name(self):
        return self._name

    def __init__(self, log_dir: str, log_filename: str):
        self._log_dir = log_dir
        self._log_filename = os.path.join(self._log_dir, log_filename)

    @abstractmethod
    def info(self, text: str):
        raise NotImplementedError


class WandbLogger(BaseLogger):
    _name: str = "WandbLogger"

    def __init__(
        self,
        log_filename: Path = Path("run.log"),
        log_dir: str = ".",
        run_name: str = "run_name",
        project_name: str = "wandb_test_llm_planning",
        log_to_stdout: bool = False,
    ):
        super().__init__(log_dir, log_filename)
        logging.basicConfig(
            filename=self._log_filename,
            filemode="w",
        )

        self._logger = logging.getLogger(self._name)
        self._logger.setLevel(logging.DEBUG)

        if log_to_stdout:
            self._logger.addHandler(logging.StreamHandler(sys.stdout))

        self.info(f"Initialized logger for project: '{project_name}'")
        self.info(f"Run name: '{run_name}'")
        self.info("Initializing wandb...")
        self._wandb_logger = wandb.init(
            project=project_name, name=run_name, dir=self._log_dir
        )

    def info(self, msg: str):
        self._logger.info(msg)

    def wandb_log(self, msg: Dict) -> None:
        self._wandb_logger.log(msg)


if __name__ == "__main__":
    project_name = "wandb_test_llm_planning"
    log_filename = "test_logger.log"
    log_to_stdout = True
    run_name = "run_name"
    log_dir = "."
    logger = WandbLogger(
        log_filename=log_filename,
        project_name=project_name,
        log_to_stdout=log_to_stdout,
        run_name=run_name,
        log_dir=log_dir,
    )

    logger.info("hello")
