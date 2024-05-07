import requests

from llm_sorter import WandbLogger
from llm_sorter.models import (
    BaseLLMModel,
    BaseModelInput,
    BaseModelOutput,
    ScoringInput,
    ScoringOutput,
)


class APIModel(BaseLLMModel):
    def __init__(
        self,
        logger: WandbLogger,
        name: str = "vicuna_7b docker",
        url: str = "http://127.0.0.1:8082",
    ) -> None:
        super().__init__(name=name, logger=logger)
        self._url = url

    def generate(self, inputs: BaseModelInput, **kwargs) -> BaseModelOutput:
        url = f"{self._url}generate"

        inputs = {"prompt": inputs.text}

        outputs = requests.post(url, json=inputs)

        # Get the response
        if outputs.status_code == 200:  # Successful request
            out = BaseModelOutput(outputs.json()["text"])
            return out
        else:
            self._logger.info(f"Error: {outputs.status_code}")
            return BaseModelOutput("")

    def score_text(self, inputs: ScoringInput, **kwargs) -> ScoringOutput:
        pass
