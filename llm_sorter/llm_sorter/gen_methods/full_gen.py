from llm_sorter import WandbLogger
from llm_sorter.datasets import BaseTask
from llm_sorter.gen_methods import BasePlanGeneration
from llm_sorter.models import BaseLLMModel, BaseModelInput, BaseModelOutput
from llm_sorter.processors import BaseProcessor


class FullPlanGeneration(BasePlanGeneration):
    """A generation method that generates a
    complete plan in a single query to a language model"""

    def __init__(self, model: BaseLLMModel, logger: WandbLogger, **kwargs):
        super().__init__(model, logger, **kwargs)

    def predict(self, gt_task: BaseTask, processor: BaseProcessor) -> BaseTask:
        """Predict a set of steps for a given task using a language model"""
        inputs: BaseModelInput = processor.to_inputs(gt_task)
        model_ouputs: BaseModelOutput = self._model.generate(inputs)
        # print(model_ouputs.text)
        output_task: BaseTask = processor.to_task(model_ouputs)
        return output_task
