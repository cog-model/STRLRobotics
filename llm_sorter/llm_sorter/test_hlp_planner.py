import random
import time

import hydra
from hydra.core.config_store import ConfigStore
from hydra.utils import instantiate

from llm_sorter import WandbLogger
from llm_sorter.datasets import HLPDataset
from llm_sorter.gen_methods import BasePlanGeneration
from llm_sorter.infrastructure import TestHLPConfig
from llm_sorter.metrics import BaseTaskMetrics
from llm_sorter.models import BaseLLMModel
from llm_sorter.processors import BaseProcessor


cs = ConfigStore.instance()
cs.store(name="hlp_planner", node=TestHLPConfig)


@hydra.main(version_base=None, config_path=".", config_name="hlp_planner")
def test_llp_planner(cfg: TestHLPConfig) -> None:
    logger: WandbLogger = instantiate(cfg.logger)
    dataset: HLPDataset = instantiate(cfg.dataset, logger=logger)
    processor: BaseProcessor = instantiate(cfg.processor, logger=logger)
    if not cfg.processor.load_prompt_from_file:
        processor.build_system_prompt(
            [
                dataset[i]
                for i in random.choices(range(len(dataset)), k=cfg.processor.n_examples)
            ]
        )
    model: BaseLLMModel = instantiate(cfg.model, logger=logger)
    metrics: BaseTaskMetrics = instantiate(
        cfg.metrics, processor=processor, logger=logger
    )
    gen_method: BasePlanGeneration = instantiate(
        cfg.gen_method, model=model, logger=logger
    )

    shuffled_indices = list(range(len(dataset)))
    random.shuffle(shuffled_indices)

    for i, idx in enumerate(shuffled_indices):
        gt_task = dataset[idx]
        # gt_task.goal = 'remove the red apple from the table in the drawer'
        try:
            start = time.time()
            predicted_task = gen_method.predict(gt_task, processor=processor)
            end = time.time()
            d = end - start
        except Exception as e:
            curr_metrics = metrics.update_error()

            logger.info(f"{i + 1}\\{len(dataset)}. Plan id:\t{gt_task.plan_id}")
            logger.info(f"Goal:           {gt_task.goal}")
            logger.info(f"Objects:        {gt_task.feedback.items}")
            logger.info(
                f"GT plan:        \n{processor._low_level_tasks_to_text(gt_task.subtasks)}"
            )
            logger.info(f"Metrics:        {curr_metrics}")
            logger.info(f"Exception:      {e}\n\n")
        else:
            elapsed_time = metrics.update_time(d)
            curr_metrics = metrics.update(
                predicted_task=predicted_task, target_task=gt_task
            )

            logger.info(f"{i + 1}\\{len(dataset)}. Plan id:\t{gt_task.plan_id}")
            logger.info(f"Goal:           {gt_task.goal}")
            logger.info(f"Objects:        {gt_task.feedback.items}")
            logger.info(
                f"GT plan:        \n{processor._low_level_tasks_to_text(gt_task.subtasks)}"
            )
            logger.info(f"Predicted plan: \n{predicted_task.text}")
            logger.info(f"Generation_time:{elapsed_time}")
            logger.info(f"Metrics:        {curr_metrics}\n\n")

    # total_metrics = {key: f'{value:0.3f}' for key, value in total_metrics.items()}
    total_metrics = metrics.calculate_metrics()

    logger.info(f"Total_metrics:  {total_metrics}")
    logger.wandb_log(total_metrics)
    logger.info("Done.")

    return None


if __name__ == "__main__":
    test_llp_planner()
